// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lightning charging for Apple devices with a Tristar (CBTL1610) port
 * controller and a D2365-family PMIC.
 *
 * iBoot leaves the PMIC input-current limit at 100 mA. On every attachment
 * this driver identifies the source and grants what it advertises, up to the
 * board maximum:
 *   USB-C cable: CC level on the orientation-selected lane (1.5 A / 3 A)
 *   USB-A cable: Apple D+/D- divider (BrickID) on both Tristar mux banks
 *   BC1.2 charging port (CDP or DCP) on the oriented USB pair: 1.5 A
 *   USB host: what the USB device controller reports through vbus_draw
 *             above 500 mA (an Apple host's current grant)
 * Anything unidentified gets the fallback, the 500 mA of a USB 2.0 port.
 * The mux is not sampled while a USB session is active: mux changes have
 * stalled USB data. A host's own advertisement (CDP, USB-C current) measured
 * after attach but before its session began is kept for that session.
 *
 * A source change does not reset the limit in hardware. The PMIC's event
 * controller runs the handler of its PMU line before it reads the event
 * block; that handler drops a grant to the fallback. The events that can mean
 * a source change then start a new identification; when none was latched,
 * the grant is ramped back. ADC completion is an event of its own.
 *
 * Copyright The Orchard Linux contributors
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* PMIC registers, relative to their banks */
#define TRISTAR_CHG_STATUS		0x0a	/* status bank */
#define TRISTAR_CHG_STATUS_ONLINE	0x42
#define TRISTAR_CHG_INPUT_LIMIT		0x03	/* control bank */
#define TRISTAR_CHG_ADC_SELECT		0x00	/* ADC bank */
#define TRISTAR_CHG_ADC_RESULT		0x01

/* Interrupts: the PMU line, ADC completion, then the source-change events. */
#define TRISTAR_IRQ_LINE		0
#define TRISTAR_IRQ_ADC			1
#define TRISTAR_IRQ_SOURCE		2
#define TRISTAR_IRQS_MAX		16

/* Input-current limit encoding: mA = code * 12.5 + 75 */
#define TRISTAR_CHG_HANDOFF_CODE	0x02	/* 100 mA, as iBoot leaves it */
#define TRISTAR_CHG_FALLBACK_CODE	0x22	/* 500 mA */

/* Tristar (CBTL1610) registers */
#define CBTL_MUX_ROUTE			0x01
#define CBTL_MUX_STATE			0x03	/* also the BC1.2 excitation */
#define CBTL_CAP_STATUS			0x0c
#define CBTL_CAP_RESPONSE		0x20
#define CBTL_CAP_LEN			8

#define TRISTAR_POLL_MS			100
#define TRISTAR_SETTLE_MS		1000
/* First look at the source, before a host has started its USB session. */
#define TRISTAR_EARLY_MS		300
#define TRISTAR_RAMP_STEP_MS		500
#define TRISTAR_RAMP_STEP_MA		500
#define TRISTAR_MAX_REDETECT		3
/* A half-inserted plug reads as no signature. */
#define TRISTAR_UNIDENTIFIED_RETRIES	2
#define TRISTAR_RETRY_MS		3000
/* More source events than this per second means something is stuck. */
#define TRISTAR_IRQ_STORM		200
/* After a line withdrawal, the time its events take to be dispatched. */
#define TRISTAR_LINE_SETTLE_MS		200

enum tristar_phase {
	TRISTAR_EMPTY,
	TRISTAR_SETTLING,
	TRISTAR_RAMPING,
	TRISTAR_CHARGING,
	TRISTAR_FAULT,
};

/* Capability-frame routes, as decoded by the Tristar firmware */
enum cbtl_route {
	CBTL_ROUTE_NONE,
	CBTL_ROUTE_USB0,
	CBTL_ROUTE_UART0,
	CBTL_ROUTE_MBUS,
	CBTL_ROUTE_USBC,
	CBTL_ROUTE_UART2,
};

struct tristar_charger {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *tristar;
	struct power_supply *psy;
	struct delayed_work poll_work;
	struct completion adc_done;
	int irqs[TRISTAR_IRQS_MAX];
	unsigned int nirqs;
	u32 status_base;
	u32 control_base;
	u32 adc_base;
	unsigned int max_ma;

	/* Serializes policy state and PMIC/Tristar sequences. */
	struct mutex lock;

	/*
	 * Event attribution: a new epoch invalidates measurements and grants.
	 * Written only by the event handlers, which the PMIC event controller
	 * runs one after another from one thread.
	 */
	unsigned int epoch;
	bool adc_waiting;
	u64 storm_start_ns;
	unsigned int storm_count;
	/* Last withdrawal by the line handler; line_seen once the poll handled it. */
	u64 line_ns;
	u64 line_seen;

	enum tristar_phase phase;
	bool online;
	bool present;
	bool stopping;
	unsigned int code;	/* last verified input limit */
	unsigned int target_code;
	unsigned int source_ma;
	/* What the USB host allows (vbus_draw), valid for usb_epoch only. */
	unsigned int usb_ma;
	unsigned int usb_epoch;
	/* A host's advertisement measured before its USB session began. */
	bool early_done;
	unsigned int early_epoch;
	unsigned int early_ma;
	enum power_supply_usb_type early_type;
	enum power_supply_usb_type usb_type;
	char source[96];
	u64 attach_ns;
	u64 step_ns;
	u64 guard_ns;
	unsigned int redetects;
	unsigned int retries;
	unsigned int epoch_seen;
};

static unsigned int tristar_code_ma(unsigned int code)
{
	return code * 25 / 2 + 75;
}

/* Round down: quantization must never exceed a permitted current. */
static unsigned int tristar_ma_code(unsigned int ma)
{
	return ma <= 75 ? 0 : (ma - 75) * 2 / 25;
}

static int tristar_set_code(struct tristar_charger *chg, unsigned int code)
{
	u32 reg = chg->control_base + TRISTAR_CHG_INPUT_LIMIT;
	unsigned int readback;
	int ret;

	ret = regmap_write(chg->regmap, reg, code);
	if (!ret)
		ret = regmap_read(chg->regmap, reg, &readback);
	if (!ret && readback != code)
		ret = -EIO;
	if (!ret)
		WRITE_ONCE(chg->code, code);
	return ret;
}

static void tristar_withdraw(struct tristar_charger *chg, unsigned int code)
{
	if (READ_ONCE(chg->code) > code && tristar_set_code(chg, code))
		dev_crit(chg->dev, "Withdrawal write failed; input limit unverified\n");
}

/* ---- Battery ---- */

struct tristar_pack {
	int capacity;
	int temp;
	int voltage_uv;
	int current_ua;
};

static int tristar_read_pack(struct tristar_pack *pack)
{
	struct power_supply *battery = power_supply_get_by_name("battery");
	union power_supply_propval val;
	int ret;

	if (!battery)
		return -ENODEV;
	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret && val.intval != 1)
		ret = -ENODEV;
	if (!ret)
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) {
		pack->capacity = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_TEMP, &val);
	}
	if (!ret) {
		pack->temp = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	}
	if (!ret) {
		pack->voltage_uv = val.intval;
		ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	}
	if (!ret)
		pack->current_ua = val.intval;
	power_supply_put(battery);
	return ret;
}

/* Grants above the fallback are allowed only inside these pack limits. */
static bool tristar_pack_allows_grant(const struct tristar_pack *pack)
{
	return pack->temp >= 50 && pack->temp <= 400 &&
	       pack->voltage_uv >= 3200000 && pack->voltage_uv <= 4350000 &&
	       pack->current_ua >= -3000000 && pack->current_ua <= 3000000;
}

/* ---- Tristar ---- */

static int cbtl_read(struct tristar_charger *chg, u8 reg, u8 *buf, u16 len)
{
	struct i2c_client *client = chg->tristar;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = 1, .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);

	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static int cbtl_write(struct tristar_charger *chg, u8 reg, u8 value)
{
	struct i2c_client *client = chg->tristar;
	u8 bytes[2] = { reg, value };
	struct i2c_msg msg = { .addr = client->addr, .len = 2, .buf = bytes };
	u8 readback;
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;
	ret = cbtl_read(chg, reg, &readback, 1);
	return ret ?: readback == value ? 0 : -EIO;
}

static int cbtl_route(struct tristar_charger *chg, u8 value)
{
	return cbtl_write(chg, CBTL_MUX_ROUTE, value);
}

/* Forward CRC-8: seed 0xff, reflected polynomial 0x8c. */
static u8 cbtl_crc(const u8 *data, unsigned int len)
{
	unsigned int crc = 0xff, i, bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}
	return crc;
}

/*
 * Decode the routes of the accessory's capability response, as the Tristar
 * firmware's legacy branch and its helper do. Extended layouts and
 * unsuccessful multi-function routes are rejected. Returns false when the
 * layout is not understood.
 */
static bool cbtl_decode(u8 h0, u8 h2, u8 h3, unsigned int orientation,
			enum cbtl_route *first, enum cbtl_route *second)
{
	bool usb = h0 & 0x18, uart = h0 & 0x03, mbus = h0 & 0x04;
	bool usbc = h2 & 0x08, uart2 = false;
	enum cbtl_route tags[2] = { CBTL_ROUTE_NONE, CBTL_ROUTE_NONE };
	unsigned int n = 0;

	if ((h0 & 0x20) || (orientation != 1 && orientation != 2))
		return false;
	/* The helper can replace the legacy flags; byte 2 bit 3 alone is not enough. */
	if (mbus) {
		if (usb && (h0 & 0x03) == 0x01 && (h3 & 0x70) == 0x20) {
			uart = mbus = usbc = false;
		} else if (usbc) {
			if (usb && (h0 & 0x03) == 0x02) {
				uart = mbus = usbc = false;
				uart2 = true;
			} else if ((h0 & 0x1b) == 0x1b) {
				usb = uart = usbc = false;
				uart2 = true;
			} else if ((h0 & 0x1b) == 0x0b) {
				usb = mbus = usbc = false;
				uart2 = true;
			}
		}
	}
	if (usb + uart + mbus + usbc + uart2 >= 3)
		return false;
	if (usb)
		tags[n++] = CBTL_ROUTE_USB0;
	if (uart)
		tags[n++] = CBTL_ROUTE_UART0;
	if (mbus)
		tags[n++] = CBTL_ROUTE_MBUS;
	if (usbc)
		tags[n++] = CBTL_ROUTE_USBC;
	if (uart2)
		tags[1] = CBTL_ROUTE_UART2;
	*first = tags[orientation == 2 ? 1 : 0];
	*second = tags[orientation == 2 ? 0 : 1];
	return true;
}

/*
 * Read the capability frame once. The status byte must be stable around the
 * response, and the response must carry the expected opcode and CRC.
 */
static int cbtl_capability(struct tristar_charger *chg, bool *usbc,
			   unsigned int *orientation)
{
	enum cbtl_route first, second;
	u8 packet[CBTL_CAP_LEN], before, after;
	int ret;

	ret = cbtl_read(chg, CBTL_CAP_STATUS, &before, 1);
	if (ret)
		return ret;
	if ((before & 0x84) != 0x80)
		return -EAGAIN;
	ret = cbtl_read(chg, CBTL_CAP_RESPONSE, packet, sizeof(packet));
	if (!ret)
		ret = cbtl_read(chg, CBTL_CAP_STATUS, &after, 1);
	if (ret)
		goto out;
	if (before != after || packet[0] != 0x75) {
		ret = -EAGAIN;
		goto out;
	}
	if (cbtl_crc(packet, 7) != packet[7]) {
		ret = -EBADMSG;
		goto out;
	}
	*orientation = 1 + !!(after & 0x40);
	if (!cbtl_decode(packet[1] & 0x3f, packet[3] & 0x08, packet[4] & 0x70,
			 *orientation, &first, &second)) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	*usbc = first == CBTL_ROUTE_USBC || second == CBTL_ROUTE_USBC;
out:
	/* The response can carry accessory identity; do not keep it. */
	memzero_explicit(packet, sizeof(packet));
	return ret;
}

/* ---- ADC ---- */

/* One conversion; completion arrives as a PMU event. Caller holds lock. */
static int tristar_adc_mv(struct tristar_charger *chg, unsigned int epoch,
			  unsigned int *mv)
{
	u8 value[2];
	u32 raw;
	int ret;

	reinit_completion(&chg->adc_done);
	WRITE_ONCE(chg->adc_waiting, true);
	ret = regmap_write(chg->regmap, chg->adc_base + TRISTAR_CHG_ADC_SELECT, 1);
	if (ret) {
		WRITE_ONCE(chg->adc_waiting, false);
		return ret;
	}
	if (!wait_for_completion_timeout(&chg->adc_done, msecs_to_jiffies(500))) {
		WRITE_ONCE(chg->adc_waiting, false);
		return -ETIMEDOUT;
	}
	if (READ_ONCE(chg->epoch) != epoch)
		return -ESTALE;
	ret = regmap_bulk_read(chg->regmap, chg->adc_base + TRISTAR_CHG_ADC_RESULT,
			       value, sizeof(value));
	if (ret)
		return ret;
	if (value[0] & 0x80)
		return -EIO;
	raw = (value[0] & 0x0f) | ((u32)value[1] << 4);
	*mv = (((u64)raw * 5000 * 16 + 40 * 65536) / 1000) * 1000 / 65536;
	return 0;
}

/*
 * Route the Tristar mux, optionally excite the other line of the pair for
 * BC1.2 detection, convert, and always restore the mux and the excitation.
 */
static int tristar_mux_sample(struct tristar_charger *chg, const u8 *routes,
			      unsigned int count, u8 excite, unsigned int epoch,
			      unsigned int *mv)
{
	unsigned int i;
	int ret = 0, restore = 0;
	u8 state;

	for (i = 0; i < count && !ret; i++)
		ret = cbtl_route(chg, routes[i]);
	if (!ret) {
		usleep_range(5000, 6000);
		if (excite) {
			ret = cbtl_write(chg, CBTL_MUX_STATE, excite);
			if (!ret)
				msleep(40);
		}
		if (!ret)
			ret = tristar_adc_mv(chg, epoch, mv);
	}
	if (excite)
		restore = cbtl_write(chg, CBTL_MUX_STATE, 0);
	if (!restore)
		restore = cbtl_route(chg, 0);
	if (!restore)
		restore = cbtl_read(chg, CBTL_MUX_STATE, &state, 1);
	if (!restore && state)
		restore = -EIO;
	if (restore) {
		dev_crit(chg->dev, "Tristar mux restore unverified (%d)\n", restore);
		return -EUCLEAN;
	}
	return ret;
}

/* Apple USB-A chargers: D+ and D- divider bands select the BrickID. */
static unsigned int tristar_brick_ma(unsigned int dp, unsigned int dm,
				     unsigned int *id)
{
	static const unsigned int brick_ma[9] = {
		500, 1000, 1000, 2100, 2400, 3000, 1000, 1000, 1000,
	};
	unsigned int p, m;

	if (dp < 1500 || dm < 1500)
		return 0;
	p = dp < 2184 ? 0 : dp < 2880 ? 1 : 2;
	m = dm < 2184 ? 0 : dm < 2880 ? 1 : 2;
	*id = 1 + 3 * p + m;
	return brick_ma[*id - 1];
}

/* ---- Identification ---- */

struct tristar_ident {
	unsigned int ma;		/* the source's advertisement, 0: none */
	enum power_supply_usb_type type;
	bool sampled;			/* the mux was used */
	char what[96];
};

/* Mux routes per bank: D- and D+ through the USB-selected bank. */
static const u8 tristar_dminus[2][2] = { { 0x11, 0x1d }, { 0x10, 0xd0 } };
static const u8 tristar_dplus[2][2] = { { 0x11, 0x1c }, { 0x10, 0xc0 } };

/*
 * BC1.2 primary and secondary detection on the oriented USB pair, as iOS
 * does for an ordinary USB source (5de33e4, 5de35dc): with the measured line
 * routed to the ADC, the Tristar excites the pair for 40 ms (register 0x03:
 * 9 while D- is measured, 10 while D+ is, plus 2 on the high bank; 5ff5a08).
 * A primary D- of 326-899 mV means a charging port; the secondary D+ then
 * tells a CDP (below 326 mV) from a DCP. Both may draw 1.5 A.
 */
static int tristar_identify_bc12(struct tristar_charger *chg, unsigned int bank,
				 unsigned int epoch, struct tristar_ident *id)
{
	unsigned int dm, dp;
	int ret;

	ret = tristar_mux_sample(chg, tristar_dminus[bank], 2, 9 + 2 * bank,
				 epoch, &dm);
	if (ret)
		return ret;
	if (dm < 326 || dm > 899)
		return 0;
	ret = tristar_mux_sample(chg, tristar_dplus[bank], 2, 10 + 2 * bank,
				 epoch, &dp);
	if (ret)
		return ret;
	id->ma = 1500;
	id->type = dp < 326 ? POWER_SUPPLY_USB_TYPE_CDP : POWER_SUPPLY_USB_TYPE_DCP;
	snprintf(id->what, sizeof(id->what),
		 "USB charging port (%s, BC1.2 D- %u mV, D+ %u mV)",
		 id->type == POWER_SUPPLY_USB_TYPE_CDP ? "CDP" : "DCP", dm, dp);
	return 0;
}

static int tristar_identify_usbc(struct tristar_charger *chg, unsigned int orientation,
				 unsigned int epoch, struct tristar_ident *id)
{
	/* Lane 0 is sampled through route 0xc0, lane 1 through 0x0c. */
	static const u8 lanes[2] = { 0xc0, 0x0c };
	unsigned int mv[2], sel = orientation == 2 ? 1 : 0, cc_ma;
	int ret, lane;

	/*
	 * The firmware reads only the orientation-selected lane. The other is
	 * logged as evidence; it is not a CC advertisement.
	 */
	for (lane = 0; lane < 2; lane++) {
		ret = tristar_mux_sample(chg, &lanes[lane], 1, 0, epoch, &mv[lane]);
		if (ret)
			return ret;
	}
	cc_ma = mv[sel] >= 1230 ? 3000 : mv[sel] >= 660 ? 1500 : 0;
	if (cc_ma < 1500) {
		/* Default USB power: a charging port may still allow 1.5 A. */
		ret = tristar_identify_bc12(chg, orientation - 1, epoch, id);
		if (ret)
			return ret;
		if (id->ma > cc_ma)
			return 0;
	}
	id->ma = cc_ma;
	id->type = POWER_SUPPLY_USB_TYPE_C;
	if (cc_ma)
		snprintf(id->what, sizeof(id->what),
			 "USB-C power source advertising %u.%u A (CC %u mV, other lane %u mV)",
			 cc_ma / 1000, cc_ma % 1000 / 100, mv[sel], mv[!sel]);
	else
		snprintf(id->what, sizeof(id->what),
			 "USB-C source with default USB power (CC %u mV, other lane %u mV)",
			 mv[sel], mv[!sel]);
	return 0;
}

static int tristar_identify_usba(struct tristar_charger *chg, unsigned int orientation,
				 unsigned int epoch, struct tristar_ident *id)
{
	unsigned int bank_ma[2], bank_id[2] = { 0, 0 }, dp, dm, brick;
	int ret, bank;

	for (bank = 0; bank < 2; bank++) {
		ret = tristar_mux_sample(chg, tristar_dminus[bank], 2, 0, epoch, &dm);
		if (!ret)
			ret = tristar_mux_sample(chg, tristar_dplus[bank], 2, 0, epoch, &dp);
		if (ret)
			return ret;
		bank_ma[bank] = tristar_brick_ma(dp, dm, &bank_id[bank]);
	}
	if (bank_ma[0] && bank_ma[1] && bank_id[0] != bank_id[1]) {
		/* Disagreeing banks: trust the lower advertisement. */
		id->ma = min(bank_ma[0], bank_ma[1]);
		brick = bank_ma[0] < bank_ma[1] ? bank_id[0] : bank_id[1];
	} else {
		bank = bank_ma[0] ? 0 : 1;
		id->ma = bank_ma[bank];
		brick = bank_id[bank];
	}
	if (id->ma) {
		id->type = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
		snprintf(id->what, sizeof(id->what), "Apple USB-A charger, BrickID %u (%u.%u A)",
			 brick, id->ma / 1000, id->ma % 1000 / 100);
		return 0;
	}
	ret = tristar_identify_bc12(chg, orientation - 1, epoch, id);
	if (!ret && !id->ma)
		snprintf(id->what, sizeof(id->what),
			 "USB-A source without charger signature (computer, hub or generic charger)");
	return ret;
}

/* Caller holds lock. */
static int tristar_identify(struct tristar_charger *chg, struct tristar_ident *id)
{
	unsigned int epoch = READ_ONCE(chg->epoch), orientation;
	bool usbc;
	u8 route, state;
	int ret;

	id->ma = 0;
	id->type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	id->sampled = false;
	ret = cbtl_read(chg, CBTL_MUX_ROUTE, &route, 1);
	if (!ret)
		ret = cbtl_read(chg, CBTL_MUX_STATE, &state, 1);
	if (ret)
		return ret;
	if (route || state) {
		snprintf(id->what, sizeof(id->what), "unknown source (mux busy %02x/%02x)",
			 route, state);
		return 0;
	}
	ret = cbtl_capability(chg, &usbc, &orientation);
	if (ret) {
		snprintf(id->what, sizeof(id->what), "unidentified cable (capability frame %d)",
			 ret);
		return 0;
	}
	id->sampled = true;
	if (usbc)
		return tristar_identify_usbc(chg, orientation, epoch, id);
	return tristar_identify_usba(chg, orientation, epoch, id);
}

/* ---- Policy ---- */

/*
 * A USB session reported in this epoch: a PMU source event may mean another
 * host, so it ends what the previous one allowed. Caller holds lock.
 */
static unsigned int tristar_usb_ma(struct tristar_charger *chg)
{
	return chg->usb_epoch == READ_ONCE(chg->epoch) ? chg->usb_ma : 0;
}

/*
 * The device controller reported a host and no disconnect since, in any
 * epoch: a PMU event ends a host's allowance, not its data session, so the
 * mux stays untouched until the session ends. Caller holds lock.
 */
static bool tristar_usb_session(struct tristar_charger *chg)
{
	return chg->usb_ma;
}

/* The highest grant the source allows, up to the board maximum. */
static unsigned int tristar_target_code(struct tristar_charger *chg)
{
	unsigned int ma = max(chg->source_ma, tristar_usb_ma(chg));

	if (ma <= tristar_code_ma(TRISTAR_CHG_FALLBACK_CODE))
		return TRISTAR_CHG_FALLBACK_CODE;
	return tristar_ma_code(min(ma, chg->max_ma));
}

/* Follow a changed allowance once the source is identified. Caller holds lock. */
static void tristar_update_target(struct tristar_charger *chg)
{
	if (chg->phase != TRISTAR_RAMPING && chg->phase != TRISTAR_CHARGING)
		return;
	chg->target_code = tristar_target_code(chg);
	if (chg->code > chg->target_code) {
		tristar_withdraw(chg, chg->target_code);
		chg->phase = TRISTAR_CHARGING;
	} else if (chg->code < chg->target_code && chg->phase == TRISTAR_CHARGING) {
		chg->phase = TRISTAR_RAMPING;
		chg->step_ns = 0;
	}
}

static void tristar_new_attachment(struct tristar_charger *chg, u64 now)
{
	chg->present = true;
	chg->phase = TRISTAR_SETTLING;
	chg->attach_ns = now;
	chg->source[0] = '\0';
	chg->source_ma = 0;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	chg->early_done = false;
	chg->redetects = 0;
	chg->retries = 0;
	chg->epoch_seen = READ_ONCE(chg->epoch);
	if (tristar_set_code(chg, TRISTAR_CHG_FALLBACK_CODE))
		dev_crit(chg->dev, "Fallback write failed; input limit unverified\n");
	dev_dbg(chg->dev, "power connected, identifying source\n");
}

/* The early look happened in this epoch. Caller holds lock. */
static bool tristar_early_done(struct tristar_charger *chg)
{
	return chg->early_done && chg->early_epoch == READ_ONCE(chg->epoch);
}

static void tristar_mux_fault(struct tristar_charger *chg)
{
	chg->phase = TRISTAR_FAULT;
	strscpy(chg->source, "Tristar mux restore failed", sizeof(chg->source));
	tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
	dev_err(chg->dev, "Tristar mux restore unverified; holding the fallback until reboot\n");
}

/*
 * Look at the source once while settling, before a host starts its USB
 * session. The identification after the settle time must not sample during
 * that session, so a host's own advertisement (CDP, USB-C current) is kept
 * from here for it; everything else is measured again after settling.
 */
static void tristar_early_identification(struct tristar_charger *chg)
{
	unsigned int epoch = READ_ONCE(chg->epoch);
	struct tristar_ident id;
	int ret;

	ret = tristar_identify(chg, &id);
	if (ret == -EUCLEAN) {
		tristar_mux_fault(chg);
		return;
	}
	if (!id.sampled)
		return;		/* cable not ready yet: try again next poll */
	chg->early_done = true;
	chg->early_epoch = epoch;
	chg->early_ma = 0;
	chg->early_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	if (ret || READ_ONCE(chg->epoch) != epoch || tristar_usb_session(chg))
		return;
	if (id.type == POWER_SUPPLY_USB_TYPE_CDP ||
	    (id.type == POWER_SUPPLY_USB_TYPE_C && id.ma)) {
		chg->early_ma = id.ma;
		chg->early_type = id.type;
		dev_dbg(chg->dev, "before USB: %s\n", id.what);
	}
}

static void tristar_run_identification(struct tristar_charger *chg, u64 now)
{
	unsigned int fallback_ma = tristar_code_ma(TRISTAR_CHG_FALLBACK_CODE);
	struct tristar_ident id;
	bool usb = tristar_usb_session(chg);
	int ret = 0;

	if (usb) {
		/* A USB session is active: do not disturb its data lines. */
		id.ma = tristar_early_done(chg) ? chg->early_ma : 0;
		id.type = id.ma ? chg->early_type : POWER_SUPPLY_USB_TYPE_UNKNOWN;
		snprintf(id.what, sizeof(id.what), "USB host%s",
			 id.type == POWER_SUPPLY_USB_TYPE_CDP ? ", charging downstream port" :
			 id.type == POWER_SUPPLY_USB_TYPE_C ? ", USB-C current advertised" : "");
	} else {
		ret = tristar_identify(chg, &id);
	}
	if (ret == -EUCLEAN) {
		tristar_mux_fault(chg);
		return;
	}
	if (ret || READ_ONCE(chg->epoch) != chg->epoch_seen) {
		/* The source changed while identifying: retry on a stable epoch. */
		chg->epoch_seen = READ_ONCE(chg->epoch);
		if (++chg->redetects > TRISTAR_MAX_REDETECT) {
			chg->phase = TRISTAR_CHARGING;
			strscpy(chg->source, "unstable source", sizeof(chg->source));
			dev_dbg(chg->dev, "source unstable during identification (%d)\n", ret);
		} else {
			chg->attach_ns = now;
		}
		return;
	}
	if (!id.ma && !usb && chg->retries < TRISTAR_UNIDENTIFIED_RETRIES) {
		chg->retries++;
		/* The settle check fires SETTLE_MS after attach_ns. */
		chg->attach_ns = now + (TRISTAR_RETRY_MS - TRISTAR_SETTLE_MS) * NSEC_PER_MSEC;
		dev_dbg(chg->dev, "no charger signature yet (%s), retry %u/%u\n",
			id.what, chg->retries, TRISTAR_UNIDENTIFIED_RETRIES);
		return;
	}
	strscpy(chg->source, id.what, sizeof(chg->source));
	chg->source_ma = id.ma;
	chg->usb_type = id.type;
	chg->target_code = tristar_target_code(chg);
	if (chg->target_code <= TRISTAR_CHG_FALLBACK_CODE) {
		chg->phase = TRISTAR_CHARGING;
		dev_dbg(chg->dev, "%s: %u mA\n", id.what, fallback_ma);
	} else {
		chg->phase = TRISTAR_RAMPING;
		chg->step_ns = 0;
		dev_dbg(chg->dev, "%s: up to %u mA\n", id.what,
			tristar_code_ma(chg->target_code));
	}
}

static void tristar_ramp_step(struct tristar_charger *chg, u64 now)
{
	struct tristar_pack pack;
	unsigned int next;

	if (chg->step_ns && now - chg->step_ns < TRISTAR_RAMP_STEP_MS * NSEC_PER_MSEC)
		return;
	if (tristar_read_pack(&pack) || !tristar_pack_allows_grant(&pack)) {
		chg->phase = TRISTAR_CHARGING;
		tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
		dev_warn(chg->dev, "battery outside the charging window; holding the fallback\n");
		return;
	}
	next = min(chg->target_code,
		   tristar_ma_code(tristar_code_ma(chg->code) + TRISTAR_RAMP_STEP_MA));
	if (tristar_set_code(chg, next)) {
		chg->phase = TRISTAR_FAULT;
		tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
		dev_err(chg->dev, "input-limit write failed; holding the fallback\n");
		return;
	}
	chg->step_ns = now;
	if (next == chg->target_code)
		chg->phase = TRISTAR_CHARGING;
}

static void tristar_guard(struct tristar_charger *chg, u64 now)
{
	struct tristar_pack pack;

	if (now - chg->guard_ns < NSEC_PER_SEC)
		return;
	chg->guard_ns = now;
	if (tristar_read_pack(&pack))
		return;
	if (chg->code > TRISTAR_CHG_FALLBACK_CODE && !tristar_pack_allows_grant(&pack)) {
		tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
		dev_warn(chg->dev, "battery outside the charging window; holding the fallback\n");
	}
}

static void tristar_poll(struct work_struct *work)
{
	struct tristar_charger *chg = container_of(to_delayed_work(work),
						   struct tristar_charger, poll_work);
	unsigned int status, old_code;
	bool old_online;
	u64 now = ktime_get_ns(), line_ns;

	mutex_lock(&chg->lock);
	if (chg->stopping)
		goto out;
	old_code = chg->code;
	old_online = chg->online;
	if (regmap_read(chg->regmap, chg->status_base + TRISTAR_CHG_STATUS, &status)) {
		tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
		dev_err_ratelimited(chg->dev, "PMIC status unreadable; holding the fallback\n");
		goto again;
	}
	chg->online = status != 0;
	if (!status) {
		if (chg->present) {
			chg->present = false;
			chg->phase = TRISTAR_EMPTY;
			chg->source[0] = '\0';
			chg->source_ma = 0;
			chg->usb_ma = 0;
			chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			if (tristar_set_code(chg, TRISTAR_CHG_HANDOFF_CODE))
				dev_crit(chg->dev, "Handoff write failed; input limit unverified\n");
			dev_dbg(chg->dev, "disconnected\n");
		}
		goto again;
	}
	if (status != TRISTAR_CHG_STATUS_ONLINE) {
		/* Transitional input state (e.g. 0x40 while attaching): no grant. */
		if (chg->code > TRISTAR_CHG_FALLBACK_CODE)
			tristar_withdraw(chg, TRISTAR_CHG_FALLBACK_CODE);
		goto again;
	}
	if (!chg->present) {
		tristar_new_attachment(chg, now);
		goto again;
	}
	if (chg->phase == TRISTAR_FAULT)
		goto again;
	if (READ_ONCE(chg->epoch) != chg->epoch_seen && chg->phase != TRISTAR_SETTLING) {
		/* The event handler already withdrew any grant. */
		chg->epoch_seen = READ_ONCE(chg->epoch);
		chg->line_seen = READ_ONCE(chg->line_ns);
		chg->phase = TRISTAR_SETTLING;
		chg->attach_ns = now;
		dev_dbg(chg->dev, "source event, re-identifying\n");
		goto again;
	}
	line_ns = READ_ONCE(chg->line_ns);
	if (line_ns != chg->line_seen) {
		/*
		 * The line handler withdrew the grant. Wait until the events of
		 * that assertion have been dispatched; if none of them was a
		 * source event, the source is unchanged: ramp back up.
		 */
		if ((s64)(now - line_ns) < (s64)TRISTAR_LINE_SETTLE_MS * NSEC_PER_MSEC)
			goto again;
		chg->line_seen = line_ns;
		if (chg->phase == TRISTAR_CHARGING && chg->code < chg->target_code) {
			chg->phase = TRISTAR_RAMPING;
			chg->step_ns = 0;
		}
	}
	switch (chg->phase) {
	case TRISTAR_SETTLING:
		if ((s64)(now - chg->attach_ns) >= (s64)TRISTAR_SETTLE_MS * NSEC_PER_MSEC)
			tristar_run_identification(chg, now);
		else if (!tristar_early_done(chg) && !tristar_usb_session(chg) &&
			 (s64)(now - chg->attach_ns) >= (s64)TRISTAR_EARLY_MS * NSEC_PER_MSEC)
			tristar_early_identification(chg);
		break;
	case TRISTAR_RAMPING:
		tristar_ramp_step(chg, now);
		break;
	case TRISTAR_CHARGING:
		tristar_guard(chg, now);
		break;
	default:
		break;
	}
again:
	mod_delayed_work(system_highpri_wq, &chg->poll_work,
			 msecs_to_jiffies(TRISTAR_POLL_MS));
	if (chg->code != old_code || chg->online != old_online)
		power_supply_changed(chg->psy);
out:
	mutex_unlock(&chg->lock);
}

/* ---- PMU events ---- */

/*
 * Drop a grant to the fallback, from an event handler. No policy lock:
 * regmap serializes the write against any sequence in flight, and a grant
 * must never wait for the policy to finish. Returns true if it lowered one.
 */
static bool tristar_event_withdraw(struct tristar_charger *chg)
{
	u32 reg = chg->control_base + TRISTAR_CHG_INPUT_LIMIT;
	unsigned int readback;

	if (READ_ONCE(chg->code) <= TRISTAR_CHG_FALLBACK_CODE)
		return false;
	if (!regmap_write(chg->regmap, reg, TRISTAR_CHG_FALLBACK_CODE) &&
	    !regmap_read(chg->regmap, reg, &readback) &&
	    readback == TRISTAR_CHG_FALLBACK_CODE)
		WRITE_ONCE(chg->code, TRISTAR_CHG_FALLBACK_CODE);
	else
		dev_crit(chg->dev, "PMU event: withdrawal write failed\n");
	return true;
}

/*
 * The PMU line asserted and the event controller has not read the events
 * yet. Withdraw first, attribute later: the grant is gone before the I2C
 * read of the event block. The poll ramps it back if no source event
 * follows (a GPIO edge or another PMU client's event).
 */
static irqreturn_t tristar_line_irq(int irq, void *data)
{
	struct tristar_charger *chg = data;

	if (tristar_event_withdraw(chg))
		WRITE_ONCE(chg->line_ns, ktime_get_ns() ?: 1);
	return IRQ_HANDLED;
}

/*
 * A PMU event that can mean a source change. The line handler has already
 * withdrawn any grant; withdraw again in case one was raised since. Then a
 * new epoch invalidates any measurement or grant in progress.
 */
static void tristar_source_event(struct tristar_charger *chg)
{
	u32 reg = chg->control_base + TRISTAR_CHG_INPUT_LIMIT;
	unsigned int i;
	u64 now;

	tristar_event_withdraw(chg);
	WRITE_ONCE(chg->epoch, chg->epoch + 1);
	mod_delayed_work(system_highpri_wq, &chg->poll_work, 0);

	now = ktime_get_ns();
	if (now - chg->storm_start_ns > NSEC_PER_SEC) {
		chg->storm_start_ns = now;
		chg->storm_count = 0;
	}
	if (++chg->storm_count > TRISTAR_IRQ_STORM) {
		dev_crit(chg->dev, "PMU source events storming; holding the fallback\n");
		regmap_write(chg->regmap, reg, TRISTAR_CHG_FALLBACK_CODE);
		WRITE_ONCE(chg->code, TRISTAR_CHG_FALLBACK_CODE);
		WRITE_ONCE(chg->phase, TRISTAR_FAULT);
		for (i = TRISTAR_IRQ_SOURCE; i < chg->nirqs; i++)
			disable_irq_nosync(chg->irqs[i]);
	}
}

static irqreturn_t tristar_source_irq(int irq, void *data)
{
	tristar_source_event(data);
	return IRQ_HANDLED;
}

/*
 * The event controller dispatches in ascending event order and ADC
 * completion has the highest number, so a source event latched together
 * with it has already started a new epoch: the waiter then discards its
 * result instead of accepting a conversion that straddled a swap.
 */
static irqreturn_t tristar_adc_irq(int irq, void *data)
{
	struct tristar_charger *chg = data;

	if (READ_ONCE(chg->adc_waiting)) {
		WRITE_ONCE(chg->adc_waiting, false);
		complete(&chg->adc_done);
	} else {
		/* Nobody started that conversion. */
		tristar_source_event(chg);
	}
	return IRQ_HANDLED;
}

/* ---- power_supply ---- */

static int tristar_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct tristar_charger *chg = power_supply_get_drvdata(psy);

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg->usb_type;
		if (val->intval == POWER_SUPPLY_USB_TYPE_UNKNOWN && tristar_usb_session(chg))
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = tristar_code_ma(chg->code) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* What the identified source or USB host allows; 0 when unknown. */
		val->intval = max(chg->source_ma, tristar_usb_ma(chg)) * 1000;
		break;
	default:
		mutex_unlock(&chg->lock);
		return -EINVAL;
	}
	mutex_unlock(&chg->lock);
	return 0;
}

/*
 * The USB device controller reports what the host allows the device to draw
 * (vbus_draw) as the input current limit: 0 without a session, 100 mA
 * unconfigured, the configuration's bMaxPower, 2 mA suspended, or more after
 * an Apple host's current grant. Not writable from user space.
 */
static int tristar_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct tristar_charger *chg = power_supply_get_drvdata(psy);
	unsigned int old_code;

	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;

	mutex_lock(&chg->lock);
	old_code = chg->code;
	/*
	 * With nothing attached there is no session: a late report (the
	 * suspend draw after a disconnect) must not make the next source look
	 * like a USB host.
	 */
	chg->usb_ma = chg->present ? max(val->intval, 0) / 1000 : 0;
	chg->usb_epoch = READ_ONCE(chg->epoch);
	dev_dbg(chg->dev, "USB host allows %u mA\n", chg->usb_ma);
	if (!chg->stopping) {
		tristar_update_target(chg);
		mod_delayed_work(system_highpri_wq, &chg->poll_work, 0);
	}
	mutex_unlock(&chg->lock);

	if (READ_ONCE(chg->code) != old_code)
		power_supply_changed(psy);
	return 0;
}

static const enum power_supply_property tristar_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc tristar_desc = {
	.name = "lightning",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID) |
		     BIT(POWER_SUPPLY_USB_TYPE_C),
	.properties = tristar_properties,
	.num_properties = ARRAY_SIZE(tristar_properties),
	.get_property = tristar_get_property,
	.set_property = tristar_set_property,
};

/* ---- Probe ---- */

static int tristar_get_bank(struct device *dev, const char *name, u32 size,
			    u32 *base)
{
	int index = of_property_match_string(dev->of_node, "reg-names", name);
	u64 address, length;
	int ret;

	if (index < 0)
		return dev_err_probe(dev, index, "missing %s register range\n", name);
	ret = of_property_read_reg(dev->of_node, index, &address, &length);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read %s register range\n", name);
	if (length < size || address + size - 1 > U32_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid %s register range\n", name);
	*base = address;
	return 0;
}

static void tristar_put_client(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

/* Leave the limit as iBoot did, so a reboot or a kexec'd kernel starts safe. */
static void tristar_stop(struct tristar_charger *chg, const char *reason)
{
	unsigned int i;

	mutex_lock(&chg->lock);
	chg->stopping = true;
	mutex_unlock(&chg->lock);
	for (i = 0; i < chg->nirqs; i++)
		disable_irq(chg->irqs[i]);
	cancel_delayed_work_sync(&chg->poll_work);
	mutex_lock(&chg->lock);
	if (tristar_set_code(chg, TRISTAR_CHG_HANDOFF_CODE))
		dev_crit(chg->dev, "%s: handoff restore failed\n", reason);
	mutex_unlock(&chg->lock);
}

static void tristar_stop_action(void *data)
{
	tristar_stop(data, "remove");
}

static int tristar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config config = {};
	struct tristar_charger *chg;
	struct device_node *np;
	unsigned int code, i;
	u32 max_ua;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;
	chg->dev = dev;
	chg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	ret = tristar_get_bank(dev, "status", TRISTAR_CHG_STATUS + 1, &chg->status_base);
	if (!ret)
		ret = tristar_get_bank(dev, "control", TRISTAR_CHG_INPUT_LIMIT + 1,
				       &chg->control_base);
	if (!ret)
		ret = tristar_get_bank(dev, "adc", TRISTAR_CHG_ADC_RESULT + 2, &chg->adc_base);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "input-current-limit-microamp", &max_ua);
	if (ret)
		return dev_err_probe(dev, ret, "missing board input-current maximum\n");
	chg->max_ma = max_ua / 1000;

	np = of_parse_phandle(dev->of_node, "apple,tristar", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "missing apple,tristar\n");
	chg->tristar = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!chg->tristar)
		return dev_err_probe(dev, -EPROBE_DEFER, "Tristar not available yet\n");
	ret = devm_add_action_or_reset(dev, tristar_put_client, chg->tristar);
	if (ret)
		return ret;
	if (!i2c_check_functionality(chg->tristar->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP, "Tristar bus lacks plain I2C\n");

	ret = platform_irq_count(pdev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to count the PMU event interrupts\n");
	if (ret <= TRISTAR_IRQ_SOURCE || ret > TRISTAR_IRQS_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "expected PMU line, ADC and source event interrupts\n");
	chg->nirqs = ret;
	for (i = 0; i < chg->nirqs; i++) {
		chg->irqs[i] = platform_get_irq(pdev, i);
		if (chg->irqs[i] < 0)
			return chg->irqs[i];
	}

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;
	init_completion(&chg->adc_done);
	INIT_DELAYED_WORK(&chg->poll_work, tristar_poll);

	/* Never adopt a limit we cannot explain; anything higher drops to the fallback. */
	ret = regmap_read(chg->regmap, chg->control_base + TRISTAR_CHG_INPUT_LIMIT, &code);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the input-current limit\n");
	chg->code = code;
	if (code > TRISTAR_CHG_FALLBACK_CODE) {
		dev_warn(dev, "Input limit %u mA at probe; dropping to %u mA\n",
			 tristar_code_ma(code), tristar_code_ma(TRISTAR_CHG_FALLBACK_CODE));
		ret = tristar_set_code(chg, TRISTAR_CHG_FALLBACK_CODE);
		if (ret)
			return dev_err_probe(dev, ret, "failed to lower the input-current limit\n");
	}

	config.drv_data = chg;
	config.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &tristar_desc, &config);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "failed to register power supply\n");

	for (i = 0; i < chg->nirqs; i++) {
		ret = devm_request_threaded_irq(dev, chg->irqs[i], NULL,
						i == TRISTAR_IRQ_LINE ? tristar_line_irq :
						i == TRISTAR_IRQ_ADC ? tristar_adc_irq :
						tristar_source_irq,
						IRQF_ONESHOT, dev_name(dev), chg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request PMU event %u\n", i);
	}

	platform_set_drvdata(pdev, chg);
	ret = devm_add_action_or_reset(dev, tristar_stop_action, chg);
	if (ret)
		return ret;

	mod_delayed_work(system_highpri_wq, &chg->poll_work, 0);
	return 0;
}

static void tristar_shutdown(struct platform_device *pdev)
{
	struct tristar_charger *chg = platform_get_drvdata(pdev);

	if (chg)
		tristar_stop(chg, "shutdown");
}

/*
 * Sleep can hide a complete source swap: park at the handoff limit and
 * identify again on resume.
 */
static int tristar_suspend(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chg->lock);
	chg->stopping = true;
	chg->present = false;
	chg->phase = TRISTAR_EMPTY;
	ret = tristar_set_code(chg, TRISTAR_CHG_HANDOFF_CODE);
	if (ret)
		chg->stopping = false;
	mutex_unlock(&chg->lock);
	if (ret)
		return ret;
	cancel_delayed_work_sync(&chg->poll_work);
	return 0;
}

static int tristar_resume(struct device *dev)
{
	struct tristar_charger *chg = dev_get_drvdata(dev);

	mutex_lock(&chg->lock);
	chg->stopping = false;
	mutex_unlock(&chg->lock);
	mod_delayed_work(system_highpri_wq, &chg->poll_work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(tristar_pm_ops, tristar_suspend, tristar_resume);

static const struct of_device_id tristar_of_match[] = {
	{ .compatible = "apple,tristar-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, tristar_of_match);

static struct platform_driver tristar_driver = {
	.driver = {
		.name = "apple-tristar-charger",
		.of_match_table = tristar_of_match,
		.pm = pm_sleep_ptr(&tristar_pm_ops),
	},
	.probe = tristar_probe,
	.shutdown = tristar_shutdown,
};
module_platform_driver(tristar_driver);

MODULE_DESCRIPTION("Apple Tristar Lightning charging");
MODULE_LICENSE("GPL");
