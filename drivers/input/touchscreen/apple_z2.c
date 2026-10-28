// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Z2 touchscreen driver
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bitmap.h>
#include <linux/bitrev.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "apple_z2_n61.h"

#define APPLE_Z2_NUM_FINGERS_OFFSET      16
#define APPLE_Z2_FINGERS_OFFSET          24
#define APPLE_Z2_TOUCH_STARTED           3
#define APPLE_Z2_TOUCH_MOVED             4
#define APPLE_Z2_CMD_READ_INTERRUPT_DATA 0xEB
#define APPLE_Z2_REPLY_INTERRUPT_DATA    0xE1
#define APPLE_Z2_FRAME_INPUT_REPORT      0x90
#define APPLE_Z2_INTERFACE_GRAPE         BIT(1)
#define APPLE_Z2_HID_REPORT_ID           0x76
#define APPLE_Z2_D111_TOUCH_REPORT       0x44
#define APPLE_Z2_D11_STATUS_REPORT       0x50
#define APPLE_Z2_REPORT_ENABLE           0xaf
#define APPLE_Z2_D11_REPORT_ENABLE       0xe2
#define APPLE_Z2_J172_HEADER_SIZE         32
#define APPLE_Z2_J172_CONTACT_SIZE        48
#define APPLE_Z2_J172_MAX_CONTACTS        32
#define APPLE_Z2_J172_FLOAT_FORMAT        BIT(1)
#define APPLE_Z2_REPORT_SURFACE           0xd9
#define APPLE_Z2_SURFACE_DESCRIPTOR_SIZE  16
#define APPLE_Z2_CMD_DEVICE_INFO         0xE2
#define APPLE_Z2_CMD_REPORT_INFO         0xE3
#define APPLE_Z2_CMD_CTRL_WRITE_SHORT    0xE4
#define APPLE_Z2_CMD_CTRL_READ_SHORT     0xE6
#define APPLE_Z2_CMD_CTRL_READ_LONG      0xE7
#define APPLE_Z2_CMD_LAST                0xE1
#define APPLE_Z2_CMD_WAKE                0xEE
#define APPLE_Z2_CMD_SIZE                16
#define APPLE_Z2_RAW_XFER_MAX_SIZE       64
#define APPLE_Z2_GEN2_MIN_RESULT_SIZE    64
#define APPLE_Z2_HBPP_CMD_BLOB           0x3001
#define APPLE_Z2_FW_MAGIC                0x5746325A
#define APPLE_Z2_RX_BUF_SIZE             8192
#define APPLE_Z2_D111_CAL_SIZE           0x400
#define LOAD_COMMAND_INIT_PAYLOAD        0
#define LOAD_COMMAND_SEND_BLOB           1
#define LOAD_COMMAND_SEND_CALIBRATION    2
#define LOAD_COMMAND_WAIT_IRQ            3
#define LOAD_COMMAND_SET_CONFIG          4
#define LOAD_COMMAND_RAW_XFER            5
#define LOAD_COMMAND_SEND_CALIBRATION_V2 6
#define APPLE_Z2_RAW_XFER_RX_BIT_REVERSE BIT(0)
#define CAL_PROP_NAME                    "apple,z2-cal-blob"
#define ORB_GAP_CAL_PROP_NAME            "apple,z2-orb-gap-cal-blob"
#define ORB_FORCE_CAL_PROP_NAME          "apple,z2-orb-force-cal-blob"
#define SHAPE_ACCEL_CAL_PROP_NAME        "apple,z2-shape-accel-cal-blob"
#define DISPLAY_OFFSET_CAL_PROP_NAME     "apple,z2-display-offset-cal-blob"
#define FIREFLY_CAL_PROP_NAME            "apple,z2-firefly-cal-blob"
#define PRESSURE_CAL_PROP_NAME           "apple,z2-pressure-cal-blob"
#define FIREFLY_P100_CAL_PROP_NAME       "apple,z2-firefly-p100-cal-blob"
#define EXT_CAL_PROP_NAME                "apple,z2-ext-cal-blob"
#define PROX_CAL_PROP_NAME               "apple,z2-prox-cal-blob"

/* Classic contact reports (iPad 5), parsed like MTParse_BinaryPathOrImage. */
#define APPLE_Z2_CLASSIC_REPORT_FIRST    0x43
#define APPLE_Z2_CLASSIC_REPORT_LAST     0x45
#define APPLE_Z2_CLASSIC_HEADER_MIN      22
#define APPLE_Z2_CLASSIC_CONTACT_MIN     20
#define APPLE_Z2_CLASSIC_CONTACT_MAX     64
#define APPLE_Z2_CLASSIC_MAX_CONTACTS    32

/* N1 HBPP bootloader (iOS MTSPIBootloader_N1). */
#define APPLE_Z2_HBPP_ACK_BLOB           0x4bc1
#define APPLE_Z2_HBPP_ACK_WRITE          0x4ad1
#define APPLE_Z2_HBPP_ATTEMPTS           5
#define APPLE_Z2_N1_VERSION_REG          0x10008ffc
#define APPLE_Z2_N1_APU_EN_REG           0x10003800
#define APPLE_Z2_N1_CONST_CAL_REG        0x10003000
#define APPLE_Z2_N1_CONST_CAL_V11A0      0x434d11a0
#define APPLE_Z2_N1_REPORT_MAX           512
#define APPLE_Z2_N1_REPORT_FAMILY        0xd1
#define APPLE_Z2_N1_REPORT_SENSOR        0xd3
#define APPLE_Z2_N1_REPORT_REGION        0xd0
#define APPLE_Z2_N1_REPORT_REGION_PARAMS 0xa1
/* Runtime bounds of the iPad 5 interrupt service and its recovery. */
#define APPLE_Z2_N1_IRQ_FRAMES           8
#define APPLE_Z2_N1_FRAME_ATTEMPTS       3
#define APPLE_Z2_N1_REARM_ATTEMPTS       3
#define APPLE_Z2_N1_REARM_DELAY_MS       1000

#define APPLE_Z2_FW_CONFIG_WORDS         11
#define APPLE_Z2_FW_CONFIG_MIN_DMA       BIT(0)
#define APPLE_Z2_FW_CONFIG_Z2_DELAY      BIT(1)
#define APPLE_Z2_FW_CONFIG_CS_DELAY      BIT(2)
#define APPLE_Z2_FW_CONFIG_CPHA          BIT(3)
#define APPLE_Z2_FW_CONFIG_CPOL          BIT(4)
#define APPLE_Z2_FW_CONFIG_BOOT_TIMEOUT  BIT(8)
#define APPLE_Z2_FW_CONFIG_HBPP_VERSION  BIT(11)

enum apple_z2_protocol_state {
	APPLE_Z2_STATE_OFF,
	APPLE_Z2_STATE_POWERED,
	APPLE_Z2_STATE_SPI_CONFIGURED,
	APPLE_Z2_STATE_BOOT_IRQ,
	APPLE_Z2_STATE_HBPP_READY,
	APPLE_Z2_STATE_FIRMWARE_READY,
	APPLE_Z2_STATE_DEVICE_INFO,
	APPLE_Z2_STATE_RUNTIME,
	APPLE_Z2_STATE_REPORTS_READY,
};

enum apple_z2_variant {
	APPLE_Z2_VARIANT_GENERIC,
	APPLE_Z2_VARIANT_J172,
	APPLE_Z2_VARIANT_J71B,
	APPLE_Z2_VARIANT_J71T,
	APPLE_Z2_VARIANT_D11,
	APPLE_Z2_VARIANT_D111,
	APPLE_Z2_VARIANT_N61,
};

struct apple_z2_hbpp_reg {
	u32 addr;
	u32 value;
};

/**
 * struct apple_z2_n1_info - N1 bootloader and runtime parameters
 * @calib_regs:		register calibration written after the firmware
 * @num_calib_regs:	number of entries in @calib_regs
 * @execute_addr:	firmware entry point (fw-execute-addr)
 * @execute_arg:	execute command argument
 * @cal_size:		size of the unit's multi-touch calibration (MtCl)
 * @cal_chunk_size:	largest calibration payload per HBPP packet
 * @control_delay_us:	delay between the two phases of a control command
 * @get_delay_us:	delay before each report read
 * @wake_holdoff_ms:	wake commands within this time are suppressed
 * @get_attempts:	report reads tried while replies are corrupt
 * @device_info_attempts: device-information reads tried
 * @device_info_retry_us: delay between device-information reads
 * @interface:		expected interface version
 * @family:		expected family (report 0xd1)
 * @rows:		expected sensor rows (report 0xd3)
 * @columns:		expected sensor columns (report 0xd3)
 * @surface_width:	expected surface width (report 0xd9)
 * @surface_height:	expected surface height (report 0xd9)
 */
struct apple_z2_n1_info {
	const struct apple_z2_hbpp_reg *calib_regs;
	unsigned int num_calib_regs;
	u32 execute_addr;
	u32 execute_arg;
	unsigned int cal_size;
	unsigned int cal_chunk_size;
	unsigned int control_delay_us;
	unsigned int get_delay_us;
	unsigned int wake_holdoff_ms;
	unsigned int get_attempts;
	unsigned int device_info_attempts;
	unsigned int device_info_retry_us;
	u8 interface;
	u8 family;
	u8 rows;
	u8 columns;
	u32 surface_width;
	u32 surface_height;
};

/**
 * struct apple_z2_chip_info - per-compatible data
 * @name:		input device name
 * @variant:		protocol variant
 * @cs_setup_us:	generation-1 chip-select setup delay
 * @cs_hold_us:		generation-1 chip-select hold delay
 * @cs_inactive_us:	generation-1 chip-select inactive delay
 * @n1:			N1 bootloader and runtime parameters
 */
struct apple_z2_chip_info {
	const char *name;
	enum apple_z2_variant variant;
	unsigned int cs_setup_us;
	unsigned int cs_hold_us;
	unsigned int cs_inactive_us;
	const struct apple_z2_n1_info *n1;
};

struct apple_z2 {
	struct spi_device *spidev;
	const struct apple_z2_chip_info *info;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_ana_gpio;
	struct gpio_desc *power_ldo_gpio;
	struct gpio_desc *display_sync_gpio;
	struct gpio_desc *display_sync1_gpio;
	struct pinctrl *j172_sync_pinctrl;
	struct pinctrl_state *j172_sync_active_state;
	struct regulator *hv_supply;
	struct regulator *core_supply;
	struct regulator *ana_supply;
	struct pinctrl *sync_pinctrl;
	struct pinctrl_state *sync_active;
	struct pinctrl_state *sync_inactive;
	struct gpio_desc *attn_gpio;
	struct delayed_work rearm_work;
	unsigned int rearm_attempts;
	unsigned long last_wake;
	bool wake_valid;
	bool ana_enabled;
	bool irq_on;
	bool stopping;
	struct clk *clk;
	struct input_dev *input_dev;
	struct completion boot_irq;
	struct mutex io_lock; /* Serializes command and IRQ transfers. */
	enum apple_z2_variant variant;
	bool no_init_ack;
	bool booted;
	bool clk_enabled;
	bool platform_powered;
	bool upload_irq_masked;
	bool runtime_frame_valid;
	bool runtime_queue_empty;
	bool runtime_control_reply;
	bool irq_type_changed;
	unsigned int saved_irq_type;
	bool surface_descriptor_valid;
	bool j172_sync_pinctrl_active;
	bool hv_enabled;
	bool core_enabled;
	enum apple_z2_protocol_state protocol_state;
	unsigned int bpw16_min_len;
	unsigned int dma_min_len;
	unsigned int z2_inter_packet_delay_us;
	unsigned int z2_cs_delay_us;
	unsigned int boot_timeout_ms;
	u32 hbpp_version;
	s16 sensor_max_x;
	s16 sensor_min_x;
	s16 sensor_max_y;
	s16 sensor_min_y;
	int index_parity;
	struct touchscreen_properties props;
	const char *fw_name;
	u8 *tx_buf;
	u8 *rx_buf;
};

static bool apple_z2_uses_ipad_protocol(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_J172 ||
	       z2->variant == APPLE_Z2_VARIANT_J71B;
}

static bool apple_z2_is_j71b(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_J71B;
}

static bool apple_z2_is_n61(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_N61;
}

static bool apple_z2_is_j71t(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_J71T;
}

/* The generation-1 byte-stream runtime protocol of the iPad 5 and 6. */
static bool apple_z2_is_gen1(const struct apple_z2 *z2)
{
	return apple_z2_is_j71b(z2) || apple_z2_is_j71t(z2);
}

static bool apple_z2_uses_gen1_data(const struct apple_z2 *z2)
{
	return apple_z2_is_gen1(z2) || apple_z2_is_n61(z2);
}

static bool apple_z2_is_d11(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_D11;
}

static bool apple_z2_is_d111(const struct apple_z2 *z2)
{
	return z2->variant == APPLE_Z2_VARIANT_D111;
}

static bool apple_z2_is_iphone7_plus(const struct apple_z2 *z2)
{
	return apple_z2_is_d11(z2) || apple_z2_is_d111(z2);
}

struct apple_z2_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_z2_hbpp_blob_hdr {
	__le16 cmd;
	__le16 len;
	__le32 addr;
	__le16 checksum;
};

struct apple_z2_fw_hdr {
	__le32 magic;
	__le32 version;
};

struct apple_z2_fw_calibration {
	__le32 address;
	__le32 max_size;
	__le32 provider;
};

enum apple_z2_calibration_provider {
	APPLE_Z2_CAL_MULTI_TOUCH,
	APPLE_Z2_CAL_ORB_GAP,
	APPLE_Z2_CAL_ORB_FORCE,
	APPLE_Z2_CAL_SHAPE_ACCEL,
	APPLE_Z2_CAL_DISPLAY_OFFSET,
	APPLE_Z2_CAL_FIREFLY,
	APPLE_Z2_CAL_PRESSURE,
	APPLE_Z2_CAL_FIREFLY_P100,
	APPLE_Z2_CAL_EXT_MULTI_TOUCH,
	APPLE_Z2_CAL_PROX = 9,
};

static void apple_z2_apply_z2_delays(struct apple_z2 *z2);
static void apple_z2_post_z2_xfer_delay(struct apple_z2 *z2);
static void apple_z2_set_gpio(struct gpio_desc *gpio, int value);
static void apple_z2_z2_checksum(u8 *cmd, unsigned int len);
static bool apple_z2_z2_checksum_valid(const u8 *cmd, unsigned int len);
static int apple_z2_hbpp_xfer(struct apple_z2 *z2, unsigned int len);
static int apple_z2_hbpp_nop(struct apple_z2 *z2);
static void apple_z2_platform_power_off(struct apple_z2 *z2);
static int apple_z2_j71t_boot(struct apple_z2 *z2);

static bool apple_z2_float_to_integer(const u8 *raw, s64 *value)
{
	u32 bits = get_unaligned_le32(raw);
	u32 exponent = (bits >> 23) & 0xff;
	u64 significand;
	u64 magnitude;
	int shift;

	if (exponent == 0xff)
		return false;
	if (!exponent) {
		*value = 0;
		return true;
	}

	significand = BIT_ULL(23) | (bits & GENMASK(22, 0));
	shift = exponent - 127 - 23;
	if (shift >= 0) {
		if (shift > 7)
			return false;
		magnitude = significand << shift;
	} else if (-shift >= 64) {
		magnitude = 0;
	} else {
		magnitude = DIV_ROUND_CLOSEST_ULL(significand,
						  BIT_ULL(-shift));
	}

	if (magnitude > S32_MAX)
		return false;
	*value = bits & BIT(31) ? -(s64)magnitude : magnitude;
	return true;
}

static bool apple_z2_scale_j172_coord(const u8 *raw, s16 maximum,
				      s16 minimum, unsigned int pixels,
				      unsigned int *position)
{
	s64 coordinate;
	s64 numerator;
	s64 denominator;

	if (maximum <= minimum || !apple_z2_float_to_integer(raw, &coordinate))
		return false;

	/* The report uses micrometres; the descriptor uses 0.01 mm. */
	numerator = coordinate - 10LL * minimum;
	denominator = 10LL * (maximum - minimum);
	if (numerator <= 0) {
		*position = 0;
	} else if (numerator >= denominator) {
		*position = pixels;
	} else {
		*position = DIV_ROUND_CLOSEST_ULL((u64)numerator * pixels,
						  denominator);
	}

	return true;
}

static bool apple_z2_scale_iphone7_coord(s16 coordinate, s16 maximum,
					 s16 minimum, unsigned int pixels,
					 unsigned int *position)
{
	s64 numerator;
	s64 denominator;

	if (maximum <= minimum)
		return false;

	numerator = coordinate - minimum;
	denominator = maximum - minimum;
	if (numerator <= 0)
		*position = 0;
	else if (numerator >= denominator)
		*position = pixels;
	else
		*position = DIV_ROUND_CLOSEST_ULL((u64)numerator * pixels,
						  denominator);

	return true;
}

static void apple_z2_parse_j172_touches(struct apple_z2 *z2,
					const u8 *msg, size_t msg_len)
{
	unsigned int contact_offset;
	unsigned int contact_bytes;
	unsigned int nfingers;
	unsigned int x;
	unsigned int y;
	unsigned int i;
	const u8 *contact;
	int slot;
	bool active;
	bool coords_valid;

	if (msg_len < APPLE_Z2_J172_HEADER_SIZE)
		return;

	contact_offset = msg[2] + msg[14];
	contact_bytes = get_unaligned_le16(msg + 16);
	nfingers = msg[22];
	if (msg[2] < APPLE_Z2_J172_HEADER_SIZE ||
	    !(get_unaligned_le16(msg + 12) & APPLE_Z2_J172_FLOAT_FORMAT) ||
	    nfingers > APPLE_Z2_J172_MAX_CONTACTS ||
	    contact_bytes != nfingers * APPLE_Z2_J172_CONTACT_SIZE ||
	    contact_offset > msg_len ||
	    contact_bytes > msg_len - contact_offset) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid J172 touch packet layout len=%zu header=%u contacts=%u bytes=%u\n",
				     msg_len, msg[2], nfingers, contact_bytes);
		return;
	}

	if (nfingers && !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "J172 touch packet without surface descriptor\n");
		return;
	}

	contact = msg + contact_offset;
	for (i = 0; i < nfingers; i++, contact += APPLE_Z2_J172_CONTACT_SIZE) {
		slot = input_mt_get_slot_by_key(z2->input_dev, contact[0]);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev,
				 "unable to get slot for J172 finger\n");
			continue;
		}

		active = contact[1] == APPLE_Z2_TOUCH_STARTED ||
			 contact[1] == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, active))
			continue;

		coords_valid = apple_z2_scale_j172_coord(contact + 4,
							 z2->sensor_max_x,
							 z2->sensor_min_x,
							 z2->props.max_x, &x);
		if (coords_valid)
			coords_valid = apple_z2_scale_j172_coord(contact + 8,
								 z2->sensor_max_y,
								 z2->sensor_min_y,
								 z2->props.max_y, &y);
		if (!coords_valid) {
			dev_warn_ratelimited(&z2->spidev->dev,
					     "invalid J172 touch coordinates\n");
			continue;
		}

		touchscreen_report_pos(z2->input_dev, &z2->props, x,
				       z2->props.max_y - y, true);
	}

	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

static void apple_z2_parse_touches(struct apple_z2 *z2,
				   const u8 *msg, size_t msg_len)
{
	s16 abs_x;
	s16 abs_y;
	s16 orientation;
	bool coords_valid;
	unsigned int x;
	unsigned int y;
	int i;
	int nfingers;
	int slot;
	int slot_valid;
	struct apple_z2_finger *fingers;

	if (!apple_z2_is_n61(z2) && msg_len >= APPLE_Z2_J172_HEADER_SIZE &&
	    msg[0] == APPLE_Z2_HID_REPORT_ID && msg[3] == 4) {
		apple_z2_parse_j172_touches(z2, msg, msg_len);
		return;
	}

	if (apple_z2_is_j71b(z2)) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "unknown contact layout, len=%zu\n", msg_len);
		return;
	}

	if (msg_len <= APPLE_Z2_NUM_FINGERS_OFFSET)
		return;
	nfingers = msg[APPLE_Z2_NUM_FINGERS_OFFSET];
	if (msg_len < APPLE_Z2_FINGERS_OFFSET ||
	    nfingers > (msg_len - APPLE_Z2_FINGERS_OFFSET) /
		       sizeof(*fingers)) {
		dev_warn(&z2->spidev->dev,
			 "invalid touch packet length %zu for %d fingers\n",
			 msg_len, nfingers);
		return;
	}
	if ((apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) && nfingers &&
	    !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "iPhone 7 Plus touch packet without surface descriptor\n");
		return;
	}
	fingers = (struct apple_z2_finger *)(msg + APPLE_Z2_FINGERS_OFFSET);
	for (i = 0; i < nfingers; i++) {
		slot = input_mt_get_slot_by_key(z2->input_dev, fingers[i].finger);
		if (slot < 0) {
			dev_warn(&z2->spidev->dev, "unable to get slot for finger\n");
			continue;
		}
		slot_valid = fingers[i].state == APPLE_Z2_TOUCH_STARTED ||
			     fingers[i].state == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER, slot_valid))
			continue;
		if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) {
			abs_x = (s16)le16_to_cpu(fingers[i].abs_x);
			abs_y = (s16)le16_to_cpu(fingers[i].abs_y);
			coords_valid = apple_z2_scale_iphone7_coord(abs_x,
								    z2->sensor_max_x,
								    z2->sensor_min_x,
								    z2->props.max_x, &x);
			if (coords_valid)
				coords_valid = apple_z2_scale_iphone7_coord(abs_y,
									    z2->sensor_max_y,
									    z2->sensor_min_y,
									    z2->props.max_y,
									    &y);
			if (!coords_valid) {
				dev_warn_ratelimited(&z2->spidev->dev,
						     "invalid iPhone 7 Plus touch coordinates\n");
				continue;
			}
			touchscreen_report_pos(z2->input_dev, &z2->props, x,
					       z2->props.max_y - y,
					       true);
		} else {
			touchscreen_report_pos(z2->input_dev, &z2->props,
					       le16_to_cpu(fingers[i].abs_x),
					       le16_to_cpu(fingers[i].abs_y),
					       true);
		}
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MAJOR,
				 le16_to_cpu(fingers[i].tool_major));
		input_report_abs(z2->input_dev, ABS_MT_WIDTH_MINOR,
				 le16_to_cpu(fingers[i].tool_minor));
		orientation = (s16)le16_to_cpu(fingers[i].orientation);
		if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2))
			orientation = (s16)(0x4000 - orientation);
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION, orientation);
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 le16_to_cpu(fingers[i].touch_major));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 le16_to_cpu(fingers[i].touch_minor));
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

/*
 * Classic contact reports 0x43..0x45 (iOS MultitouchSupport
 * MTParse_BinaryFrameHeader and mt_FillMTContactDirectFromBinary): byte 2
 * is the header length, LE16 at 14 the image bytes after the header, byte
 * 16 the contact count and byte 17 the contact stride. Each contact holds
 * its ID, its state, signed LE16 X/Y at 4/6 in 0.01 mm, the ellipse radii
 * at 12/14 and the angle at 16.
 */
static void apple_z2_parse_classic_touches(struct apple_z2 *z2,
					   const u8 *report, size_t len)
{
	DECLARE_BITMAP(ids, 256);
	unsigned int header_len, offset, stride, count, i, x, y;
	const u8 *contact;
	bool active;
	int slot;

	if (!len || report[0] < APPLE_Z2_CLASSIC_REPORT_FIRST ||
	    report[0] > APPLE_Z2_CLASSIC_REPORT_LAST)
		return;
	if (len < APPLE_Z2_CLASSIC_HEADER_MIN)
		goto invalid;

	header_len = report[2];
	count = report[16];
	stride = report[17];
	offset = header_len + get_unaligned_le16(report + 14);
	if (header_len < APPLE_Z2_CLASSIC_HEADER_MIN || offset > len ||
	    count > APPLE_Z2_CLASSIC_MAX_CONTACTS ||
	    (count && (stride < APPLE_Z2_CLASSIC_CONTACT_MIN ||
		       stride > APPLE_Z2_CLASSIC_CONTACT_MAX ||
		       count > (len - offset) / stride)))
		goto invalid;

	if (count && !z2->surface_descriptor_valid) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "touch report without surface descriptor\n");
		return;
	}

	/* Check the whole report before changing any slot. */
	bitmap_zero(ids, 256);
	for (i = 0; i < count; i++)
		if (test_and_set_bit(report[offset + i * stride], ids))
			goto invalid;

	for (i = 0; i < count; i++) {
		contact = report + offset + i * stride;
		slot = input_mt_get_slot_by_key(z2->input_dev, contact[0]);
		if (slot < 0) {
			dev_warn_ratelimited(&z2->spidev->dev,
					     "unable to get slot for finger\n");
			continue;
		}

		active = contact[1] == APPLE_Z2_TOUCH_STARTED ||
			 contact[1] == APPLE_Z2_TOUCH_MOVED;
		input_mt_slot(z2->input_dev, slot);
		if (!input_mt_report_slot_state(z2->input_dev, MT_TOOL_FINGER,
						active))
			continue;

		if (!apple_z2_scale_iphone7_coord((s16)get_unaligned_le16(contact + 4),
						  z2->sensor_max_x,
						  z2->sensor_min_x,
						  z2->props.max_x, &x) ||
		    !apple_z2_scale_iphone7_coord((s16)get_unaligned_le16(contact + 6),
						  z2->sensor_max_y,
						  z2->sensor_min_y,
						  z2->props.max_y, &y))
			continue;

		/* Raw Y grows upwards; the DT inverts it for the panel. */
		touchscreen_report_pos(z2->input_dev, &z2->props, x, y, true);
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MAJOR,
				 get_unaligned_le16(contact + 12));
		input_report_abs(z2->input_dev, ABS_MT_TOUCH_MINOR,
				 get_unaligned_le16(contact + 14));
		input_report_abs(z2->input_dev, ABS_MT_ORIENTATION,
				 (s16)get_unaligned_le16(contact + 16));
	}

	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
	return;

invalid:
	dev_warn_ratelimited(&z2->spidev->dev,
			     "invalid touch report %02x, len=%zu\n",
			     report[0], len);
}

static void apple_z2_dispatch_frame(struct apple_z2 *z2, const u8 *payload,
				    size_t payload_len)
{
	const u8 *report;
	u16 report_len;

	/* The iPad 5 sends classic reports without the HID envelope. */
	if (apple_z2_is_j71t(z2)) {
		apple_z2_parse_classic_touches(z2, payload, payload_len);
		return;
	}

	/* N61 Gen1 and iPhone 7 Plus carry the legacy report header directly. */
	if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) {
		if (payload_len < APPLE_Z2_FINGERS_OFFSET)
			return;

		/*
		 * D111 hardware traces identify 0x44 as its touch report.  No D11
		 * runtime capture currently establishes a report identifier, so
		 * retain its length-based dispatch until that protocol is verified.
		 */
		if (apple_z2_is_d111(z2) &&
		    payload[0] != APPLE_Z2_D111_TOUCH_REPORT)
			return;

		apple_z2_parse_touches(z2, payload, payload_len);
		return;
	}

	if (payload_len < 4 || payload[0] != APPLE_Z2_FRAME_INPUT_REPORT)
		return;

	report_len = get_unaligned_le16(payload + 2);
	if (report_len != payload_len - 4) {
		dev_warn_ratelimited(&z2->spidev->dev,
				     "invalid input report length %u for payload %zu\n",
				     report_len, payload_len);
		return;
	}

	if (!(payload[1] & APPLE_Z2_INTERFACE_GRAPE) || !report_len)
		return;

	report = payload + 4;
	if (report[0] != APPLE_Z2_HID_REPORT_ID)
		return;

	apple_z2_parse_touches(z2, report, report_len);
}

static bool apple_z2_gen2_packet_valid(const u8 *buf, size_t buf_len,
				       u8 counter, bool strict_counter,
				       bool check_counter)
{
	bool counter_valid;
	u16 payload_len;
	u16 checksum;
	int i;

	if (buf_len < 7)
		return false;

	counter_valid = !check_counter ||
			(strict_counter ? buf[1] == counter :
					  buf[1] == 1 || buf[1] == 2);
	payload_len = get_unaligned_le16(buf + 2);
	if (((buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) & 0xff) ||
	    (buf[0] & 0xfe) != 0xea || !counter_valid ||
	    payload_len < 2 || payload_len + 5 > buf_len)
		return false;

	checksum = get_unaligned_le16(buf + 5 + payload_len - 2);
	for (i = 0; i < payload_len - 2; i++)
		checksum -= buf[5 + i];

	return !checksum;
}

/*
 * Generation-1 data phases: the controller expects the request bytes to
 * keep streaming while it answers, so transmit from a copy. On J71b,
 * transfers above 16 bytes run in SPI mode 0 at 500 kHz.
 */
static int apple_z2_gen1_exchange(struct apple_z2 *z2, u8 *buf, size_t len)
{
	struct spi_transfer xfer = {
		.rx_buf = buf,
		.len = len,
		.bits_per_word = 8,
	};
	u32 saved_mode = z2->spidev->mode;
	bool mode_changed = apple_z2_is_j71b(z2) && len > 16;
	u8 *tx;
	int error, restore;

	if (!len || len > APPLE_Z2_RX_BUF_SIZE)
		return -EMSGSIZE;
	if (apple_z2_uses_gen1_data(z2))
		xfer.speed_hz = 500000;
	tx = kmemdup(buf, len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;
	xfer.tx_buf = tx;

	if (mode_changed) {
		z2->spidev->mode = saved_mode & ~SPI_CPHA;
		error = spi_setup(z2->spidev);
		if (error)
			goto restore_mode;
	}
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
restore_mode:
	if (mode_changed) {
		z2->spidev->mode = saved_mode;
		restore = spi_setup(z2->spidev);
		if (!error)
			error = restore;
	}
	kfree(tx);
	return error;
}

static int apple_z2_read_packet(struct apple_z2 *z2)
{
	struct spi_transfer xfer = { };
	bool strict_counter;
	int error;
	size_t pkt_len;
	size_t wire_len;
	u16 payload_len;
	u8 counter;

	z2->runtime_frame_valid = false;
	z2->runtime_queue_empty = false;
	z2->runtime_control_reply = false;
	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
	counter = z2->index_parity + 1;
	z2->tx_buf[1] = counter;
	apple_z2_z2_checksum(z2->tx_buf, APPLE_Z2_CMD_SIZE);
	xfer.tx_buf = z2->tx_buf;
	xfer.rx_buf = z2->rx_buf;
	xfer.len = APPLE_Z2_CMD_SIZE;

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);
	if (apple_z2_is_n61(z2)) {
		if (!apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
			return -EPROTO;
	}

	/*
	 * The iPad 5 (iOS 0xfffffff005ea1cf4) checks the whole length reply:
	 * an E-class response with the byte sum of its first 14 bytes at 14.
	 */
	if (apple_z2_is_j71t(z2)) {
		if ((z2->rx_buf[0] & 0xf0) != 0xe0 ||
		    !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
			return -EBADMSG;
	} else if (z2->rx_buf[0] != APPLE_Z2_REPLY_INTERRUPT_DATA) {
		return 0;
	}

	payload_len = get_unaligned_le16(z2->rx_buf + 1);
	if (apple_z2_uses_gen1_data(z2) && !payload_len) {
		z2->runtime_queue_empty = true;
		if (apple_z2_is_n61(z2))
			z2->index_parity = !z2->index_parity;
		return 0;
	}
	if (apple_z2_is_j71t(z2))
		pkt_len = (size_t)payload_len + 5;
	else if (apple_z2_uses_gen1_data(z2))
		pkt_len = max_t(size_t, 16, (size_t)payload_len + 5);
	else
		pkt_len = (payload_len + 8) & 0xfffffffc;
	if (pkt_len > APPLE_Z2_RX_BUF_SIZE) {
		dev_warn(&z2->spidev->dev, "packet too large: %zu\n", pkt_len);
		return -EMSGSIZE;
	}
	wire_len = pkt_len;
	if (apple_z2_is_d11(z2))
		wire_len = max_t(size_t, wire_len,
				 APPLE_Z2_GEN2_MIN_RESULT_SIZE);

	if (apple_z2_uses_gen1_data(z2)) {
		/* Data phase acknowledges the queued report. */
		if (apple_z2_is_n61(z2)) {
			error = apple_z2_n61_interrupt_request(z2->rx_buf,
							       wire_len, counter);
			if (error)
				return error;
		} else {
			memset(z2->rx_buf, 0, max_t(size_t, wire_len, APPLE_Z2_CMD_SIZE));
			z2->rx_buf[0] = APPLE_Z2_CMD_READ_INTERRUPT_DATA;
			z2->rx_buf[1] = counter;
			z2->rx_buf[2] = 1;
			apple_z2_z2_checksum(z2->rx_buf, APPLE_Z2_CMD_SIZE);
		}
		/*
		 * The iPad 5 (iOS 0xfffffff005ea1f24) carries the same checksum
		 * in the last two bytes of the data phase, not at 14.
		 */
		if (apple_z2_is_j71t(z2)) {
			u16 checksum = get_unaligned_le16(z2->rx_buf + 14);

			put_unaligned_le16(0, z2->rx_buf + 14);
			put_unaligned_le16(checksum, z2->rx_buf + wire_len - 2);
		}
		error = apple_z2_gen1_exchange(z2, z2->rx_buf, wire_len);
	} else if (apple_z2_uses_ipad_protocol(z2) ||
		   apple_z2_is_iphone7_plus(z2)) {
		memset(z2->rx_buf, 0xa5, wire_len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = wire_len;
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
	} else {
		error = spi_read(z2->spidev, z2->rx_buf, pkt_len);
	}
	if (error)
		return error;
	if (apple_z2_is_iphone7_plus(z2))
		apple_z2_post_z2_xfer_delay(z2);

	/*
	 * Generation-1 firmware answers no-data (0) or a checksummed control
	 * reply while service is pending. Neither is a frame; keep the tag.
	 */
	if (apple_z2_uses_gen1_data(z2) &&
	    (!z2->rx_buf[0] ||
	     (wire_len >= 16 && z2->rx_buf[0] == APPLE_Z2_CMD_LAST &&
	      apple_z2_z2_checksum_valid(z2->rx_buf, 16)))) {
		z2->runtime_control_reply = true;
		return 0;
	}

	/* D11 touch reports do not follow the requested alternating tag. */
	strict_counter = !apple_z2_is_d11(z2) ||
			 (pkt_len > 5 && z2->rx_buf[5] ==
					 APPLE_Z2_D11_STATUS_REPORT);
	/*
	 * The iPad 5 firmware's data header is not known to echo the tag, and
	 * iOS does not check it there; its payload has to fill the transfer.
	 */
	z2->runtime_frame_valid =
		apple_z2_gen2_packet_valid(z2->rx_buf, pkt_len, counter,
					   strict_counter,
					   !apple_z2_is_j71t(z2)) &&
		(!apple_z2_is_j71t(z2) ||
		 get_unaligned_le16(z2->rx_buf + 2) + 5 == pkt_len);
	if (!z2->runtime_frame_valid)
		return 0;

	payload_len = get_unaligned_le16(z2->rx_buf + 2);
	apple_z2_dispatch_frame(z2, z2->rx_buf + 5, payload_len - 2);
	z2->index_parity = !z2->index_parity;

	return 0;
}

static void apple_z2_reset_protocol(struct apple_z2 *z2)
{
	z2->protocol_state = APPLE_Z2_STATE_OFF;
	z2->index_parity = 0;
	z2->runtime_frame_valid = false;
	z2->surface_descriptor_valid = false;
}

static int apple_z2_advance_protocol(struct apple_z2 *z2,
				     enum apple_z2_protocol_state expected,
				     enum apple_z2_protocol_state next)
{
	if (z2->protocol_state != expected)
		return -EPROTO;

	z2->protocol_state = next;
	return 0;
}

static void apple_z2_z2_checksum(u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];
	put_unaligned_le16(checksum, cmd + len - 2);
}

static bool apple_z2_z2_checksum_valid(const u8 *cmd, unsigned int len)
{
	u16 checksum = 0;
	int i;

	if (len < 2)
		return false;

	for (i = 0; i < len - 2; i++)
		checksum += cmd[i];

	return get_unaligned_le16(cmd + len - 2) == checksum;
}

static void apple_z2_set_spi_delay(struct spi_delay *delay,
				   unsigned int usecs)
{
	if (usecs > U16_MAX)
		usecs = U16_MAX;

	delay->value = usecs;
	delay->unit = SPI_DELAY_UNIT_USECS;
}

static unsigned int apple_z2_effective_cs_delay_us(struct apple_z2 *z2)
{
	/* J172 firmware expresses this field in ns; Linux SPI uses us here. */
	if (apple_z2_uses_gen1_data(z2))
		return z2->info->cs_setup_us;
	return apple_z2_uses_ipad_protocol(z2) ? 10 : z2->z2_cs_delay_us;
}

static void apple_z2_apply_z2_delays(struct apple_z2 *z2)
{
	unsigned int cs_delay_us = apple_z2_effective_cs_delay_us(z2);
	unsigned int hold_us = cs_delay_us;
	unsigned int inactive_us = apple_z2_uses_ipad_protocol(z2) ? 10 :
				   z2->z2_inter_packet_delay_us;

	if (apple_z2_uses_gen1_data(z2)) {
		hold_us = z2->info->cs_hold_us;
		inactive_us = z2->info->cs_inactive_us;
	}

	if (cs_delay_us) {
		apple_z2_set_spi_delay(&z2->spidev->cs_setup, cs_delay_us);
		apple_z2_set_spi_delay(&z2->spidev->cs_hold, hold_us);
	}
	if (inactive_us)
		apple_z2_set_spi_delay(&z2->spidev->cs_inactive, inactive_us);
}

static void apple_z2_post_z2_xfer_delay(struct apple_z2 *z2)
{
	unsigned int delay_us = z2->z2_inter_packet_delay_us;

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static void apple_z2_post_ack_delay(struct apple_z2 *z2)
{
	unsigned int delay_us = apple_z2_effective_cs_delay_us(z2);

	if (delay_us)
		usleep_range(delay_us, delay_us + 100);
}

static void apple_z2_avoid_short_dma(struct apple_z2 *z2,
				     struct spi_transfer *xfer)
{
	/*
	 * iPhone 7 Plus firmware describes a target-specific minimum
	 * DMA transfer length, not a property of the SPI controller.  Supplying
	 * an RX buffer makes shorter writes full-duplex, which the T8010
	 * controller handles through PIO without changing the wire transfer.
	 */
	if (!apple_z2_is_iphone7_plus(z2) || !z2->dma_min_len ||
	    !xfer->tx_buf || xfer->rx_buf ||
	    xfer->len >= z2->dma_min_len)
		return;

	memset(z2->rx_buf, 0, xfer->len);
	xfer->rx_buf = z2->rx_buf;
}

/* Sends the 16-byte command in tx_buf as it is. */
static int apple_z2_z2_raw_xfer_locked(struct apple_z2 *z2, const char *tag,
				       bool post_delay)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = APPLE_Z2_CMD_SIZE,
	};
	int error;

	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error) {
		dev_err(&z2->spidev->dev, "%s transfer failed: %d\n",
			tag, error);
		return error;
	}
	if (post_delay)
		apple_z2_post_z2_xfer_delay(z2);

	dev_dbg(&z2->spidev->dev, "%s tx=%*ph rx=%*ph\n", tag,
		APPLE_Z2_CMD_SIZE, z2->tx_buf, APPLE_Z2_CMD_SIZE,
		z2->rx_buf);
	return 0;
}

static int apple_z2_z2_xfer_locked_delay(struct apple_z2 *z2,
					 const char *tag, bool post_delay)
{
	apple_z2_z2_checksum(z2->tx_buf, APPLE_Z2_CMD_SIZE);
	return apple_z2_z2_raw_xfer_locked(z2, tag, post_delay);
}

static int apple_z2_z2_xfer_locked(struct apple_z2 *z2, const char *tag)
{
	return apple_z2_z2_xfer_locked_delay(z2, tag, true);
}

static int apple_z2_z2_cmd_locked_delay(struct apple_z2 *z2, u8 cmd,
					u8 report, const char *tag,
					bool post_delay)
{
	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = cmd;
	z2->tx_buf[1] = report;

	return apple_z2_z2_xfer_locked_delay(z2, tag, post_delay);
}

static int apple_z2_z2_cmd_locked(struct apple_z2 *z2, u8 cmd, u8 report,
				  const char *tag)
{
	return apple_z2_z2_cmd_locked_delay(z2, cmd, report, tag, true);
}

static int apple_z2_read_device_info_locked(struct apple_z2 *z2)
{
	u16 max_packet_size;
	u8 interface_version;
	u8 platform_id;
	int error;

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_WAKE, 0,
					     "device-info-wake", false);
	if (error)
		return error;
	usleep_range(2000, 2100);

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_DEVICE_INFO, 0,
					     "device-info", false);
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_LAST, 0,
					     "device-info-last", false);
	if (error)
		return error;

	if (z2->rx_buf[0] != APPLE_Z2_CMD_DEVICE_INFO ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
		return -EPROTO;

	platform_id = z2->rx_buf[1];
	interface_version = z2->rx_buf[2];
	max_packet_size = get_unaligned_le16(z2->rx_buf + 3);

	dev_dbg(&z2->spidev->dev,
		"device info: platform=%u interface=%u max-packet=%u\n",
		 platform_id, interface_version, max_packet_size);
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_FIRMWARE_READY,
					 APPLE_Z2_STATE_DEVICE_INFO);
}

static int apple_z2_write_report_locked(struct apple_z2 *z2, u8 report,
					const u8 *data, u8 len,
					const char *tag)
{
	char last_tag[32];
	int error;

	if (len > 11)
		return -EINVAL;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_CTRL_WRITE_SHORT;
	z2->tx_buf[1] = report;
	z2->tx_buf[2] = len;
	memcpy(z2->tx_buf + 3, data, len);
	error = apple_z2_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = APPLE_Z2_CMD_LAST;
	snprintf(last_tag, sizeof(last_tag), "%s-last", tag);
	error = apple_z2_z2_xfer_locked(z2, last_tag);
	if (error)
		return error;

	if (apple_z2_is_d11(z2) &&
	    (z2->rx_buf[0] != APPLE_Z2_CMD_LAST ||
	     !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE)))
		return -EPROTO;

	return 0;
}

static int apple_z2_enable_d11_reports_locked(struct apple_z2 *z2)
{
	static const u8 enable = 1;

	return apple_z2_write_report_locked(z2, APPLE_Z2_D11_REPORT_ENABLE,
					    &enable, sizeof(enable), "d11-e2");
}

static int apple_z2_read_report_info_locked(struct apple_z2 *z2, u8 report,
					    u8 *status, u16 *len,
					    const char *tag)
{
	char xfer_tag[40];
	unsigned int i;
	int error;

	for (i = 0; i < 2; i++) {
		snprintf(xfer_tag, sizeof(xfer_tag), "%s-info-%u", tag, i + 1);
		error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_REPORT_INFO,
					       report, xfer_tag);
		if (error)
			return error;
	}

	if (z2->rx_buf[0] != APPLE_Z2_CMD_REPORT_INFO ||
	    (z2->rx_buf[1] != report && !apple_z2_is_j71t(z2)) ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE))
		return -EPROTO;

	*status = z2->rx_buf[2];
	*len = get_unaligned_le16(z2->rx_buf + 3);
	dev_dbg(&z2->spidev->dev,
		"%s report=%02x status=%u len=%u\n", tag, report,
		*status, *len);

	return 0;
}

static int apple_z2_read_report_locked(struct apple_z2 *z2, u8 report,
				       u16 len, const char *tag)
{
	struct spi_transfer xfer = { };
	char last_tag[32];
	unsigned int data_len;
	int error;

	if (!len || len > APPLE_Z2_RX_BUF_SIZE - 5)
		return -EMSGSIZE;

	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = len <= 11 ? APPLE_Z2_CMD_CTRL_READ_SHORT :
					 APPLE_Z2_CMD_CTRL_READ_LONG;
	z2->tx_buf[1] = report;
	put_unaligned_le16(len, z2->tx_buf + 3);
	error = apple_z2_z2_xfer_locked(z2, tag);
	if (error)
		return error;

	if (len <= 11) {
		if (apple_z2_is_n61(z2)) {
			apple_z2_n61_short_read_request(z2->rx_buf, report, len);
			error = apple_z2_gen1_exchange(z2, z2->rx_buf, APPLE_Z2_CMD_SIZE);
			if (!error)
				apple_z2_post_z2_xfer_delay(z2);
			return error;
		}
		snprintf(last_tag, sizeof(last_tag), "%s-last", tag);
		error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_LAST, 0,
					       last_tag);
		if (!error)
			dev_dbg(&z2->spidev->dev, "%s short read done\n",
				tag);
		return error;
	}

	data_len = len + 5;
	apple_z2_apply_z2_delays(z2);
	if (apple_z2_uses_gen1_data(z2)) {
		/* Long read request repeated with phase 1 and a checksum. */
		if (apple_z2_is_n61(z2)) {
			apple_z2_n61_long_read_request(z2->rx_buf, report, len);
		} else {
			memset(z2->rx_buf, 0, data_len);
			z2->rx_buf[0] = APPLE_Z2_CMD_CTRL_READ_LONG;
			z2->rx_buf[1] = report;
			z2->rx_buf[2] = 1;
			put_unaligned_le16(len, z2->rx_buf + 3);
			apple_z2_z2_checksum(z2->rx_buf, data_len);
		}
		error = apple_z2_gen1_exchange(z2, z2->rx_buf, data_len);
	} else {
		memset(z2->rx_buf, 0xa5, data_len);
		xfer.tx_buf = z2->rx_buf;
		xfer.rx_buf = z2->rx_buf;
		xfer.len = data_len;
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
	}
	if (error)
		return error;
	apple_z2_post_z2_xfer_delay(z2);
	dev_dbg(&z2->spidev->dev, "%s long read len=%u\n", tag, data_len);

	return 0;
}

/*
 * Surface descriptor (report 0xd9): LE32 width and height in 0.01 mm, then
 * the signed LE16 left, bottom, right and top edges.
 */
static int apple_z2_set_surface(struct apple_z2 *z2, const u8 *descriptor)
{
	unsigned int x_pixels;
	unsigned int y_pixels;
	unsigned int x_res;
	unsigned int y_res;
	s16 max_x;
	s16 min_x;
	s16 max_y;
	s16 min_y;
	u32 width;
	u32 height;

	width = get_unaligned_le32(descriptor);
	height = get_unaligned_le32(descriptor + 4);
	min_x = (s16)get_unaligned_le16(descriptor + 8);
	max_x = (s16)get_unaligned_le16(descriptor + 12);
	min_y = (s16)get_unaligned_le16(descriptor + 10);
	max_y = (s16)get_unaligned_le16(descriptor + 14);
	if (!width || !height || max_x <= min_x || max_y <= min_y)
		return -EPROTO;

	z2->sensor_max_x = max_x;
	z2->sensor_min_x = min_x;
	z2->sensor_max_y = max_y;
	z2->sensor_min_y = min_y;
	z2->surface_descriptor_valid = true;

	x_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_X) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_X) + 1;
	y_pixels = input_abs_get_max(z2->input_dev, ABS_MT_POSITION_Y) -
		   input_abs_get_min(z2->input_dev, ABS_MT_POSITION_Y) + 1;
	x_res = max(DIV_ROUND_CLOSEST(x_pixels * 100, width), 1U);
	y_res = max(DIV_ROUND_CLOSEST(y_pixels * 100, height), 1U);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(z2->input_dev, ABS_MT_POSITION_Y, y_res);
	if (test_bit(ABS_X, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_X, x_res);
	if (test_bit(ABS_Y, z2->input_dev->absbit))
		input_abs_set_res(z2->input_dev, ABS_Y, y_res);

	dev_dbg(&z2->spidev->dev,
		"surface %ux%u, bounds x=%d..%d y=%d..%d, resolution %ux%u units/mm\n",
		 width, height, min_x, max_x, min_y, max_y, x_res, y_res);

	return 0;
}

static int apple_z2_store_surface_descriptor(struct apple_z2 *z2, u16 len)
{
	if (len != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE ||
	    z2->rx_buf[0] != APPLE_Z2_CMD_CTRL_READ_LONG ||
	    z2->rx_buf[1] != APPLE_Z2_REPORT_SURFACE ||
	    z2->rx_buf[2] > (apple_z2_uses_gen1_data(z2) ? 1 : 0) ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, len + 5))
		return -EPROTO;

	return apple_z2_set_surface(z2, z2->rx_buf + 3);
}

/*
 * The iPad 5 suppresses wake commands within its hold-off time (iOS N1
 * wake helper 0xfffffff005ea75e8) and then waits 2 ms.
 */
static int apple_z2_n1_wake_locked(struct apple_z2 *z2)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	int error;

	if (z2->wake_valid &&
	    time_before(jiffies, z2->last_wake +
				 msecs_to_jiffies(n1->wake_holdoff_ms)))
		return 0;

	error = apple_z2_z2_cmd_locked_delay(z2, APPLE_Z2_CMD_WAKE, 0,
					     "n1-wake", false);
	if (error)
		return error;

	z2->last_wake = jiffies;
	z2->wake_valid = true;
	usleep_range(2000, 2100);
	return 0;
}

/*
 * Device information (iOS 0xfffffff005ea2880): E2 is sent twice and the
 * second reply carries the interface version and the maximum packet size.
 */
static int apple_z2_n1_device_info_locked(struct apple_z2 *z2)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	u16 max_packet_size;
	unsigned int i;
	int error = 0;

	for (i = 0; i < n1->device_info_attempts; i++) {
		if (i)
			usleep_range(n1->device_info_retry_us,
				     n1->device_info_retry_us + 1000);

		error = apple_z2_n1_wake_locked(z2);
		if (!error)
			error = apple_z2_z2_cmd_locked_delay(z2,
							     APPLE_Z2_CMD_DEVICE_INFO,
							     0, "n1-e2", false);
		if (!error)
			error = apple_z2_z2_cmd_locked_delay(z2,
							     APPLE_Z2_CMD_DEVICE_INFO,
							     0, "n1-e2-reply",
							     false);
		if (!error &&
		    (z2->rx_buf[0] != APPLE_Z2_CMD_DEVICE_INFO ||
		     !apple_z2_z2_checksum_valid(z2->rx_buf, APPLE_Z2_CMD_SIZE)))
			error = -EBADMSG;
		if (!error)
			break;
	}
	if (error) {
		dev_err(&z2->spidev->dev, "device information failed: %d\n",
			error);
		return error;
	}

	max_packet_size = get_unaligned_le16(z2->rx_buf + 3);
	if (z2->rx_buf[2] != n1->interface || max_packet_size < 16 ||
	    max_packet_size > APPLE_Z2_RX_BUF_SIZE - 5) {
		dev_err(&z2->spidev->dev,
			"unexpected interface %u, max packet %u\n",
			z2->rx_buf[2], max_packet_size);
		return -EPROTO;
	}

	return 0;
}

/*
 * One report read: E3 for the length, then E6 (below 12 bytes) or E7.
 * The second phase sets byte 2 to 1 and keeps the first phase's checksum,
 * at 14 for E6 and in the last two bytes of the length + 5 transfer for
 * E7 (iOS 0xfffffff005ea3110 and 0xfffffff005ea3374). The data starts at
 * rx_buf + 3. -EBADMSG reports a corrupt reply.
 */
static int apple_z2_n1_get_report_once(struct apple_z2 *z2, u8 report,
				       u16 min_len, u16 *len)
{
	unsigned int xfer_len;
	u16 checksum;
	u8 status;
	u8 cmd;
	int error;

	error = apple_z2_n1_wake_locked(z2);
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, report, &status, len,
						 "n1-report");
	if (error)
		return error == -EPROTO ? -EBADMSG : error;
	if (status || *len < min_len || *len > APPLE_Z2_N1_REPORT_MAX) {
		dev_err(&z2->spidev->dev,
			"report %02x: status %u, length %u\n", report, status,
			*len);
		return -EPROTO;
	}

	error = apple_z2_n1_wake_locked(z2);
	if (error)
		return error;

	cmd = *len < 12 ? APPLE_Z2_CMD_CTRL_READ_SHORT :
			  APPLE_Z2_CMD_CTRL_READ_LONG;
	memset(z2->tx_buf, 0, APPLE_Z2_CMD_SIZE);
	memset(z2->rx_buf, 0, APPLE_Z2_CMD_SIZE);
	z2->tx_buf[0] = cmd;
	z2->tx_buf[1] = report;
	put_unaligned_le16(*len, z2->tx_buf + 3);
	error = apple_z2_z2_xfer_locked(z2, "n1-get");
	if (error)
		return error;
	checksum = get_unaligned_le16(z2->tx_buf + 14);

	if (cmd == APPLE_Z2_CMD_CTRL_READ_SHORT) {
		z2->tx_buf[2] = 1;
		error = apple_z2_z2_raw_xfer_locked(z2, "n1-get-data", false);
		xfer_len = APPLE_Z2_CMD_SIZE;
	} else {
		xfer_len = *len + 5;
		memset(z2->rx_buf, 0, xfer_len);
		z2->rx_buf[0] = cmd;
		z2->rx_buf[1] = report;
		z2->rx_buf[2] = 1;
		put_unaligned_le16(*len, z2->rx_buf + 3);
		put_unaligned_le16(checksum, z2->rx_buf + xfer_len - 2);
		apple_z2_apply_z2_delays(z2);
		error = apple_z2_gen1_exchange(z2, z2->rx_buf, xfer_len);
	}
	if (error)
		return error;

	if (z2->rx_buf[0] != cmd ||
	    !apple_z2_z2_checksum_valid(z2->rx_buf, xfer_len))
		return -EBADMSG;

	return 0;
}

/*
 * iOS delays every report read and retries corrupt replies; one budget
 * covers its nested retries. Other errors end the read.
 */
static int apple_z2_n1_get_report_locked(struct apple_z2 *z2, u8 report,
					 u16 min_len, u16 *len)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	unsigned int i;
	int error = -EBADMSG;

	for (i = 0; i < n1->get_attempts && error == -EBADMSG; i++) {
		if (i)
			usleep_range(1000, 2000);
		usleep_range(n1->get_delay_us, n1->get_delay_us + 1000);
		error = apple_z2_n1_get_report_once(z2, report, min_len, len);
	}
	if (error)
		dev_err(&z2->spidev->dev, "report %02x read failed: %d\n",
			report, error);

	return error;
}

/*
 * Runtime discovery after the firmware starts (iOS cacheProperties,
 * 0xfffffff005e976e0): device information, then the family, sensor,
 * region and region-parameter reports, then the surface descriptor.
 */
static int apple_z2_n1_discover_locked(struct apple_z2 *z2)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	const u8 *data = z2->rx_buf + 3;
	u16 len;
	int error;

	z2->wake_valid = false;
	error = apple_z2_n1_device_info_locked(z2);
	if (error)
		return error;

	error = apple_z2_n1_get_report_locked(z2, APPLE_Z2_N1_REPORT_FAMILY,
					      1, &len);
	if (error)
		return error;
	if (data[0] != n1->family) {
		dev_err(&z2->spidev->dev, "unexpected family %u\n", data[0]);
		return -ENODEV;
	}

	/* Endianness (1: little endian), rows, columns, BE16 version. */
	error = apple_z2_n1_get_report_locked(z2, APPLE_Z2_N1_REPORT_SENSOR,
					      5, &len);
	if (error)
		return error;
	if (data[0] != 1 || data[1] != n1->rows || data[2] != n1->columns) {
		dev_err(&z2->spidev->dev,
			"unexpected sensor: endianness %u, %ux%u\n",
			data[0], data[1], data[2]);
		return -ENODEV;
	}
	dev_dbg(&z2->spidev->dev, "firmware version %#x\n",
		get_unaligned_be16(data + 3));

	error = apple_z2_n1_get_report_locked(z2, APPLE_Z2_N1_REPORT_REGION,
					      8, &len);
	if (error)
		return error;

	error = apple_z2_n1_get_report_locked(z2,
					      APPLE_Z2_N1_REPORT_REGION_PARAMS,
					      6, &len);
	if (error)
		return error;

	error = apple_z2_n1_get_report_locked(z2, APPLE_Z2_REPORT_SURFACE,
					      APPLE_Z2_SURFACE_DESCRIPTOR_SIZE,
					      &len);
	if (error)
		return error;
	if (get_unaligned_le32(data) != n1->surface_width ||
	    get_unaligned_le32(data + 4) != n1->surface_height) {
		dev_err(&z2->spidev->dev, "unexpected surface %ux%u\n",
			get_unaligned_le32(data), get_unaligned_le32(data + 4));
		return -ENODEV;
	}

	return apple_z2_set_surface(z2, data);
}

static int apple_z2_iphone7_plus_init_locked(struct apple_z2 *z2)
{
	static const u8 disable_legacy_reports[] = { 0 };
	u16 surface_len;
	u8 status;
	int error;

	error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_WAKE, 0,
				       "iphone7-plus-wake");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, APPLE_Z2_REPORT_SURFACE,
						 &status, &surface_len,
						 "iphone7-plus-d9");
	if (error)
		return error;
	if (status || surface_len != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, APPLE_Z2_REPORT_SURFACE,
					    surface_len,
					    "iphone7-plus-d9-read");
	if (error)
		return error;
	error = apple_z2_store_surface_descriptor(z2, surface_len);
	if (error)
		return error;

	error = apple_z2_write_report_locked(z2, APPLE_Z2_REPORT_ENABLE,
					     disable_legacy_reports,
					     sizeof(disable_legacy_reports),
					     "iphone7-plus-af");
	if (error)
		return error;

	dev_dbg(&z2->spidev->dev,
		"iPhone 7 Plus report initialization complete\n");

	return 0;
}

static int apple_z2_n61_init_locked(struct apple_z2 *z2)
{
	static const u8 rep9d[] = { 1, 0, 0, 0, 0, 0, 0, 0 };
	static const u8 repbf[] = { 0x9d, 0x81, 0x09, 0 };
	static const u8 repaf[] = { 0 };
	u16 length;
	u8 status;
	int error;

	error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_WAKE, 0, "n61-wake");
	if (error)
		return error;
	error = apple_z2_read_report_info_locked(z2, APPLE_Z2_REPORT_SURFACE,
						 &status, &length, "n61-d9");
	if (error)
		return error;
	if (status || length != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE)
		return -EPROTO;
	error = apple_z2_read_report_locked(z2, APPLE_Z2_REPORT_SURFACE,
					    length, "n61-d9-read");
	if (error)
		return error;
	error = apple_z2_store_surface_descriptor(z2, length);
	if (error)
		return error;
	error = apple_z2_write_report_locked(z2, 0x9d, rep9d, sizeof(rep9d), "n61-9d");
	if (error)
		return error;
	error = apple_z2_write_report_locked(z2, 0xbf, repbf, sizeof(repbf), "n61-bf");
	if (error)
		return error;
	return apple_z2_write_report_locked(z2, 0xaf, repaf, sizeof(repaf), "n61-af");
}

static int apple_z2_j172_init_reads_locked(struct apple_z2 *z2)
{
	u8 status;
	u16 db_len;
	u16 prop_len;
	u16 surface_len;
	unsigned int i;
	int error;

	error = apple_z2_read_report_info_locked(z2, APPLE_Z2_REPORT_SURFACE,
						 &status, &surface_len, "d9");
	if (error)
		return error;
	if (status || surface_len != APPLE_Z2_SURFACE_DESCRIPTOR_SIZE)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, APPLE_Z2_REPORT_SURFACE,
					    surface_len, "d9-read");
	if (error)
		return error;
	error = apple_z2_store_surface_descriptor(z2, surface_len);
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xdb, &status, &db_len,
						 "db");
	if (error)
		return error;
	if (status || db_len != 0x7d)
		return -EPROTO;

	error = apple_z2_read_report_locked(z2, 0xdb, db_len, "db-read-1");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0x73, &status, &prop_len,
						 "73");
	if (error)
		return error;
	/* J71b firmware reports 0x73 as unsupported (status 2, no data). */
	if (apple_z2_is_j71b(z2) && status == 2 && !prop_len) {
		prop_len = 0;
	} else if (status || prop_len != 8) {
		return -EPROTO;
	} else {
		error = apple_z2_read_report_locked(z2, 0x73, prop_len,
						    "73-read-1");
		if (error)
			return error;
	}

	for (i = 2; i <= 3; i++) {
		char db_tag[16];
		char prop_tag[16];

		snprintf(db_tag, sizeof(db_tag), "db-read-%u", i);
		error = apple_z2_read_report_locked(z2, 0xdb, db_len, db_tag);
		if (error)
			return error;

		if (!prop_len)
			continue;
		snprintf(prop_tag, sizeof(prop_tag), "73-read-%u", i);
		error = apple_z2_read_report_locked(z2, 0x73, prop_len,
						    prop_tag);
		if (error)
			return error;
	}

	return 0;
}

static int apple_z2_j172_init_writes_locked(struct apple_z2 *z2)
{
	static const u8 rep9d[] = { 0x01, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00 };
	static const u8 repbf[] = { 0x9d, 0x81, 0x09, 0x00 };
	u8 status;
	u16 len;
	int error;

	error = apple_z2_z2_cmd_locked(z2, APPLE_Z2_CMD_WAKE, 0, "j172-wake");
	if (error)
		return error;
	usleep_range(2200, 2300);

	error = apple_z2_read_report_info_locked(z2, 0x9d, &status, &len,
						 "9d");
	if (error)
		return error;
	/* J71b firmware has no 0x9d; its paired 0xbf mode write is J172-only. */
	if (apple_z2_is_j71b(z2) && status == 2 && !len)
		return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_RUNTIME,
						 APPLE_Z2_STATE_REPORTS_READY);
	if (status || len != sizeof(rep9d))
		return -EPROTO;

	error = apple_z2_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-1");
	if (error)
		return error;
	usleep_range(50500, 50700);
	error = apple_z2_write_report_locked(z2, 0x9d, rep9d,
					     sizeof(rep9d), "j172-9d-2");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xbf, &status, &len,
						 "bf");
	if (error)
		return error;
	if (status || len != sizeof(repbf))
		return -EPROTO;
	error = apple_z2_write_report_locked(z2, 0xbf, repbf,
					     sizeof(repbf), "j172-bf");
	if (error)
		return error;

	error = apple_z2_read_report_info_locked(z2, 0xaf, &status, &len,
						 "af");
	if (error)
		return error;
	if (status != 2 || len)
		return -EPROTO;

	dev_dbg(&z2->spidev->dev, "J172 report initialization complete\n");
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_RUNTIME,
					 APPLE_Z2_STATE_REPORTS_READY);
}

/*
 * Generation-1 controllers raise a level interrupt while reports are
 * queued. Service it by reading (and so acknowledging) up to four reports.
 */
static int apple_z2_gen1_drain(struct apple_z2 *z2)
{
	unsigned int i;
	int error;

	for (i = 0; i < 4; i++) {
		error = apple_z2_read_packet(z2);
		if (error)
			return error;
		if (z2->runtime_queue_empty ||
		    (!z2->runtime_frame_valid && !z2->runtime_control_reply))
			break;
	}
	return 0;
}

static int apple_z2_gen1_set_level_irq(struct apple_z2 *z2)
{
	int error;

	z2->saved_irq_type = irq_get_trigger_type(z2->spidev->irq);
	if (!z2->saved_irq_type)
		return -EINVAL;
	error = irq_set_irq_type(z2->spidev->irq, IRQ_TYPE_LEVEL_LOW);
	if (error)
		return error;
	z2->irq_type_changed = true;
	return 0;
}

static void apple_z2_gen1_restore_irq_type(struct apple_z2 *z2)
{
	int error;

	if (!z2->irq_type_changed)
		return;
	error = irq_set_irq_type(z2->spidev->irq, z2->saved_irq_type);
	if (error)
		dev_warn(&z2->spidev->dev,
			 "failed to restore IRQ type: %d\n", error);
	else
		z2->irq_type_changed = false;
}

/*
 * The interrupt is requested disabled. Track whether it is enabled, so that
 * a boot that failed with it disabled is not disabled a second time.
 */
static void apple_z2_irq_on(struct apple_z2 *z2)
{
	if (z2->irq_on)
		return;
	z2->irq_on = true;
	enable_irq(z2->spidev->irq);
}

static void apple_z2_irq_off(struct apple_z2 *z2)
{
	if (!z2->irq_on)
		return;
	z2->irq_on = false;
	disable_irq(z2->spidev->irq);
}

static void apple_z2_release_contacts(struct apple_z2 *z2)
{
	unsigned int i;

	for (i = 0; i < z2->input_dev->mt->num_slots; i++) {
		input_mt_slot(z2->input_dev, i);
		input_mt_report_slot_inactive(z2->input_dev);
	}
	input_mt_sync_frame(z2->input_dev);
	input_sync(z2->input_dev);
}

/*
 * ATN is the interrupt line read back as a GPIO. Without it, frames are
 * read until the queue is empty.
 */
static int apple_z2_n1_attn(struct apple_z2 *z2, bool queue_empty)
{
	if (!z2->attn_gpio)
		return !queue_empty;

	return gpiod_get_value_cansleep(z2->attn_gpio);
}

/*
 * One iPad 5 frame: 1 when a frame (or a control reply) was read, 0 when
 * the queue is empty and ATN is released. A failed read, or an empty queue
 * while ATN stays asserted, is retried up to three times.
 */
static int apple_z2_n1_frame_locked(struct apple_z2 *z2)
{
	unsigned int i;
	int error = 0;

	for (i = 0; i < APPLE_Z2_N1_FRAME_ATTEMPTS; i++) {
		if (i)
			usleep_range(1000, 2000);

		error = apple_z2_read_packet(z2);
		if (!error) {
			if (z2->runtime_frame_valid || z2->runtime_control_reply)
				return 1;
			if (!z2->runtime_queue_empty) {
				error = -EBADMSG;
				continue;
			}
			error = apple_z2_n1_attn(z2, true);
			if (!error)
				return 0;
			if (error > 0)
				error = -EPROTO;
		}
	}

	return error;
}

/*
 * Read frames while ATN stays asserted, at most eight per call; more, or a
 * failed frame, is an error.
 */
static int apple_z2_n1_service_locked(struct apple_z2 *z2)
{
	unsigned int i;
	int ret;

	for (i = 0; i < APPLE_Z2_N1_IRQ_FRAMES; i++) {
		ret = apple_z2_n1_frame_locked(z2);
		if (ret <= 0)
			return ret;
		ret = apple_z2_n1_attn(z2, false);
		if (ret <= 0)
			return ret;
	}

	return -EOVERFLOW;
}

/*
 * After a runtime error the interrupt stays masked and this work tries to
 * resynchronise: wake and drain, then rediscovery, up to three times a
 * second apart, then a full boot. It never reloads firmware otherwise.
 */
static void apple_z2_n1_rearm_work(struct work_struct *work)
{
	struct apple_z2 *z2 = container_of(to_delayed_work(work),
					   struct apple_z2, rearm_work);
	int error;

	mutex_lock(&z2->io_lock);
	if (z2->stopping || !z2->booted) {
		mutex_unlock(&z2->io_lock);
		return;
	}

	if (z2->rearm_attempts++ < APPLE_Z2_N1_REARM_ATTEMPTS) {
		z2->wake_valid = false;
		error = apple_z2_n1_wake_locked(z2);
		if (!error)
			error = apple_z2_n1_service_locked(z2);
		if (error) {
			apple_z2_release_contacts(z2);
			z2->index_parity = 0;
			error = apple_z2_n1_discover_locked(z2);
			if (!error)
				error = apple_z2_n1_service_locked(z2);
		}
		if (!error) {
			z2->rearm_attempts = 0;
			apple_z2_irq_on(z2);
		} else {
			dev_warn(&z2->spidev->dev,
				 "touch resynchronisation %u failed: %d\n",
				 z2->rearm_attempts, error);
			schedule_delayed_work(&z2->rearm_work,
					      msecs_to_jiffies(APPLE_Z2_N1_REARM_DELAY_MS));
		}
		mutex_unlock(&z2->io_lock);
		return;
	}

	z2->booted = false;
	mutex_unlock(&z2->io_lock);

	dev_warn(&z2->spidev->dev, "restarting the touch controller\n");
	apple_z2_platform_power_off(z2);
	error = apple_z2_j71t_boot(z2);
	if (error)
		dev_err(&z2->spidev->dev, "touch controller restart failed: %d\n",
			error);
}

static void apple_z2_n1_irq_locked(struct apple_z2 *z2)
{
	int error;

	if (z2->stopping || !z2->irq_on)
		return;

	error = apple_z2_n1_service_locked(z2);
	if (!error)
		return;

	dev_warn_ratelimited(&z2->spidev->dev,
			     "touch frame error %d, resynchronising\n", error);
	apple_z2_release_contacts(z2);
	z2->irq_on = false;
	disable_irq_nosync(z2->spidev->irq);
	z2->rearm_attempts = 0;
	schedule_delayed_work(&z2->rearm_work,
			      msecs_to_jiffies(APPLE_Z2_N1_REARM_DELAY_MS));
}

static irqreturn_t apple_z2_irq(int irq, void *data)
{
	struct apple_z2 *z2 = data;
	int error;

	if (unlikely(!z2->booted)) {
		complete(&z2->boot_irq);
	} else if (apple_z2_is_j71t(z2)) {
		mutex_lock(&z2->io_lock);
		apple_z2_n1_irq_locked(z2);
		mutex_unlock(&z2->io_lock);
	} else {
		mutex_lock(&z2->io_lock);
		error = apple_z2_is_gen1(z2) ? apple_z2_gen1_drain(z2) :
						 apple_z2_read_packet(z2);
		mutex_unlock(&z2->io_lock);
		if (error)
			dev_warn_ratelimited(&z2->spidev->dev,
					     "runtime packet read failed: %d\n",
					     error);
	}

	return IRQ_HANDLED;
}

static void apple_z2_set_gpio(struct gpio_desc *gpio, int value)
{
	if (gpio)
		gpiod_set_value_cansleep(gpio, value);
}

static void apple_z2_pulse_gpio_pair(struct gpio_desc *gpio0,
				     struct gpio_desc *gpio1)
{
	apple_z2_set_gpio(gpio0, 1);
	apple_z2_set_gpio(gpio1, 1);
	usleep_range(1000, 2000);
	apple_z2_set_gpio(gpio0, 0);
	apple_z2_set_gpio(gpio1, 0);
	usleep_range(1000, 2000);
}

static int apple_z2_j172_sync_claim_gpios(struct apple_z2 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	if (!z2->display_sync1_gpio) {
		z2->display_sync1_gpio =
			devm_gpiod_get_optional(dev, "display-sync1",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync1_gpio)) {
			error = PTR_ERR(z2->display_sync1_gpio);
			z2->display_sync1_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio) {
		z2->display_sync_gpio =
			devm_gpiod_get_optional(dev, "display-sync",
						GPIOD_OUT_LOW);
		if (IS_ERR(z2->display_sync_gpio)) {
			error = PTR_ERR(z2->display_sync_gpio);
			z2->display_sync_gpio = NULL;
			return error;
		}
	}

	if (!z2->display_sync_gpio || !z2->display_sync1_gpio)
		return -ENODEV;

	return 0;
}

static int apple_z2_j172_sync_set_inactive(struct apple_z2 *z2)
{
	int error;

	if (z2->j172_sync_pinctrl_active) {
		devm_pinctrl_put(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		z2->j172_sync_active_state = NULL;
		z2->j172_sync_pinctrl_active = false;
	}

	error = apple_z2_j172_sync_claim_gpios(z2);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync1_gpio);
	if (error)
		return error;

	error = gpiod_direction_input(z2->display_sync_gpio);
	if (error)
		return error;

	dev_dbg(&z2->spidev->dev, "display-sync pins inactive\n");
	return 0;
}

static int apple_z2_j172_sync_set_active(struct apple_z2 *z2)
{
	struct device *dev = &z2->spidev->dev;
	int error;

	error = apple_z2_j172_sync_set_inactive(z2);
	if (error)
		return error;

	devm_gpiod_put(dev, z2->display_sync1_gpio);
	z2->display_sync1_gpio = NULL;
	devm_gpiod_put(dev, z2->display_sync_gpio);
	z2->display_sync_gpio = NULL;

	z2->j172_sync_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(z2->j172_sync_pinctrl)) {
		error = PTR_ERR(z2->j172_sync_pinctrl);
		z2->j172_sync_pinctrl = NULL;
		goto reclaim_gpios;
	}

	z2->j172_sync_active_state =
		pinctrl_lookup_state(z2->j172_sync_pinctrl,
				     "display-sync-active");
	if (IS_ERR(z2->j172_sync_active_state)) {
		error = PTR_ERR(z2->j172_sync_active_state);
		z2->j172_sync_active_state = NULL;
		goto put_pinctrl;
	}

	error = pinctrl_select_state(z2->j172_sync_pinctrl,
				     z2->j172_sync_active_state);
	if (error)
		goto put_pinctrl;

	z2->j172_sync_pinctrl_active = true;
	dev_dbg(dev, "display-sync pins active\n");
	return 0;

put_pinctrl:
	devm_pinctrl_put(z2->j172_sync_pinctrl);
	z2->j172_sync_pinctrl = NULL;
	z2->j172_sync_active_state = NULL;
reclaim_gpios:
	if (apple_z2_j172_sync_claim_gpios(z2))
		dev_warn(dev, "failed to reclaim display-sync GPIOs\n");
	return error;
}

static void apple_z2_platform_power_off(struct apple_z2 *z2)
{
	int error;

	if (apple_z2_is_gen1(z2))
		apple_z2_gen1_restore_irq_type(z2);

	if (apple_z2_is_j71t(z2)) {
		gpiod_set_value_cansleep(z2->reset_gpio, 1);
		error = pinctrl_select_state(z2->sync_pinctrl,
					     z2->sync_inactive);
		if (error)
			dev_warn(&z2->spidev->dev,
				 "failed to stop display sync: %d\n", error);
		if (z2->clk_enabled) {
			clk_disable_unprepare(z2->clk);
			z2->clk_enabled = false;
		}
		apple_z2_set_gpio(z2->power_ldo_gpio, 0);
		if (z2->ana_enabled) {
			error = regulator_disable(z2->ana_supply);
			if (error)
				dev_warn(&z2->spidev->dev,
					 "failed to disable analog supply: %d\n",
					 error);
			else
				z2->ana_enabled = false;
		}
		z2->platform_powered = false;
		return;
	}

	if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2))
		gpiod_set_value_cansleep(z2->reset_gpio, 1);

	if (z2->clk_enabled) {
		clk_disable_unprepare(z2->clk);
		z2->clk_enabled = false;
	}
	if (apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2))
		usleep_range(1000, 2000);

	if (apple_z2_is_j71b(z2)) {
		apple_z2_set_gpio(z2->power_ldo_gpio, 0);
		apple_z2_set_gpio(z2->power_ana_gpio, 0);
		z2->platform_powered = false;
		return;
	}

	if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) {
		if (z2->core_enabled) {
			error = regulator_disable(z2->core_supply);
			if (error) {
				dev_warn(&z2->spidev->dev,
					 "failed to disable core supply: %d\n", error);
			} else {
				z2->core_enabled = false;
			}
			usleep_range(1000, 2000);
		}
		if (z2->hv_enabled) {
			error = regulator_disable(z2->hv_supply);
			if (error) {
				dev_warn(&z2->spidev->dev,
					 "failed to disable HV supply: %d\n", error);
			} else {
				z2->hv_enabled = false;
			}
		}
		z2->platform_powered = false;
		return;
	}

	if (apple_z2_uses_ipad_protocol(z2)) {
		error = apple_z2_j172_sync_set_inactive(z2);
		if (error)
			dev_warn(&z2->spidev->dev,
				 "failed to set display-sync inactive: %d\n",
				 error);
	} else {
		apple_z2_set_gpio(z2->display_sync1_gpio, 0);
		apple_z2_set_gpio(z2->display_sync_gpio, 0);
	}
	apple_z2_set_gpio(z2->power_ldo_gpio, 0);
	apple_z2_set_gpio(z2->power_ana_gpio, 0);
	z2->platform_powered = false;
}

static int apple_z2_enable_clock(struct apple_z2 *z2)
{
	int error;

	if (!z2->clk || z2->clk_enabled)
		return 0;

	error = clk_prepare_enable(z2->clk);
	if (error)
		return error;

	z2->clk_enabled = true;
	return 0;
}

static int apple_z2_j172_power_on(struct apple_z2 *z2)
{
	int error;

	apple_z2_platform_power_off(z2);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	msleep(50);

	apple_z2_set_gpio(z2->power_ana_gpio, 1);
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	error = apple_z2_j172_sync_set_active(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	error = apple_z2_enable_clock(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	usleep_range(15000, 16000);
	z2->platform_powered = true;
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_OFF,
					 APPLE_Z2_STATE_POWERED);
}

static int apple_z2_j71b_power_on(struct apple_z2 *z2)
{
	int error;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	apple_z2_platform_power_off(z2);
	msleep(50);

	apple_z2_set_gpio(z2->power_ana_gpio, 1);
	usleep_range(10000, 11000);
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	usleep_range(1000, 2000);

	error = apple_z2_enable_clock(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	usleep_range(15000, 16000);
	z2->platform_powered = true;
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_OFF,
					 APPLE_Z2_STATE_POWERED);
}

/*
 * iPad 5 power-up (iOS 0xfffffff005e985dc, 0xfffffff005ea100c,
 * 0xfffffff005ea0ecc): analog supply and LDO, 5 ms, reference clock,
 * 15 ms, REQ_WAKEUP (19 c1), 10 ms, display sync, the HBPP NOP probe,
 * reset release and 15 ms.
 */
static int apple_z2_j71t_power_on(struct apple_z2 *z2)
{
	int error;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	usleep_range(5000, 6000);

	error = regulator_enable(z2->ana_supply);
	if (error)
		return error;
	z2->ana_enabled = true;
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	usleep_range(5000, 6000);

	error = apple_z2_enable_clock(z2);
	if (error)
		goto err_power_off;
	usleep_range(15000, 16000);

	z2->tx_buf[0] = 0x19;
	z2->tx_buf[1] = 0xc1;
	error = apple_z2_hbpp_xfer(z2, 2);
	if (error)
		goto err_power_off;
	usleep_range(10000, 11000);

	error = pinctrl_select_state(z2->sync_pinctrl, z2->sync_active);
	if (error)
		goto err_power_off;

	error = apple_z2_hbpp_nop(z2);
	if (error)
		goto err_power_off;
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	usleep_range(15000, 16000);

	z2->platform_powered = true;
	return 0;

err_power_off:
	apple_z2_platform_power_off(z2);
	return error;
}

static int apple_z2_n61_power_on(struct apple_z2 *z2)
{
	int error;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	/* Chestnut prepares its shared boost/Sage rails before the touch LDO. */
	error = regulator_enable(z2->hv_supply);
	if (error)
		return error;
	z2->hv_enabled = true;
	error = regulator_enable(z2->core_supply);
	if (error)
		goto err_power_off;
	z2->core_enabled = true;
	/* The N61 enable-CS helper settles for 5 ms before clock startup. */
	usleep_range(5000, 6000);
	error = apple_z2_enable_clock(z2);
	if (error)
		goto err_power_off;
	usleep_range(15000, 16000);
	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_STATE_POWERED;
	return 0;

err_power_off:
	apple_z2_platform_power_off(z2);
	return error;
}

static int apple_z2_iphone7_plus_power_on(struct apple_z2 *z2)
{
	int error;

	gpiod_set_value_cansleep(z2->reset_gpio, 1);

	error = regulator_enable(z2->hv_supply);
	if (error)
		return error;
	z2->hv_enabled = true;
	usleep_range(2000, 5000);

	error = regulator_enable(z2->core_supply);
	if (error)
		goto err_power_off;
	z2->core_enabled = true;
	usleep_range(2000, 5000);

	error = apple_z2_enable_clock(z2);
	if (error)
		goto err_power_off;
	usleep_range(1000, 2000);

	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_STATE_POWERED;
	return 0;

err_power_off:
	apple_z2_platform_power_off(z2);
	return error;
}

static int apple_z2_platform_power_on(struct apple_z2 *z2)
{
	int error;

	if (z2->platform_powered)
		return 0;

	if (apple_z2_is_j71b(z2))
		return apple_z2_j71b_power_on(z2);
	if (apple_z2_is_j71t(z2))
		return apple_z2_j71t_power_on(z2);
	if (apple_z2_uses_ipad_protocol(z2))
		return apple_z2_j172_power_on(z2);
	if (apple_z2_is_iphone7_plus(z2))
		return apple_z2_iphone7_plus_power_on(z2);

	apple_z2_set_gpio(z2->power_ana_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_set_gpio(z2->power_ldo_gpio, 1);
	usleep_range(1000, 2000);
	apple_z2_pulse_gpio_pair(z2->display_sync_gpio,
				 z2->display_sync1_gpio);

	error = apple_z2_enable_clock(z2);
	if (error) {
		apple_z2_platform_power_off(z2);
		return error;
	}

	usleep_range(1000, 2000);
	z2->platform_powered = true;
	z2->protocol_state = APPLE_Z2_STATE_POWERED;
	return 0;
}

static int apple_z2_pulse_reset(struct apple_z2 *z2)
{
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	usleep_range(1000, 2000);
	return 0;
}

/* Build calibration blob, caller is responsible for freeing the blob data. */
static const char *apple_z2_calibration_property(u32 provider)
{
	switch (provider) {
	case APPLE_Z2_CAL_MULTI_TOUCH:
		return CAL_PROP_NAME;
	case APPLE_Z2_CAL_ORB_GAP:
		return ORB_GAP_CAL_PROP_NAME;
	case APPLE_Z2_CAL_ORB_FORCE:
		return ORB_FORCE_CAL_PROP_NAME;
	case APPLE_Z2_CAL_SHAPE_ACCEL:
		return SHAPE_ACCEL_CAL_PROP_NAME;
	case APPLE_Z2_CAL_DISPLAY_OFFSET:
		return DISPLAY_OFFSET_CAL_PROP_NAME;
	case APPLE_Z2_CAL_FIREFLY:
		return FIREFLY_CAL_PROP_NAME;
	case APPLE_Z2_CAL_PRESSURE:
		return PRESSURE_CAL_PROP_NAME;
	case APPLE_Z2_CAL_FIREFLY_P100:
		return FIREFLY_P100_CAL_PROP_NAME;
	case APPLE_Z2_CAL_EXT_MULTI_TOUCH:
		return EXT_CAL_PROP_NAME;
	case APPLE_Z2_CAL_PROX:
		return PROX_CAL_PROP_NAME;
	default:
		return NULL;
	}
}

static const u8 *apple_z2_build_cal_blob(struct apple_z2 *z2,
					 const char *property, u32 address,
					 u32 max_size, size_t *size,
					 bool required)
{
	u8 *cal_data;
	int cal_size;
	size_t padded_size;
	size_t blob_size;
	u32 checksum;
	u16 checksum_hdr;
	int i;
	struct apple_z2_hbpp_blob_hdr *hdr;
	int error;

	if (!device_property_present(&z2->spidev->dev, property))
		return required ? ERR_PTR(-ENOENT) : NULL;

	cal_size = device_property_count_u8(&z2->spidev->dev, property);
	if (cal_size <= 0)
		return ERR_PTR(cal_size ?: -EINVAL);
	if (cal_size > max_size)
		return ERR_PTR(-E2BIG);

	padded_size = round_up((size_t)cal_size, sizeof(__le32));
	blob_size = sizeof(struct apple_z2_hbpp_blob_hdr) + padded_size +
		    sizeof(__le32);
	u8 *blob_data __free(kfree) = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data)
		return ERR_PTR(-ENOMEM);

	hdr = (struct apple_z2_hbpp_blob_hdr *)blob_data;
	hdr->cmd = cpu_to_le16(APPLE_Z2_HBPP_CMD_BLOB);
	hdr->len = cpu_to_le16(padded_size / sizeof(__le32));
	hdr->addr = cpu_to_le32(address);

	checksum_hdr = 0;
	for (i = 2; i < 8; i++)
		checksum_hdr += blob_data[i];
	hdr->checksum = cpu_to_le16(checksum_hdr);

	cal_data = blob_data + sizeof(struct apple_z2_hbpp_blob_hdr);
	error = device_property_read_u8_array(&z2->spidev->dev, property,
					      cal_data, cal_size);
	if (error)
		return ERR_PTR(error);

	checksum = 0;
	for (i = 0; i < cal_size; i++)
		checksum += cal_data[i];
	put_unaligned_le32(checksum, cal_data + padded_size);

	*size = blob_size;
	return no_free_ptr(blob_data);
}

/*
 * Build an HBPP14 data packet loading @len bytes to @address: the 18 e1
 * prefix, the 30 01 data command, the BE16 word count, the address as two
 * BE16 halves (low half first), a BE16 sum of the count and address bytes,
 * the payload with each byte pair swapped and its 32-bit byte sum.
 */
static u8 *apple_z2_build_hbpp_blob(struct apple_z2 *z2, u32 address,
				    const u8 *data, size_t len, size_t *size)
{
	size_t padded_size;
	size_t words;
	size_t blob_size;
	u32 checksum;
	u16 header_checksum;
	u8 *payload;
	unsigned int i;

	padded_size = round_up(len, sizeof(__le32));
	words = padded_size / sizeof(__le32);
	if (!words || words > U16_MAX)
		return ERR_PTR(-E2BIG);
	blob_size = 12 + padded_size + sizeof(__le32);
	u8 *blob_data __free(kfree) = kzalloc(blob_size, GFP_KERNEL);
	if (!blob_data)
		return ERR_PTR(-ENOMEM);

	put_unaligned_be16(0x18e1, blob_data);
	put_unaligned_be16(APPLE_Z2_HBPP_CMD_BLOB, blob_data + 2);
	/*
	 * HBPP 0x103 encodes the full number of four-byte payload words,
	 * HBPP 0x104 and later one less.
	 */
	put_unaligned_be16(z2->hbpp_version >= 0x104 ? words - 1 : words, blob_data + 4);
	put_unaligned_be16(address, blob_data + 6);
	put_unaligned_be16(address >> 16, blob_data + 8);
	header_checksum = 0;
	for (i = 4; i < 10; i++)
		header_checksum += blob_data[i];
	put_unaligned_be16(header_checksum, blob_data + 10);

	payload = blob_data + 12;
	memcpy(payload, data, len);
	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += payload[i];
	for (i = 0; i < padded_size; i += 2)
		swap(payload[i], payload[i + 1]);
	put_unaligned_be16(checksum, payload + padded_size);
	put_unaligned_be16(checksum >> 16, payload + padded_size + 2);

	*size = blob_size;
	return no_free_ptr(blob_data);
}

/* Build the HBPP14 calibration packet used by iPhone 7 Plus Gen2 firmware. */
static const u8 *apple_z2_build_iphone7_cal(struct apple_z2 *z2,
					    const char *property, u32 address,
					    u32 max_size, size_t *size)
{
	int cal_size;
	int error;

	/*
	 * Units without this calibration keep the default the firmware file
	 * loads before the record.
	 */
	if (!device_property_present(&z2->spidev->dev, property)) {
		dev_info(&z2->spidev->dev, "no %s, keeping the firmware default\n", property);
		*size = 0;
		return NULL;
	}

	cal_size = device_property_count_u8(&z2->spidev->dev, property);
	if (cal_size <= 0)
		return ERR_PTR(cal_size ?: -EINVAL);
	if (cal_size > max_size)
		return ERR_PTR(-E2BIG);

	u8 *cal_data __free(kfree) = kmalloc(cal_size, GFP_KERNEL);
	if (!cal_data)
		return ERR_PTR(-ENOMEM);
	error = device_property_read_u8_array(&z2->spidev->dev, property,
					      cal_data, cal_size);
	if (error)
		return ERR_PTR(error);

	return apple_z2_build_hbpp_blob(z2, address, cal_data, cal_size, size);
}

/* N1 controller values are two BE16 halves, low half first. */
static void apple_z2_hbpp_put32(u8 *p, u32 value)
{
	put_unaligned_be16(value, p);
	put_unaligned_be16(value >> 16, p + 2);
}

static u32 apple_z2_hbpp_get32(const u8 *p)
{
	return get_unaligned_be16(p) | (u32)get_unaligned_be16(p + 2) << 16;
}

static u16 apple_z2_hbpp_sum(const u8 *p, size_t len)
{
	u16 sum = 0;

	while (len--)
		sum += *p++;
	return sum;
}

/* Full-duplex transfer of the first @len bytes of tx_buf into rx_buf. */
static int apple_z2_hbpp_xfer(struct apple_z2 *z2, unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = len,
		.bits_per_word = 8,
	};

	memset(z2->rx_buf, 0, len);
	return spi_sync_transfer(z2->spidev, &xfer, 1);
}

/* ATN_ACK (iOS 0xfffffff005ea23c0): 1a a1, the reply is a BE16 status. */
static int apple_z2_hbpp_ack(struct apple_z2 *z2, u16 *reply)
{
	int error;

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	error = apple_z2_hbpp_xfer(z2, 2);
	if (!error)
		*reply = get_unaligned_be16(z2->rx_buf);
	return error;
}

/* The NOP probe: 1a a1, then 18 e1 seven times (iOS 0xfffffff005ea21a8). */
static int apple_z2_hbpp_nop(struct apple_z2 *z2)
{
	unsigned int i;

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	for (i = 2; i < APPLE_Z2_CMD_SIZE; i += 2) {
		z2->tx_buf[i] = 0x18;
		z2->tx_buf[i + 1] = 0xe1;
	}
	return apple_z2_hbpp_xfer(z2, APPLE_Z2_CMD_SIZE);
}

static bool apple_z2_hbpp_word(u16 word)
{
	switch (word) {
	case 0x18e1:
	case 0x1aa1:
	case 0x1f01:
	case 0x4879:
	case 0x4969:
	case 0x4ad1:
	case 0x4bc1:
		return true;
	default:
		return false;
	}
}

/*
 * Register read (iOS 0xfffffff005ea2564): 1c 73, the address and the BE16
 * sum of its bytes, then 1a a1 18 e1 18 e1 18 e1 returns the value at 2.
 */
static int apple_z2_hbpp_read_reg(struct apple_z2 *z2, u32 addr, u32 *value)
{
	static const u8 result[] = {
		0x1a, 0xa1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
	};
	int error;

	z2->tx_buf[0] = 0x1c;
	z2->tx_buf[1] = 0x73;
	apple_z2_hbpp_put32(z2->tx_buf + 2, addr);
	put_unaligned_be16(apple_z2_hbpp_sum(z2->tx_buf + 2, 4), z2->tx_buf + 6);
	error = apple_z2_hbpp_xfer(z2, 8);
	if (error)
		return error;

	memcpy(z2->tx_buf, result, sizeof(result));
	error = apple_z2_hbpp_xfer(z2, sizeof(result));
	if (!error)
		*value = apple_z2_hbpp_get32(z2->rx_buf + 2);
	return error;
}

/*
 * Masked register write (iOS 0xfffffff005ea268c): 1e 33, address, mask,
 * value and the BE16 sum of those 12 bytes; ATN_ACK answers 0x4ad1.
 */
static int apple_z2_hbpp_write_reg(struct apple_z2 *z2, u32 addr, u32 value)
{
	u16 reply;
	int error;

	z2->tx_buf[0] = 0x1e;
	z2->tx_buf[1] = 0x33;
	apple_z2_hbpp_put32(z2->tx_buf + 2, addr);
	apple_z2_hbpp_put32(z2->tx_buf + 6, 0xffffffff);
	apple_z2_hbpp_put32(z2->tx_buf + 10, value);
	put_unaligned_be16(apple_z2_hbpp_sum(z2->tx_buf + 2, 12),
			   z2->tx_buf + 14);
	error = apple_z2_hbpp_xfer(z2, APPLE_Z2_CMD_SIZE);
	if (!error)
		error = apple_z2_hbpp_ack(z2, &reply);
	if (error)
		return error;
	if (reply != APPLE_Z2_HBPP_ACK_WRITE) {
		dev_err(&z2->spidev->dev, "register %#x write: ACK %#06x\n",
			addr, reply);
		return -EPROTO;
	}

	return 0;
}

/*
 * HBPP data packets are sent byte-wise as they are; ATN_ACK answers 0x4bc1.
 * iOS tries each packet up to five times.
 */
static int apple_z2_hbpp_send(struct apple_z2 *z2, const u8 *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = data,
		.len = len,
		.bits_per_word = 8,
	};
	unsigned int i;
	u16 reply = 0;
	int error = 0;

	for (i = 0; i < APPLE_Z2_HBPP_ATTEMPTS; i++) {
		if (i)
			usleep_range(1000, 2000);
		error = spi_sync_transfer(z2->spidev, &xfer, 1);
		if (!error)
			error = apple_z2_hbpp_ack(z2, &reply);
		if (!error && reply == APPLE_Z2_HBPP_ACK_BLOB)
			return 0;
	}

	dev_err(&z2->spidev->dev, "HBPP packet of %zu bytes: error %d, ACK %#06x\n",
		len, error, reply);
	return error ?: -EPROTO;
}

/*
 * The unit's calibration goes to the controller in packets of at most
 * cal_chunk_size bytes (iOS 0xfffffff005ea5f58).
 */
static int apple_z2_n1_send_calibration(struct apple_z2 *z2,
					const char *property, u32 address,
					u32 max_size)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	size_t offset, chunk, size;
	int cal_size;
	int error;

	if (!device_property_present(&z2->spidev->dev, property))
		return 0;

	cal_size = device_property_count_u8(&z2->spidev->dev, property);
	if (cal_size <= 0)
		return cal_size ?: -EINVAL;
	if (cal_size > max_size)
		return -E2BIG;

	u8 *cal_data __free(kfree) = kmalloc(cal_size, GFP_KERNEL);
	if (!cal_data)
		return -ENOMEM;
	error = device_property_read_u8_array(&z2->spidev->dev, property,
					      cal_data, cal_size);
	if (error)
		return error;

	for (offset = 0; offset < cal_size; offset += chunk) {
		chunk = min_t(size_t, cal_size - offset, n1->cal_chunk_size);

		const u8 *blob __free(kfree) =
			apple_z2_build_hbpp_blob(z2, address + offset,
						 cal_data + offset, chunk,
						 &size);
		if (IS_ERR(blob))
			return PTR_ERR(blob);

		error = apple_z2_hbpp_send(z2, blob, size);
		if (error)
			return error;
	}

	return 0;
}

/*
 * HBPP detection (iOS 0xfffffff005ea2348): both BE16 words the NOP probe
 * returns are HBPP status words. The controller version is at 0x10008ffc.
 */
static int apple_z2_n1_detect(struct apple_z2 *z2)
{
	u32 version;
	int error;

	error = apple_z2_hbpp_nop(z2);
	if (error)
		return error;
	if (!apple_z2_hbpp_word(get_unaligned_be16(z2->rx_buf)) ||
	    !apple_z2_hbpp_word(get_unaligned_be16(z2->rx_buf + 2))) {
		dev_err(&z2->spidev->dev, "no HBPP bootloader: %*ph\n", 4,
			z2->rx_buf);
		return -ENODEV;
	}

	error = apple_z2_hbpp_read_reg(z2, APPLE_Z2_N1_VERSION_REG, &version);
	if (error)
		return error;
	if (!version || version == U32_MAX) {
		dev_err(&z2->spidev->dev, "invalid controller version %#x\n",
			version);
		return -ENODEV;
	}

	dev_dbg(&z2->spidev->dev, "N1 controller %#x\n", version);
	return 0;
}

/*
 * After the calibration and the firmware: the register calibration of
 * MTSPIBootloader_N1::performCalibSeq() (iOS 0xfffffff005ea8830), the
 * calibration request 1f 01 with its 65 ms, and the firmware execute
 * command 1d 53 (iOS 0xfffffff005ea5fc8). The bootloader waits 40 ms
 * after it, its caller another 2 ms before discovery.
 */
static int apple_z2_n1_start_firmware(struct apple_z2 *z2)
{
	const struct apple_z2_n1_info *n1 = z2->info->n1;
	u32 version, value;
	unsigned int i;
	u16 reply;
	int error;

	/* iOS reads the version again after the firmware upload. */
	error = apple_z2_hbpp_read_reg(z2, APPLE_Z2_N1_VERSION_REG, &version);
	if (error)
		return error;

	for (i = 0; i < n1->num_calib_regs; i++) {
		value = n1->calib_regs[i].value;
		if (n1->calib_regs[i].addr == APPLE_Z2_N1_CONST_CAL_REG &&
		    version == APPLE_Z2_N1_CONST_CAL_V11A0)
			value = 2;
		error = apple_z2_hbpp_write_reg(z2, n1->calib_regs[i].addr,
						value);
		if (error)
			return error;
	}

	/* SPI_APU_EN, read by iOS for its log only */
	error = apple_z2_hbpp_read_reg(z2, APPLE_Z2_N1_APU_EN_REG, &value);
	if (error)
		return error;
	dev_dbg(&z2->spidev->dev, "controller %#x, SPI_APU_EN %#x\n",
		version, value);

	z2->tx_buf[0] = 0x1f;
	z2->tx_buf[1] = 0x01;
	error = apple_z2_hbpp_xfer(z2, 2);
	if (error)
		return error;
	msleep(65);
	/* iOS checks only that the ACK transfer completes. */
	error = apple_z2_hbpp_ack(z2, &reply);
	if (error)
		return error;
	dev_dbg(&z2->spidev->dev, "calibration status %#06x\n", reply);

	z2->tx_buf[0] = 0x1d;
	z2->tx_buf[1] = 0x53;
	apple_z2_hbpp_put32(z2->tx_buf + 2, n1->execute_addr);
	put_unaligned_le32(n1->execute_arg, z2->tx_buf + 6);
	put_unaligned_be16(apple_z2_hbpp_sum(z2->tx_buf + 2, 8), z2->tx_buf + 10);
	error = apple_z2_hbpp_xfer(z2, 12);
	if (error)
		return error;
	msleep(40);
	usleep_range(2000, 3000);

	return 0;
}

static int apple_z2_n1_finish_boot(struct apple_z2 *z2)
{
	int error;

	error = apple_z2_n1_start_firmware(z2);
	if (error)
		return error;

	/*
	 * Drain what queued up during discovery, then take the falling ATN
	 * edge the DT describes, as iOS does; the interrupt thread reads
	 * frames while ATN stays asserted.
	 */
	mutex_lock(&z2->io_lock);
	z2->index_parity = 0;
	error = apple_z2_n1_discover_locked(z2);
	if (!error) {
		z2->booted = true;
		error = apple_z2_n1_service_locked(z2);
	}
	if (!error) {
		z2->rearm_attempts = 0;
		apple_z2_irq_on(z2);
	}
	mutex_unlock(&z2->io_lock);

	return error;
}

static const u8 *apple_z2_build_n61_cal(struct apple_z2 *z2,
				       const char *property, u32 address,
				       u32 maximum, u32 provider, size_t *size)
{
	struct device *dev = &z2->spidev->dev;
	u8 *cal __free(kfree) = NULL;
	u8 *packet __free(kfree) = NULL;
	int length, error;

	if (!apple_z2_n61_calibration_target(address, maximum, provider))
		return ERR_PTR(-EINVAL);
	if (!device_property_present(dev, property))
		return provider == APPLE_Z2_CAL_MULTI_TOUCH ? ERR_PTR(-ENOENT) : NULL;
	length = device_property_count_u8(dev, property);
	if (length <= 0 || length > maximum)
		return ERR_PTR(-EINVAL);
	cal = kmalloc(length, GFP_KERNEL);
	if (!cal)
		return ERR_PTR(-ENOMEM);
	error = device_property_read_u8_array(dev, property, cal, length);
	if (error)
		return ERR_PTR(error);
	packet = kzalloc(16 + round_up(length, 4), GFP_KERNEL);
	if (!packet)
		return ERR_PTR(-ENOMEM);
	error = apple_z2_n61_calibration_packet(packet, 16 + round_up(length, 4),
					       cal, length, address);
	if (error < 0)
		return ERR_PTR(error);
	*size = error;
	return no_free_ptr(packet);
}

/* Gen1 byte streams keep CS asserted across bounded PIO chunks. */
static int apple_z2_send_n61_blob(struct apple_z2 *z2, const u8 *data,
				  u32 size, bool init)
{
	struct spi_transfer chunks[APPLE_Z2_N61_MAX_CHUNKS] = { };
	struct spi_transfer ack = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = 2,
		.bits_per_word = 8,
		.speed_hz = 500000,
	};
	unsigned int count, i;
	u32 offset = 0;
	int error;

	if (!size || size > APPLE_Z2_N61_MAX_BLOB ||
	    (init && !apple_z2_n61_init_valid(data, size)))
		return -EINVAL;
	count = DIV_ROUND_UP(size, APPLE_Z2_N61_CHUNK);
	for (i = 0; i < count; i++) {
		chunks[i].tx_buf = data + offset;
		chunks[i].len = min_t(u32, size - offset, APPLE_Z2_N61_CHUNK);
		chunks[i].bits_per_word = 8;
		chunks[i].speed_hz = 500000;
		offset += chunks[i].len;
	}
	if (init) {
		memset(z2->rx_buf, 0, size);
		chunks[0].rx_buf = z2->rx_buf;
	}
	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, chunks, count);
	if (error)
		return error;
	if (init)
		return 0;
	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	error = spi_sync_transfer(z2->spidev, &ack, 1);
	if (!error)
		apple_z2_post_ack_delay(z2);
	return error;
}

static int apple_z2_send_firmware_blob(struct apple_z2 *z2, const u8 *data,
				       u32 size, bool init, bool defer_irq_wait)
{
	struct spi_transfer blob_xfer = {
		.tx_buf = data,
		.len = size,
		.bits_per_word = 8,
	};
	struct spi_transfer ack_xfer = {
		.tx_buf = z2->tx_buf,
		.len = 2,
		.bits_per_word = 8,
	};
	bool ready;
	int error;

	if (apple_z2_is_n61(z2))
		return apple_z2_send_n61_blob(z2, data, size, init);

	/*
	 * iPhone 7 Plus firmware stores HBPP byte streams in 8-bit wire order.
	 * SmartIO may still DMA these transfers, but using 16-bit SPI words would
	 * swap every byte pair a second time.
	 */
	if (!init && !apple_z2_is_iphone7_plus(z2) &&
	    size >= z2->bpw16_min_len)
		blob_xfer.bits_per_word = 16;
	if (init && apple_z2_is_j71b(z2))
		blob_xfer.speed_hz = 500000;

	dev_dbg(&z2->spidev->dev, "firmware blob len=%u bpw=%u\n",
		size, blob_xfer.bits_per_word);

	if ((apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) && init) {
		if (size != 4 || data[0] != 0x1a || data[1] != 0xa1 ||
		    data[2] != 0x18 || data[3] != 0xe1)
			return dev_err_probe(&z2->spidev->dev, -EINVAL,
					     "invalid HBPP14 readiness command\n");
		memset(z2->rx_buf, 0, size);
		blob_xfer.rx_buf = z2->rx_buf;
	}
	apple_z2_avoid_short_dma(z2, &blob_xfer);

	z2->tx_buf[0] = 0x1a;
	z2->tx_buf[1] = 0xa1;
	apple_z2_avoid_short_dma(z2, &ack_xfer);
	reinit_completion(&z2->boot_irq);
	error = spi_sync_transfer(z2->spidev, &blob_xfer, 1);
	if (error)
		return error;

	if ((apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) && init) {
		ready = z2->rx_buf[0] == 0x1f && z2->rx_buf[1] == 0x01;
		if (!ready)
			return dev_err_probe(&z2->spidev->dev, -EPROTO,
					     "touch controller not ready\n");
		dev_dbg(&z2->spidev->dev, "HBPP14 ready\n");
	}

	if (!init || !z2->no_init_ack) {
		error = spi_sync_transfer(z2->spidev, &ack_xfer, 1);
		if (error)
			return error;
		apple_z2_post_ack_delay(z2);
	}
	if ((apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) && init) {
		error = apple_z2_advance_protocol(z2, APPLE_Z2_STATE_BOOT_IRQ,
						  APPLE_Z2_STATE_HBPP_READY);
		if (error)
			return error;
	}

	if (defer_irq_wait && z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		apple_z2_irq_on(z2);
	}

	if (!defer_irq_wait)
		wait_for_completion_timeout(&z2->boot_irq,
					    msecs_to_jiffies(20));

	return 0;
}

static int apple_z2_send_firmware_raw_xfer(struct apple_z2 *z2,
					   const u8 *data, u32 size)
{
	struct spi_transfer xfer = { };
	const u8 *logical_tx;
	u32 tx_len, rx_len, flags, xfer_len;
	unsigned int i;
	int error;

	if (size < 3 * sizeof(__le32))
		return -EINVAL;

	tx_len = get_unaligned_le32(data);
	rx_len = get_unaligned_le32(data + sizeof(__le32));
	flags = get_unaligned_le32(data + 2 * sizeof(__le32));
	if (tx_len > APPLE_Z2_RAW_XFER_MAX_SIZE ||
	    rx_len > APPLE_Z2_RAW_XFER_MAX_SIZE ||
	    flags & ~APPLE_Z2_RAW_XFER_RX_BIT_REVERSE ||
	    size != 3 * sizeof(__le32) + tx_len ||
	    (!tx_len && !rx_len))
		return -EINVAL;

	xfer_len = max(tx_len, rx_len);
	if ((tx_len && tx_len != xfer_len) ||
	    (rx_len && rx_len != xfer_len))
		return -EINVAL;

	logical_tx = data + 3 * sizeof(__le32);
	if (tx_len)
		xfer.tx_buf = logical_tx;
	if (rx_len) {
		memset(z2->rx_buf, 0, rx_len);
		xfer.rx_buf = z2->rx_buf;
	}
	xfer.len = xfer_len;
	xfer.bits_per_word = 8;
	apple_z2_avoid_short_dma(z2, &xfer);

	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;
	if (flags & APPLE_Z2_RAW_XFER_RX_BIT_REVERSE) {
		for (i = 0; i < rx_len; i++)
			z2->rx_buf[i] = bitrev8(z2->rx_buf[i]);
	}

	dev_dbg(&z2->spidev->dev,
		"firmware raw xfer tx_len=%u rx_len=%u flags=%#x tx=%*ph rx=%*ph\n",
		 tx_len, rx_len, flags, min(tx_len, 16U), logical_tx,
		 min(rx_len, 16U), z2->rx_buf);

	return 0;
}

static bool apple_z2_wait_firmware_irq(struct apple_z2 *z2, u32 timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);

	if (!timeout)
		timeout = msecs_to_jiffies(1);

	if (!wait_for_completion_timeout(&z2->boot_irq, timeout)) {
		dev_warn(&z2->spidev->dev,
			 "firmware IRQ wait timed out after %u ms\n", timeout_ms);
		return false;
	}

	return true;
}

static int apple_z2_wait_ready_irq(struct apple_z2 *z2, u32 timeout_ms)
{
	if (!apple_z2_wait_firmware_irq(z2, timeout_ms))
		return -ETIMEDOUT;
	if (!apple_z2_uses_ipad_protocol(z2) && !apple_z2_is_iphone7_plus(z2))
		return 0;

	if (apple_z2_uses_ipad_protocol(z2) &&
	    !apple_z2_wait_firmware_irq(z2, timeout_ms))
		return -ETIMEDOUT;

	dev_dbg(&z2->spidev->dev, "firmware ready IRQ%s received\n",
		apple_z2_uses_ipad_protocol(z2) ? "s" : "");
	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_HBPP_READY,
					 APPLE_Z2_STATE_FIRMWARE_READY);
}

static u32 apple_z2_fw_config_word(const u8 *data, unsigned int word)
{
	return get_unaligned_le32(data + word * sizeof(__le32));
}

static int apple_z2_apply_fw_config(struct apple_z2 *z2, const u8 *data,
				    size_t size)
{
	struct spi_device *spi = z2->spidev;
	u32 valid, min_dma, z2_delay, cs_delay, cpha, cpol, boot_timeout;
	u32 mode = spi->mode;
	int error;

	if (size < APPLE_Z2_FW_CONFIG_WORDS * sizeof(__le32))
		return -EINVAL;

	valid = apple_z2_fw_config_word(data, 0);
	min_dma = apple_z2_fw_config_word(data, 1);
	z2_delay = apple_z2_fw_config_word(data, 2);
	cs_delay = apple_z2_fw_config_word(data, 3);
	cpha = apple_z2_fw_config_word(data, 4);
	cpol = apple_z2_fw_config_word(data, 5);
	boot_timeout = apple_z2_fw_config_word(data, 9);
	z2->hbpp_version = (valid & APPLE_Z2_FW_CONFIG_HBPP_VERSION) ?
			   apple_z2_fw_config_word(data, 10) : 0;
	if ((valid & APPLE_Z2_FW_CONFIG_MIN_DMA) && min_dma) {
		if (apple_z2_is_iphone7_plus(z2)) {
			if (min_dma > APPLE_Z2_RX_BUF_SIZE)
				return -EINVAL;
			z2->dma_min_len = min_dma;
		} else if (apple_z2_is_j71b(z2)) {
			/* Shorter 16-bit transfers corrupt J71b uploads. */
			z2->bpw16_min_len = max_t(u32, 17, min_dma);
		} else {
			z2->bpw16_min_len = min_dma;
		}
	}
	/* The iPad 5 control delay comes from its match data. */
	if ((valid & APPLE_Z2_FW_CONFIG_Z2_DELAY) && !z2->info->n1)
		z2->z2_inter_packet_delay_us = z2_delay;
	/* iPhone 7 Plus uses zero to retain the required 1 ms CS delay. */
	if ((valid & APPLE_Z2_FW_CONFIG_CS_DELAY) &&
	    (!apple_z2_is_iphone7_plus(z2) || cs_delay))
		z2->z2_cs_delay_us = cs_delay;
	if ((valid & APPLE_Z2_FW_CONFIG_BOOT_TIMEOUT) && boot_timeout)
		z2->boot_timeout_ms = boot_timeout;

	if (valid & APPLE_Z2_FW_CONFIG_CPHA) {
		if (cpha)
			mode |= SPI_CPHA;
		else
			mode &= ~SPI_CPHA;
	}
	if (valid & APPLE_Z2_FW_CONFIG_CPOL) {
		if (cpol)
			mode |= SPI_CPOL;
		else
			mode &= ~SPI_CPOL;
	}

	if (mode != spi->mode) {
		u32 old_mode = spi->mode;

		spi->mode = mode;
		error = spi_setup(spi);
		if (error) {
			spi->mode = old_mode;
			return error;
		}
	}

	apple_z2_apply_z2_delays(z2);
	dev_dbg(&spi->dev,
		"firmware SPI config: valid=%#x dma-min-len=%u bpw16-min-len=%u z2-delay=%u cs-delay=%u boot-timeout=%u mode=%#x\n",
		 valid, z2->dma_min_len, z2->bpw16_min_len,
		 z2->z2_inter_packet_delay_us,
		 z2->z2_cs_delay_us, z2->boot_timeout_ms, spi->mode);

	return 0;
}

static int apple_z2_upload_firmware(struct apple_z2 *z2)
{
	const struct apple_z2_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	size_t next_idx;
	size_t size;
	u32 load_cmd;
	u32 address;
	u32 timeout_ms;
	bool defer_irq_wait;
	bool init;
	bool n61_touch_calibration = false;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return dev_err_probe(&z2->spidev->dev, error,
				     "unable to load firmware\n");
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return dev_err_probe(&z2->spidev->dev, -EINVAL,
				     "invalid firmware header\n");

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		/*
		 * The N1 bootloader is found by its NOP probe and needs no boot
		 * interrupt; its packets are checked through ATN_ACK, and its
		 * calibration uses the HBPP14 records.
		 */
		if (apple_z2_is_j71t(z2) &&
		    (load_cmd == LOAD_COMMAND_INIT_PAYLOAD ||
		     load_cmd == LOAD_COMMAND_WAIT_IRQ ||
		     load_cmd == LOAD_COMMAND_SEND_CALIBRATION))
			return -EINVAL;

		switch (load_cmd) {
		case LOAD_COMMAND_INIT_PAYLOAD:
		case LOAD_COMMAND_SEND_BLOB: {
			const u8 *blob = fw->data + fw_idx;

			if (size > fw->size - fw_idx)
				return -EINVAL;
			if (apple_z2_is_j71t(z2)) {
				error = apple_z2_hbpp_send(z2, blob, size);
				if (error)
					return error;
				fw_idx += size;
				break;
			}
			init = load_cmd == LOAD_COMMAND_INIT_PAYLOAD;
			next_idx = round_up(fw_idx + size, 4);
			defer_irq_wait = next_idx <= fw->size &&
				fw->size - next_idx >= 2 * sizeof(__le32) &&
				get_unaligned_le32(fw->data + next_idx) ==
					LOAD_COMMAND_WAIT_IRQ;
			error = apple_z2_send_firmware_blob(z2, blob, size, init,
							    defer_irq_wait);
			if (error)
				return error;
			fw_idx += size;
			break;
		}
		case LOAD_COMMAND_RAW_XFER:
			if (apple_z2_is_n61(z2))
				return -EINVAL;
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_send_firmware_raw_xfer(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case LOAD_COMMAND_SET_CONFIG:
			if (apple_z2_is_n61(z2))
				return -EINVAL;
			if (size > fw->size - fw_idx)
				return -EINVAL;
			error = apple_z2_apply_fw_config(z2, fw->data + fw_idx, size);
			if (error)
				return error;
			fw_idx += size;
			break;
		case LOAD_COMMAND_SEND_CALIBRATION: {
			const u8 *data __free(kfree) = NULL;

			if (apple_z2_is_n61(z2))
				return -EINVAL;

			address = size;
			data = apple_z2_build_cal_blob(z2, CAL_PROP_NAME, address,
						       U32_MAX, &size, false);
			if (IS_ERR(data))
				return PTR_ERR(data);
			if (data) {
				error = apple_z2_send_firmware_blob(z2, data, size, false, false);
				if (error)
					return error;
			}
			break;
		}
		case LOAD_COMMAND_SEND_CALIBRATION_V2: {
			const u8 *data __free(kfree) = NULL;
			const char *property;
			u32 max_size;
			u32 provider;

			if (!apple_z2_is_iphone7_plus(z2) &&
			    !apple_z2_uses_ipad_protocol(z2) &&
			    !apple_z2_is_n61(z2) && !apple_z2_is_j71t(z2))
				return -EINVAL;
			if (size != sizeof(struct apple_z2_fw_calibration) ||
			    size > fw->size - fw_idx)
				return -EINVAL;
			address = get_unaligned_le32(fw->data + fw_idx);
			max_size = get_unaligned_le32(fw->data + fw_idx + sizeof(__le32));
			provider = get_unaligned_le32(fw->data + fw_idx +
						      2 * sizeof(__le32));
			property = apple_z2_calibration_property(provider);
			if (!property || !max_size)
				return -EINVAL;
			fw_idx += size;

			if (apple_z2_is_j71t(z2)) {
				error = apple_z2_n1_send_calibration(z2, property,
								     address,
								     max_size);
				if (error)
					return error;
				break;
			}

			if (apple_z2_is_n61(z2)) {
				data = apple_z2_build_n61_cal(z2, property, address,
							      max_size, provider, &size);
				if (!IS_ERR_OR_NULL(data) && provider == APPLE_Z2_CAL_MULTI_TOUCH)
					n61_touch_calibration = true;
			} else {
				data = apple_z2_build_iphone7_cal(z2, property, address,
								  max_size, &size);
			}
			if (IS_ERR(data))
				return PTR_ERR(data);
			if (!data)
				break;
			error = apple_z2_send_firmware_blob(z2, data, size,
							    false, false);
			if (error)
				return error;
			break;
		}
		case LOAD_COMMAND_WAIT_IRQ:
			if (apple_z2_is_n61(z2))
				return -EINVAL;
			if (size != sizeof(__le32) || size > fw->size - fw_idx)
				return -EINVAL;
			timeout_ms = get_unaligned_le32(fw->data + fw_idx);
			fw_idx += size;
			error = apple_z2_wait_ready_irq(z2, timeout_ms);
			if (error)
				return error;
			if (apple_z2_uses_ipad_protocol(z2)) {
				apple_z2_irq_off(z2);
				error = apple_z2_read_device_info_locked(z2);
				/* J71b: stay masked until its first reports are read. */
				if (apple_z2_is_j71b(z2))
					z2->upload_irq_masked = true;
				else
					apple_z2_irq_on(z2);
				if (error)
					return error;
			}
			break;
		default:
			return -EINVAL;
		}

		fw_idx = round_up(fw_idx, 4);
	}

	if (apple_z2_is_j71t(z2))
		return apple_z2_n1_finish_boot(z2);

	if (apple_z2_is_n61(z2)) {
		if (!n61_touch_calibration)
			return -ENOENT;
		msleep(50);
		mutex_lock(&z2->io_lock);
		error = apple_z2_n61_init_locked(z2);
		if (!error) {
			z2->booted = true;
			z2->protocol_state = APPLE_Z2_STATE_REPORTS_READY;
		}
		mutex_unlock(&z2->io_lock);
		if (!error && z2->upload_irq_masked) {
			z2->upload_irq_masked = false;
			enable_irq(z2->spidev->irq);
		}
		return error;
	}

	if (apple_z2_is_iphone7_plus(z2)) {
		msleep(50);
		mutex_lock(&z2->io_lock);
		error = apple_z2_iphone7_plus_init_locked(z2);
		if (!error) {
			z2->booted = true;
			error = apple_z2_read_packet(z2);
		}
		if (!error && apple_z2_is_d11(z2) &&
		    (!z2->runtime_frame_valid ||
		     z2->rx_buf[5] != APPLE_Z2_D11_STATUS_REPORT))
			error = -EPROTO;
		if (!error && apple_z2_is_d11(z2))
			error = apple_z2_enable_d11_reports_locked(z2);
		mutex_unlock(&z2->io_lock);
		return error;
	}

	z2->booted = true;
	mutex_lock(&z2->io_lock);
	error = apple_z2_read_packet(z2);
	if (!error && apple_z2_uses_ipad_protocol(z2) && !z2->runtime_frame_valid &&
	    !(apple_z2_is_j71b(z2) && z2->runtime_queue_empty))
		error = -EPROTO;
	if (!error && apple_z2_uses_ipad_protocol(z2))
		error = apple_z2_advance_protocol(z2, APPLE_Z2_STATE_DEVICE_INFO,
						  APPLE_Z2_STATE_RUNTIME);
	if (!error && apple_z2_uses_ipad_protocol(z2))
		error = apple_z2_j172_init_reads_locked(z2);
	if (!error && apple_z2_is_j71b(z2))
		error = apple_z2_gen1_drain(z2);
	if (!error && apple_z2_is_j71b(z2))
		error = apple_z2_gen1_set_level_irq(z2);
	if (!error && apple_z2_is_j71b(z2) && z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		apple_z2_irq_on(z2);
	}
	mutex_unlock(&z2->io_lock);
	if (error)
		return error;

	if (!apple_z2_uses_ipad_protocol(z2))
		return 0;

	msleep(2260);
	mutex_lock(&z2->io_lock);
	error = apple_z2_j172_init_writes_locked(z2);
	mutex_unlock(&z2->io_lock);
	return error;
}

static int apple_z2_apply_initial_config(struct apple_z2 *z2)
{
	const struct apple_z2_fw_hdr *fw_hdr;
	const struct firmware *fw __free(firmware) = NULL;
	size_t fw_idx = sizeof(*fw_hdr);
	u32 load_cmd;
	size_t size;
	int error;

	error = request_firmware(&fw, z2->fw_name, &z2->spidev->dev);
	if (error)
		return error;
	if (fw->size < sizeof(*fw_hdr))
		return -EINVAL;

	fw_hdr = (const struct apple_z2_fw_hdr *)fw->data;
	if (le32_to_cpu(fw_hdr->magic) != APPLE_Z2_FW_MAGIC ||
	    le32_to_cpu(fw_hdr->version) != 1)
		return -EINVAL;

	while (fw_idx < fw->size) {
		if (fw->size - fw_idx < 2 * sizeof(__le32))
			return -EINVAL;

		load_cmd = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);
		size = get_unaligned_le32(fw->data + fw_idx);
		fw_idx += sizeof(__le32);

		if (load_cmd == LOAD_COMMAND_SEND_CALIBRATION)
			continue;
		if (size > fw->size - fw_idx)
			return -EINVAL;
		if (load_cmd == LOAD_COMMAND_SEND_CALIBRATION_V2) {
			fw_idx = round_up(fw_idx + size, 4);
			continue;
		}
		if (load_cmd == LOAD_COMMAND_SET_CONFIG)
			return apple_z2_apply_fw_config(z2,
						fw->data + fw_idx, size);
		if (load_cmd != LOAD_COMMAND_INIT_PAYLOAD &&
		    load_cmd != LOAD_COMMAND_SEND_BLOB &&
		    load_cmd != LOAD_COMMAND_RAW_XFER &&
		    load_cmd != LOAD_COMMAND_WAIT_IRQ)
			return -EINVAL;

		fw_idx = round_up(fw_idx + size, 4);
	}

	return -EINVAL;
}

static int apple_z2_boot_preamble(struct apple_z2 *z2, bool full_duplex)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.len = 4,
		.bits_per_word = 8,
		.speed_hz = apple_z2_is_j71b(z2) ? 500000 : 0,
	};
	int error;

	memset(z2->tx_buf, 0, xfer.len);
	if (full_duplex) {
		memset(z2->rx_buf, 0, xfer.len);
		xfer.rx_buf = z2->rx_buf;
	}

	apple_z2_apply_z2_delays(z2);
	error = spi_sync_transfer(z2->spidev, &xfer, 1);
	if (error)
		return error;

	return apple_z2_advance_protocol(z2, APPLE_Z2_STATE_POWERED,
					 APPLE_Z2_STATE_SPI_CONFIGURED);
}

static int apple_z2_iphone7_plus_post_boot_preamble(struct apple_z2 *z2)
{
	struct spi_transfer xfer = {
		.tx_buf = z2->tx_buf,
		.rx_buf = z2->rx_buf,
		.len = 4,
		.bits_per_word = 8,
	};

	memset(z2->tx_buf, 0, xfer.len);
	memset(z2->rx_buf, 0, xfer.len);
	apple_z2_apply_z2_delays(z2);

	return spi_sync_transfer(z2->spidev, &xfer, 1);
}

static int apple_z2_n61_boot(struct apple_z2 *z2)
{
	static const u8 preamble[] = { 0x19, 0xc1 };
	static const u8 check[] = { 0x1a, 0xa1, 0x18, 0xe1, 0x18, 0xe1,
				    0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
				    0x18, 0xe1, 0x18, 0xe1 };
	unsigned int attempt;
	int error;

	z2->booted = false;
	apple_z2_reset_protocol(z2);
	error = apple_z2_n61_power_on(z2);
	if (error)
		return error;
	/* AppleMultitouchN1SPI exchanges the preamble while reset is held. */
	error = apple_z2_send_n61_blob(z2, preamble, sizeof(preamble), true);
	if (error)
		goto err_power;
	usleep_range(10000, 11000);
	/* N61 reset-deassert preflight clocks one complete CHECK exchange. */
	error = apple_z2_send_n61_blob(z2, check, sizeof(check), true);
	if (error)
		goto err_power;
	gpiod_set_value_cansleep(z2->reset_gpio, 0);
	/* N61 reset-deassert-delay is 15 ms. Readiness is synchronous. */
	usleep_range(15000, 16000);
	for (attempt = 0; attempt < APPLE_Z2_N61_READY_ATTEMPTS; attempt++) {
		error = apple_z2_send_n61_blob(z2, check, sizeof(check), true);
		if (error)
			goto err_power;
		if (apple_z2_n61_rom_ready(z2->rx_buf, sizeof(check)))
			break;
		if (attempt + 1 < APPLE_Z2_N61_READY_ATTEMPTS)
			usleep_range(5000, 6000);
	}
	if (attempt == APPLE_Z2_N61_READY_ATTEMPTS) {
		error = -ETIMEDOUT;
		goto err_power;
	}
	/* IRQF_NO_AUTOEN keeps runtime IRQs masked until reports are ready. */
	z2->upload_irq_masked = true;
	z2->protocol_state = APPLE_Z2_STATE_HBPP_READY;
	error = apple_z2_upload_firmware(z2);
	if (!error)
		return 0;
	z2->upload_irq_masked = false;
err_power:
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	apple_z2_platform_power_off(z2);
	apple_z2_reset_protocol(z2);
	return error;
}

/*
 * The iPad 5 boots without a boot interrupt: power-up, HBPP detection,
 * the firmware file (calibration and firmware packets), then the N1
 * register calibration, execute and runtime discovery.
 */
static int apple_z2_j71t_boot(struct apple_z2 *z2)
{
	int error;

	z2->booted = false;
	z2->wake_valid = false;
	apple_z2_reset_protocol(z2);

	/* The SPI mode and timing must be set before the first transfer. */
	error = apple_z2_apply_initial_config(z2);
	if (error)
		return error;

	error = apple_z2_platform_power_on(z2);
	if (error)
		return error;

	error = apple_z2_n1_detect(z2);
	if (!error)
		error = apple_z2_upload_firmware(z2);
	if (error) {
		z2->booted = false;
		apple_z2_platform_power_off(z2);
		apple_z2_reset_protocol(z2);
	}

	return error;
}

static int apple_z2_boot(struct apple_z2 *z2)
{
	bool irq_enabled = false;
	int error;

	if (apple_z2_is_j71t(z2))
		return apple_z2_j71t_boot(z2);
	if (apple_z2_is_n61(z2))
		return apple_z2_n61_boot(z2);

	z2->booted = false;
	apple_z2_reset_protocol(z2);
	error = apple_z2_platform_power_on(z2);
	if (error)
		return error;

	if (apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) {
		error = apple_z2_apply_initial_config(z2);
		if (error)
			goto err_stop;
	}
	if (apple_z2_uses_ipad_protocol(z2)) {
		error = apple_z2_boot_preamble(z2, false);
		if (error)
			goto err_stop;
	} else if (apple_z2_is_iphone7_plus(z2)) {
		/* iPhone 7 Plus performs a legacy reset and zero preamble first. */
		error = apple_z2_pulse_reset(z2);
		if (error)
			goto err_stop;
		error = apple_z2_boot_preamble(z2, true);
		if (error)
			goto err_stop;

		/* Hold reset while arming the IRQ to discard the first boot edge. */
		gpiod_set_value_cansleep(z2->reset_gpio, 1);
		usleep_range(1000, 2000);
	}

	reinit_completion(&z2->boot_irq);
	apple_z2_irq_on(z2);
	irq_enabled = true;
	if (apple_z2_is_j71b(z2)) {
		/* J71b boots on reset release; a pulse would restart it. */
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		error = 0;
	} else if (apple_z2_uses_ipad_protocol(z2)) {
		error = apple_z2_pulse_reset(z2);
	} else if (apple_z2_is_iphone7_plus(z2)) {
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		usleep_range(1000, 2000);
		error = 0;
	} else {
		gpiod_set_value_cansleep(z2->reset_gpio, 0);
		error = 0;
	}
	if (error)
		goto err_stop;

	if (!wait_for_completion_timeout(&z2->boot_irq,
					 msecs_to_jiffies(z2->boot_timeout_ms))) {
		error = -ETIMEDOUT;
		goto err_stop;
	}
	if (apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) {
		error = apple_z2_advance_protocol(z2,
						  APPLE_Z2_STATE_SPI_CONFIGURED,
						  APPLE_Z2_STATE_BOOT_IRQ);
		if (error)
			goto err_stop;
	}
	if (apple_z2_is_iphone7_plus(z2)) {
		error = apple_z2_iphone7_plus_post_boot_preamble(z2);
		if (error)
			goto err_stop;
	}

	if (apple_z2_uses_ipad_protocol(z2) || apple_z2_is_iphone7_plus(z2)) {
		apple_z2_irq_off(z2);
		z2->upload_irq_masked = true;
	}

	error = apple_z2_upload_firmware(z2);
	if (error)
		goto err_stop;

	return 0;

err_stop:
	z2->booted = false;
	if (z2->upload_irq_masked) {
		z2->upload_irq_masked = false;
		apple_z2_irq_on(z2);
	}
	if (irq_enabled)
		apple_z2_irq_off(z2);
	gpiod_set_value_cansleep(z2->reset_gpio, 1);
	apple_z2_platform_power_off(z2);
	apple_z2_reset_protocol(z2);
	return error;
}

static const struct apple_z2_chip_info apple_z2_j293_info = {
	.name = "MacBookPro17,1 Touch Bar",
	.variant = APPLE_Z2_VARIANT_GENERIC,
};

static const struct apple_z2_chip_info apple_z2_j493_info = {
	.name = "Mac14,7 Touch Bar",
	.variant = APPLE_Z2_VARIANT_GENERIC,
};

static const struct apple_z2_chip_info apple_z2_j71b_info = {
	.name = "iPad7,5 Touchscreen",
	.variant = APPLE_Z2_VARIANT_J71B,
	.cs_setup_us = 1000,
	.cs_inactive_us = 1000,
};

/*
 * iPad 5 (J71t) N1, personality C1F15,1 (iOS AppleMultitouchN1SPI). The
 * register calibration is N1's defaults with the C1F15,1 overrides
 * fll-mval and clk32-clock-enable (iOS 0xfffffff005ea8ab0,
 * 0xfffffff005ea8b00); the descriptor values are this model's.
 */
static const struct apple_z2_hbpp_reg apple_z2_j71t_calib_regs[] = {
	{ 0x10003060, 6089 },	/* fll-mval */
	{ 0x1000305c, 0x20 },
	{ 0x10003058, 6 },
	{ APPLE_Z2_N1_CONST_CAL_REG, 3 },
	{ 0x10003518, 1 },	/* clk32-clock-enable */
};

static const struct apple_z2_n1_info apple_z2_j71t_n1 = {
	.calib_regs = apple_z2_j71t_calib_regs,
	.num_calib_regs = ARRAY_SIZE(apple_z2_j71t_calib_regs),
	.execute_addr = 0x10003400,
	.execute_arg = 0x100,
	.cal_size = 0x400,
	.cal_chunk_size = 0x3f0,
	.control_delay_us = 25,
	.get_delay_us = 2000,
	.wake_holdoff_ms = 100,
	.get_attempts = 5,
	.device_info_attempts = 4,
	.device_info_retry_us = 3000,
	.interface = 1,
	.family = 21,
	.rows = 40,
	.columns = 30,
	.surface_width = 14745,
	.surface_height = 19657,
};

/*
 * iOS configureSPI() programs a zero inter-word delay and 10 us chip-select
 * setup and hold, with no separate inactive time.
 */
static const struct apple_z2_chip_info apple_z2_j71t_info = {
	.name = "iPad6,11 Touchscreen",
	.variant = APPLE_Z2_VARIANT_J71T,
	.cs_setup_us = 10,
	.cs_hold_us = 10,
	.n1 = &apple_z2_j71t_n1,
};

static const struct apple_z2_chip_info apple_z2_j172_info = {
	.name = "iPad7,12 Touchscreen",
	.variant = APPLE_Z2_VARIANT_J172,
};

static const struct apple_z2_chip_info apple_z2_d11_info = {
	.name = "iPhone9,2 Touchscreen",
	.variant = APPLE_Z2_VARIANT_D11,
};

static const struct apple_z2_chip_info apple_z2_d111_info = {
	.name = "iPhone9,4 Touchscreen",
	.variant = APPLE_Z2_VARIANT_D111,
};

static const struct apple_z2_chip_info apple_z2_n61_info = {
	.name = "iPhone 6 Touchscreen",
	.variant = APPLE_Z2_VARIANT_N61,
	.cs_setup_us = 1000,
	.cs_hold_us = 1000,
	.cs_inactive_us = 1000,
};

static int apple_z2_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct apple_z2_chip_info *info;
	struct apple_z2 *z2;
	unsigned int mt_flags = INPUT_MT_DIRECT;
	unsigned int slots;
	int cal_size;
	int error;

	info = spi_get_device_match_data(spi);
	if (!info)
		return -ENODEV;

	z2 = devm_kzalloc(dev, sizeof(*z2), GFP_KERNEL);
	if (!z2)
		return -ENOMEM;

	z2->tx_buf = devm_kzalloc(dev, APPLE_Z2_CMD_SIZE, GFP_KERNEL);
	if (!z2->tx_buf)
		return -ENOMEM;
	z2->rx_buf = devm_kzalloc(dev, APPLE_Z2_RX_BUF_SIZE, GFP_KERNEL);
	if (!z2->rx_buf)
		return -ENOMEM;

	z2->spidev = spi;
	z2->info = info;
	z2->variant = info->variant;
	if (apple_z2_is_n61(z2)) {
		z2->z2_inter_packet_delay_us = 1000;
		z2->z2_cs_delay_us = 1000;
	}
	z2->boot_timeout_ms = 20;
	if (apple_z2_is_iphone7_plus(z2)) {
		z2->z2_inter_packet_delay_us = 1000;
		z2->z2_cs_delay_us = 1000;
		z2->boot_timeout_ms = 500;
	}
	if (apple_z2_is_j71b(z2))
		z2->boot_timeout_ms = 500;
	if (info->n1)
		z2->z2_inter_packet_delay_us = info->n1->control_delay_us;
	z2->bpw16_min_len = 1;
	device_property_read_u32(dev, "apple,z2-bpw16-min-len",
				 &z2->bpw16_min_len);
	if (!z2->bpw16_min_len)
		z2->bpw16_min_len = 1;
	z2->no_init_ack = device_property_read_bool(dev,
						    "apple,z2-no-init-ack");
	init_completion(&z2->boot_irq);
	mutex_init(&z2->io_lock);
	INIT_DELAYED_WORK(&z2->rearm_work, apple_z2_n1_rearm_work);
	spi_set_drvdata(spi, z2);

	z2->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(z2->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->reset_gpio),
				     "unable to get reset GPIO\n");
	if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) {
		z2->hv_supply = devm_regulator_get(dev, apple_z2_is_n61(z2) ? "ana" : "hv");
		if (IS_ERR(z2->hv_supply))
			return dev_err_probe(dev, PTR_ERR(z2->hv_supply),
					     "unable to get HV supply\n");

		z2->core_supply = devm_regulator_get(dev, "core");
		if (IS_ERR(z2->core_supply))
			return dev_err_probe(dev, PTR_ERR(z2->core_supply),
					     "unable to get core supply\n");
	}
	if (apple_z2_is_j71t(z2)) {
		/* Claimed as an input so its pin is configured as one. */
		z2->attn_gpio = devm_gpiod_get_optional(dev, "attn", GPIOD_IN);
		if (IS_ERR(z2->attn_gpio))
			return dev_err_probe(dev, PTR_ERR(z2->attn_gpio),
					     "unable to get ATN GPIO\n");

		z2->ana_supply = devm_regulator_get(dev, "ana");
		if (IS_ERR(z2->ana_supply))
			return dev_err_probe(dev, PTR_ERR(z2->ana_supply),
					     "unable to get analog supply\n");

		z2->sync_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(z2->sync_pinctrl))
			return dev_err_probe(dev, PTR_ERR(z2->sync_pinctrl),
					     "unable to get display-sync pins\n");
		z2->sync_active = pinctrl_lookup_state(z2->sync_pinctrl,
						       "display-sync-active");
		if (IS_ERR(z2->sync_active))
			return dev_err_probe(dev, PTR_ERR(z2->sync_active),
					     "missing display-sync-active state\n");
		z2->sync_inactive = pinctrl_lookup_state(z2->sync_pinctrl,
							 "display-sync-inactive");
		if (IS_ERR(z2->sync_inactive))
			return dev_err_probe(dev, PTR_ERR(z2->sync_inactive),
					     "missing display-sync-inactive state\n");
	}

	z2->power_ana_gpio =
		devm_gpiod_get_optional(dev, "power-ana", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ana_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ana_gpio),
				     "unable to get analog power GPIO\n");
	z2->power_ldo_gpio =
		devm_gpiod_get_optional(dev, "power-ldo", GPIOD_OUT_LOW);
	if (IS_ERR(z2->power_ldo_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->power_ldo_gpio),
				     "unable to get LDO power GPIO\n");
	z2->display_sync_gpio =
		devm_gpiod_get_optional(dev, "display-sync", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync_gpio),
				     "unable to get display-sync GPIO\n");
	z2->display_sync1_gpio =
		devm_gpiod_get_optional(dev, "display-sync1", GPIOD_OUT_LOW);
	if (IS_ERR(z2->display_sync1_gpio))
		return dev_err_probe(dev, PTR_ERR(z2->display_sync1_gpio),
				     "unable to get display-sync1 GPIO\n");

	if (apple_z2_is_n61(z2))
		z2->clk = devm_clk_get(dev, NULL);
	else
		z2->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(z2->clk))
		return dev_err_probe(dev, PTR_ERR(z2->clk),
				     "unable to get clock\n");

	error = devm_request_threaded_irq(dev, spi->irq, NULL, apple_z2_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  "apple-z2-irq", z2);
	if (error)
		return dev_err_probe(dev, error, "unable to request IRQ\n");

	error = device_property_read_string(dev, "firmware-name", &z2->fw_name);
	if (error)
		return dev_err_probe(dev, error, "unable to get firmware name\n");
	if (apple_z2_is_iphone7_plus(z2)) {
		static const struct {
			const char *name;
			unsigned int max_size;
		} calibration_properties[] = {
			{ CAL_PROP_NAME, 0x708 },
			{ ORB_GAP_CAL_PROP_NAME, 0x3e0 },
			{ ORB_FORCE_CAL_PROP_NAME, 0x5c4 },
			{ SHAPE_ACCEL_CAL_PROP_NAME, 0xd0 },
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(calibration_properties); i++) {
			cal_size = device_property_count_u8(dev,
							    calibration_properties[i].name);
			if (cal_size <= 0 ||
			    cal_size > calibration_properties[i].max_size ||
			    (apple_z2_is_d111(z2) &&
			     i == APPLE_Z2_CAL_MULTI_TOUCH &&
			     cal_size != APPLE_Z2_D111_CAL_SIZE))
				return dev_err_probe(dev, -EINVAL,
						     "invalid iPhone 7 Plus calibration %s size %d\n",
						     calibration_properties[i].name, cal_size);
		}
	}

	/* The bootloader copies the unit's calibration from the ADT. */
	if (info->n1) {
		cal_size = device_property_count_u8(dev, CAL_PROP_NAME);
		if (cal_size != info->n1->cal_size)
			return dev_err_probe(dev, -EINVAL,
					     "invalid calibration size %d\n",
					     cal_size);

		u8 *cal __free(kfree) = kmalloc(cal_size, GFP_KERNEL);
		if (!cal)
			return -ENOMEM;
		error = device_property_read_u8_array(dev, CAL_PROP_NAME, cal,
						      cal_size);
		if (error)
			return dev_err_probe(dev, error,
					     "unable to read calibration\n");
		/* An erased or unset record is not a calibration. */
		if (!memchr_inv(cal, 0, cal_size) ||
		    !memchr_inv(cal, 0xff, cal_size))
			return dev_err_probe(dev, -EINVAL,
					     "blank calibration\n");
	}

	z2->input_dev = devm_input_allocate_device(dev);
	if (!z2->input_dev)
		return -ENOMEM;
	z2->input_dev->name = info->name;
	z2->input_dev->phys = "apple_z2";
	z2->input_dev->id.bustype = BUS_SPI;

	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	touchscreen_parse_properties(z2->input_dev, true, &z2->props);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_WIDTH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MAJOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_TOUCH_MINOR, 0, 65535, 0, 0);
	input_set_abs_params(z2->input_dev, ABS_MT_ORIENTATION, -32768, 32767,
			     0, 0);

	if (apple_z2_uses_ipad_protocol(z2)) {
		slots = APPLE_Z2_J172_MAX_CONTACTS;
	} else if (apple_z2_is_j71t(z2)) {
		slots = APPLE_Z2_CLASSIC_MAX_CONTACTS;
		mt_flags |= INPUT_MT_DROP_UNUSED;
	} else if (apple_z2_is_iphone7_plus(z2) || apple_z2_is_n61(z2)) {
		slots = 10;
		if (apple_z2_is_d11(z2))
			mt_flags |= INPUT_MT_DROP_UNUSED;
	} else {
		slots = 256;
	}
	error = input_mt_init_slots(z2->input_dev, slots, mt_flags);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to initialize multitouch slots\n");
	error = input_register_device(z2->input_dev);
	if (error)
		return dev_err_probe(dev, error,
				     "unable to register input device\n");

	usleep_range(5000, 10000);
	return apple_z2_boot(z2);
}

static void apple_z2_stop(struct apple_z2 *z2)
{
	bool irq_on;

	if (apple_z2_is_j71t(z2)) {
		mutex_lock(&z2->io_lock);
		z2->stopping = true;
		mutex_unlock(&z2->io_lock);
		cancel_delayed_work_sync(&z2->rearm_work);
	}

	/* The iPad 5 interrupt thread may mask the interrupt itself. */
	mutex_lock(&z2->io_lock);
	irq_on = z2->irq_on;
	z2->irq_on = false;
	mutex_unlock(&z2->io_lock);
	if (irq_on)
		disable_irq(z2->spidev->irq);
	gpiod_direction_output(z2->reset_gpio, 1);
	z2->booted = false;
	apple_z2_platform_power_off(z2);
	apple_z2_reset_protocol(z2);
}

static void apple_z2_remove(struct spi_device *spi)
{
	apple_z2_stop(spi_get_drvdata(spi));
}

static int apple_z2_suspend(struct device *dev)
{
	apple_z2_stop(spi_get_drvdata(to_spi_device(dev)));

	return 0;
}

static int apple_z2_resume(struct device *dev)
{
	struct apple_z2 *z2 = spi_get_drvdata(to_spi_device(dev));

	z2->stopping = false;
	return apple_z2_boot(z2);
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_z2_pm, apple_z2_suspend, apple_z2_resume);

static const struct of_device_id apple_z2_of_match[] = {
	{ .compatible = "apple,j293-touchbar", .data = &apple_z2_j293_info },
	{ .compatible = "apple,j493-touchbar", .data = &apple_z2_j493_info },
	{ .compatible = "apple,j71b-touchscreen", .data = &apple_z2_j71b_info },
	{ .compatible = "apple,j71t-touchscreen", .data = &apple_z2_j71t_info },
	{ .compatible = "apple,j172-touchscreen", .data = &apple_z2_j172_info },
	{ .compatible = "apple,d11-touchscreen", .data = &apple_z2_d11_info },
	{ .compatible = "apple,d111-touchscreen", .data = &apple_z2_d111_info },
	{ .compatible = "apple,n61-touchscreen", .data = &apple_z2_n61_info },
	{}
};
MODULE_DEVICE_TABLE(of, apple_z2_of_match);

static struct spi_device_id apple_z2_of_id[] = {
	{ .name = "j293-touchbar", .driver_data = (kernel_ulong_t)&apple_z2_j293_info },
	{ .name = "j493-touchbar", .driver_data = (kernel_ulong_t)&apple_z2_j493_info },
	{ .name = "j71b-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_j71b_info },
	{ .name = "j71t-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_j71t_info },
	{ .name = "j172-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_j172_info },
	{ .name = "d11-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_d11_info },
	{ .name = "d111-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_d111_info },
	{ .name = "n61-touchscreen", .driver_data = (kernel_ulong_t)&apple_z2_n61_info },
	{}
};
MODULE_DEVICE_TABLE(spi, apple_z2_of_id);

static struct spi_driver apple_z2_driver = {
	.driver = {
		.name	= "apple-z2",
		.pm	= pm_sleep_ptr(&apple_z2_pm),
		.of_match_table = apple_z2_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = apple_z2_of_id,
	.probe    = apple_z2_probe,
	.remove   = apple_z2_remove,
};

module_spi_driver(apple_z2_driver);

MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/dfrmtfw-*.bin");
MODULE_DESCRIPTION("Apple Z2 touchscreens driver");
