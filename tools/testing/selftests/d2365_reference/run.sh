#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
set -eu
cd "$(dirname "$0")"
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM
core=../../../../drivers/power/reset
${CC:-cc} -std=c11 -Wall -Wextra -Werror ${CFLAGS:-} -I../../../../include "$core/apple-d2365-reference-core.c" test.c -o "$out/test"
"$out/test"
${CC:-cc} -std=c11 -Wall -Wextra -Werror ${CFLAGS:-} -I../../../../include "$core/apple-cbtl-idbus.c" test_idbus.c -o "$out/test_idbus"
"$out/test_idbus"
${CC:-cc} -std=c11 -Wall -Wextra -Werror ${CFLAGS:-} -I../../../../include \
	"$core/apple-cbtl-idbus.c" "$core/apple-d2365-source.c" \
	"$core/apple-d2365-reference-core.c" test_source.c -o "$out/test_source"
"$out/test_source"

python3 test_current_api.py
python3 test_usb_stream.py
