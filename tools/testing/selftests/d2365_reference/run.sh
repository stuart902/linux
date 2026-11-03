#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
set -eu
cd "$(dirname "$0")"
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror ${CFLAGS:-} reference.c test.c -o "$out/test"
"$out/test"
