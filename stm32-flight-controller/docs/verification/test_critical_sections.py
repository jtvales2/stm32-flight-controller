#!/usr/bin/env python3
"""Host behavior checks for the open-source PRIMASK changes.

Run from any directory:
    python docs/verification/test_critical_sections.py --cc gcc
    python docs/verification/test_critical_sections.py --repo PATH --cc PATH_TO_GCC

This uses the full real sbus.c and the exact fc_core_baro_feed definition
extracted from the real fc_core.c, replacing target interrupt primitives with
observable mocks. It does NOT verify target CMSIS assembly, a complete Keil
firmware build, interrupt latency, or hardware/flight behavior.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


MOCK_HEADER = r"""
#ifndef TEST_STM32F4XX_H
#define TEST_STM32F4XX_H
#include <stdint.h>
uint32_t test_irq_get(void);
void test_irq_disable(void);
void test_irq_set(uint32_t value);
void test_irq_enable(void);
#define __get_PRIMASK() test_irq_get()
#define __disable_irq() test_irq_disable()
#define __set_PRIMASK(value) test_irq_set(value)
#define __enable_irq() test_irq_enable()
#endif
"""

IRQ_MOCK = r"""
static uint32_t irq_mask, saved_mask;
static unsigned irq_gets, irq_disables, irq_sets, irq_enables;
static void (*before_disable)(void);
static void (*before_restore)(void);

uint32_t test_irq_get(void) {
    ++irq_gets;
    saved_mask = irq_mask;
    return irq_mask;
}
void test_irq_disable(void) {
    ++irq_disables;
    /* Model a producer interrupt between PRIMASK read and masking. */
    if (before_disable && irq_mask == 0u) {
        void (*callback)(void) = before_disable;
        before_disable = NULL;
        callback();
    }
    irq_mask = 1u;
}
void test_irq_set(uint32_t value) {
    assert(irq_mask == 1u);
    assert(value == saved_mask);
    ++irq_sets;
    if (before_restore) before_restore();
    irq_mask = value;
}
void test_irq_enable(void) {
    ++irq_enables;
    irq_mask = 0u;
}
static void irq_begin(uint32_t mask) {
    irq_mask = mask;
    saved_mask = 99u;
    irq_gets = irq_disables = irq_sets = irq_enables = 0u;
    before_disable = before_restore = NULL;
}
static void irq_end(uint32_t mask) {
    assert(irq_mask == mask);
    assert(irq_gets == 1u);
    assert(irq_disables == 1u);
    assert(irq_sets == 1u);
    assert(irq_enables == 0u);
}
"""

SBUS_PREFIX = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sbus.h"
#include "stm32f4xx.h"
"""

SBUS_TESTS = r"""
static const uint16_t channels[16] = {
    0u, 2047u, 172u, 1811u, 512u, 1024u, 77u, 1999u,
    1u, 3u, 7u, 15u, 31u, 63u, 127u, 255u
};
static uint8_t frame[25];
static uint32_t frame_count;
static void make_frame(uint8_t flags, uint8_t tail) {
    memset(frame, 0, sizeof(frame));
    frame[0] = 0x0fu;
    for (unsigned channel = 0; channel < 16u; ++channel) {
        for (unsigned bit = 0; bit < 11u; ++bit) {
            unsigned offset = 11u * channel + bit;
            if ((channels[channel] >> bit) & 1u)
                frame[1u + offset / 8u] |= (uint8_t)(1u << (offset % 8u));
        }
    }
    frame[23] = flags;
    frame[24] = tail;
}
static void check_frame(const sbus_frame_t *out, uint8_t flags) {
    for (unsigned i = 0; i < 16u; ++i) assert(out->ch[i] == channels[i]);
    assert(out->lost_frame == ((flags & 0x04u) ? 1u : 0u));
    assert(out->failsafe == ((flags & 0x08u) ? 1u : 0u));
    assert(out->frame_cnt == frame_count);
}
static void expect_no_frame(uint32_t mask) {
    sbus_frame_t out, unchanged;
    memset(&out, 0xa5, sizeof(out));
    unchanged = out;
    irq_begin(mask);
    assert(sbus_read_latest(&out) == 0);
    irq_end(mask);
    assert(memcmp(&out, &unchanged, sizeof(out)) == 0);
}
static void feed_during_disable(void) {
    sbus_feed_bytes(frame, (uint16_t)sizeof(frame));
    ++frame_count;
}
int main(void) {
    for (uint32_t mask = 0u; mask <= 1u; ++mask) {
        expect_no_frame(mask);
        for (unsigned variant = 0; variant < 2u; ++variant) {
            uint8_t flags = variant ? 0x0cu : 0u;
            make_frame(flags, variant ? 0x04u : 0x00u);
            /* Exercise the real parser's continuation across input chunks. */
            sbus_feed_bytes(frame, 7u);
            sbus_feed_bytes(frame + 7u, 18u);
            ++frame_count;
            sbus_frame_t out;
            memset(&out, 0xa5, sizeof(out));
            irq_begin(mask);
            assert(sbus_read_latest(&out) == 1);
            irq_end(mask);
            check_frame(&out, flags);
            expect_no_frame(mask);
        }
        make_frame(0u, 0xaau);
        sbus_feed_bytes(frame, (uint16_t)sizeof(frame));
        expect_no_frame(mask);
        make_frame(0u, 0x00u);
        frame[0] = 0x00u; /* Missing start marker must not publish a frame. */
        memset(frame + 1u, 0, 24u);
        sbus_feed_bytes(frame, (uint16_t)sizeof(frame));
        expect_no_frame(mask);
    }

    /* A frame arriving at the masking boundary must be checked in-section. */
    make_frame(0x04u, 0x00u);
    sbus_frame_t out;
    irq_begin(0u);
    before_disable = feed_during_disable;
    assert(sbus_read_latest(&out) == 1);
    irq_end(0u);
    check_frame(&out, 0x04u);
    expect_no_frame(0u);
    puts("PASS: actual sbus.c parser/read paths, PRIMASK 0/1, frame boundary race");
    return 0;
}
"""

BARO_PREFIX = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
typedef struct {
    uint8_t valid, stale_pending;
    uint32_t ts_ticks, rx_ms;
    int32_t press_pa, temp_centi;
    float alt_rel_m, vz_mps;
} test_baro_t;
static struct { test_baro_t baro; } s;
"""

BARO_TESTS = r"""
static test_baro_t expected;
static unsigned restore_observations;
static void observe_publish(void) {
    /* The complete expected state must be ready before IRQ restoration. */
    assert(irq_mask == 1u);
    assert(s.baro.valid == expected.valid);
    assert(s.baro.stale_pending == expected.stale_pending);
    assert(s.baro.ts_ticks == expected.ts_ticks);
    assert(s.baro.rx_ms == expected.rx_ms);
    assert(s.baro.press_pa == expected.press_pa);
    assert(s.baro.temp_centi == expected.temp_centi);
    assert(s.baro.alt_rel_m == expected.alt_rel_m);
    assert(s.baro.vz_mps == expected.vz_mps);
    ++restore_observations;
}
int main(void) {
    for (uint32_t mask = 0u; mask <= 1u; ++mask) {
        for (uint8_t valid = 0u; valid <= 2u; ++valid) {
            /* Invalid feed must only clear valid, preserving every other field. */
            s.baro.valid = 1u;
            s.baro.stale_pending = 1u;
            s.baro.ts_ticks = 11u;
            s.baro.rx_ms = 29u;
            s.baro.press_pa = 90123;
            s.baro.temp_centi = 2134;
            s.baro.alt_rel_m = 1.25f;
            s.baro.vz_mps = -0.5f;
            expected = s.baro;
            expected.valid = valid ? 1u : 0u;
            if (valid) {
                expected.ts_ticks = 123456u;
                expected.press_pa = 100123;
                expected.temp_centi = -123;
                expected.alt_rel_m = 3.5f;
                expected.vz_mps = -1.75f;
                expected.stale_pending = 0u;
            }
            restore_observations = 0u;
            irq_begin(mask);
            before_restore = observe_publish;
            fc_core_baro_feed(valid, 123456u, 100123, -123, 3.5f, -1.75f);
            irq_end(mask);
            assert(restore_observations == 1u);
        }
    }
    puts("PASS: exact fc_core_baro_feed valid/invalid paths, PRIMASK 0/1, retained state");
    return 0;
}
"""


def extract_function(source: bytes, name: str) -> bytes:
    """Preserve exact bytes, balancing braces while ignoring C literals/comments."""
    text = source.decode("latin-1")
    match = re.search(r"\bvoid\s+" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", text)
    if not match:
        raise ValueError(f"Could not locate definition of {name}")
    depth = 1
    i = match.end()
    while i < len(text):
        if text.startswith("//", i):
            end = text.find("\n", i + 2)
            i = len(text) if end < 0 else end + 1
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            if end < 0:
                raise ValueError("Unterminated C comment")
            i = end + 2
        elif text[i] in "\"'":
            quote = text[i]
            i += 1
            while i < len(text) and text[i] != quote:
                i += 2 if text[i] == "\\" else 1
            i += 1
        else:
            depth += (text[i] == "{") - (text[i] == "}")
            i += 1
            if depth == 0:
                return source[match.start():i]
    raise ValueError(f"Unterminated definition of {name}")


def check_baro_publish_order(function: bytes) -> None:
    """Supplement runtime observation with a source-order publication check."""
    text = function.decode("latin-1")
    # Remove comments so comment text cannot satisfy the checks.
    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
    publish = re.search(r"s\.baro\.valid\s*=\s*1u?\s*;", text)
    if publish is None:
        raise ValueError("Baro valid-last publication assignment was not found")
    for field in ("ts_ticks", "press_pa", "temp_centi", "alt_rel_m", "vz_mps", "stale_pending"):
        assignment = re.search(r"s\.baro\." + field + r"\s*=", text)
        if assignment is None or assignment.start() >= publish.start():
            raise ValueError(f"Baro {field} must be assigned before valid publication")
    restore = re.search(r"__set_PRIMASK\s*\(", text)
    if restore is None or restore.start() <= publish.start():
        raise ValueError("Baro IRQ restoration must follow valid publication")


def run(command: list[str], working_dir: Path) -> None:
    completed = subprocess.run(command, cwd=working_dir, capture_output=True, text=True,
                               errors="replace", timeout=60)
    if completed.stdout:
        print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n")
    if completed.stderr:
        print(completed.stderr, file=sys.stderr,
              end="" if completed.stderr.endswith("\n") else "\n")
    if completed.returncode:
        raise RuntimeError(f"Command failed ({completed.returncode}): {command[0]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--cc", default="gcc", help="GCC-compatible host C compiler executable")
    parser.add_argument("--temp-parent", type=Path,
                        help="Existing writable directory for temporary build files")
    args = parser.parse_args()
    repo = args.repo.resolve()
    firmware = repo / "firmware" / "stm32"
    sbus = firmware / "Core" / "Src" / "sbus.c"
    core = firmware / "FlightController" / "fc_core" / "fc_core.c"
    if not sbus.is_file() or not core.is_file():
        parser.error(f"Expected source files under {firmware}; use --repo for a different repository")
    compiler = shutil.which(args.cc)
    if not compiler:
        parser.error(f"Host compiler not found: {args.cc}")
    baro_function = extract_function(core.read_bytes(), "fc_core_baro_feed")
    check_baro_publish_order(baro_function)
    with tempfile.TemporaryDirectory(prefix="flight-irq-check-", dir=args.temp_parent) as directory:
        temporary = Path(directory)
        (temporary / "stm32f4xx.h").write_text(MOCK_HEADER, encoding="ascii")
        (temporary / "sbus_harness.c").write_text(SBUS_PREFIX + IRQ_MOCK + SBUS_TESTS,
                                                  encoding="ascii")
        (temporary / "baro_harness.c").write_bytes(
            (BARO_PREFIX + IRQ_MOCK).encode("ascii") + baro_function + BARO_TESTS.encode("ascii"))
        common = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                  "-I", str(temporary), "-I", str(firmware / "Core" / "Inc")]
        for test in ("sbus", "baro"):
            executable = temporary / (test + ".exe")
            command = common + [str(temporary / (test + "_harness.c"))]
            if test == "sbus":
                command.append(str(sbus))
            command += ["-o", str(executable)]
            run(command, temporary)
            run([str(executable)], temporary)
    print("PASS: source check confirms baro payload/stale_pending precede valid-last publication.")
    print("LIMIT: host mocks do not verify target CMSIS assembly, full Keil build, latency, or hardware.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
