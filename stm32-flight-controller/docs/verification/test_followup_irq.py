#!/usr/bin/env python3
"""Host behavior verification of the follow-up PRIMASK remediations.

Usage: python docs/verification/test_followup_irq.py --cc gcc
       python docs/verification/test_followup_irq.py --repo PATH --cc PATH_TO_GCC

Extracts exact definitions from real fc_core.c, imu_bringup.c, and the frontend
SetBias implementation. Only state and interrupt/dependency observation are
stubbed. These host tests do not validate target CMSIS instructions, a complete
Keil build, interrupt latency, initialization lifetime, or hardware/flight safety.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


COMMON = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint32_t irq_mask, irq_saved;
static unsigned irq_gets, irq_disables, irq_restores, irq_enables;
static void (*restore_observer)(void);
static uint32_t mock_get(void) {
    ++irq_gets;
    irq_saved = irq_mask;
    return irq_mask;
}
static void mock_disable(void) {
    ++irq_disables;
    irq_mask = 1u;
}
static void mock_restore(uint32_t mask) {
    assert(irq_mask == 1u);
    assert(mask == irq_saved);
    ++irq_restores;
    if (restore_observer) restore_observer();
    irq_mask = mask;
}
void mock_enable(void) {
    ++irq_enables;
    irq_mask = 0u;
}
#define __get_PRIMASK() mock_get()
#define __disable_irq() mock_disable()
#define __set_PRIMASK(value) mock_restore(value)
#define __enable_irq() mock_enable()
static void irq_begin(uint32_t mask) {
    irq_mask = mask;
    irq_saved = 99u;
    irq_gets = irq_disables = irq_restores = irq_enables = 0u;
    restore_observer = NULL;
}
static void irq_end(uint32_t mask, unsigned sections) {
    assert(irq_mask == mask);
    assert(irq_gets == sections);
    assert(irq_disables == sections);
    assert(irq_restores == sections);
    assert(irq_enables == 0u);
}
"""

ASYNC_STATE = r"""
enum { FC_EVT_ALT_EXIT = 18, FC_ALT_EXIT_BARO_STALE = 3 };
typedef struct {
    uint8_t valid, stale_pending;
    uint32_t ts_ticks, rx_ms;
    int32_t press_pa, temp_centi;
    float alt_rel_m, vz_mps;
} test_baro_t;
typedef struct {
    uint8_t active, was_sw;
    float z_sp_m, thr_mid, thr_hover, i_term, thr_out, enter_t;
} test_alt_t;
typedef struct {
    test_baro_t baro;
    struct { test_alt_t alt; } ctrl;
    struct { uint8_t alt_sw; } rc;
} test_state_t;
static test_state_t s, original;
static uint32_t entry_mask;
static unsigned events;
static void fc_evt_push(int id, int16_t a, int16_t b, int16_t c, int16_t d) {
    assert(id == FC_EVT_ALT_EXIT);
    assert(a == FC_ALT_EXIT_BARO_STALE);
    assert(b == 0 && c == 0 && d == 0);
    /* Event notification remains after the small flag-consumption section. */
    assert(irq_mask == entry_mask);
    assert(irq_restores == 1u);
    assert(s.baro.valid == 0u && s.baro.stale_pending == 0u);
    assert(s.ctrl.alt.active == 0u && s.ctrl.alt.was_sw == s.rc.alt_sw);
    ++events;
}
static void observe_consumed_flag(void) {
    test_state_t expected = original;
    expected.baro.stale_pending = 0u;
    assert(irq_mask == 1u);
    /* Existing critical-section extent is retained: only the flag is consumed. */
    assert(memcmp(&s, &expected, sizeof(s)) == 0);
}
"""

ASYNC_TEST = r"""
int main(void) {
    for (uint32_t mask = 0u; mask <= 1u; ++mask) {
        for (uint8_t stale = 0u; stale <= 1u; ++stale) {
            for (uint8_t active = 0u; active <= 1u; ++active) {
                for (uint8_t sw = 0u; sw <= 1u; ++sw) {
                    memset(&s, 0, sizeof(s));
                    s.baro.valid = 1u;
                    s.baro.stale_pending = stale;
                    s.baro.ts_ticks = 1234u;
                    s.baro.rx_ms = 5678u;
                    s.baro.press_pa = 100100;
                    s.baro.temp_centi = 2211;
                    s.baro.alt_rel_m = 2.5f;
                    s.baro.vz_mps = -1.25f;
                    s.ctrl.alt.active = active;
                    s.ctrl.alt.was_sw = (uint8_t)!sw;
                    s.ctrl.alt.z_sp_m = 3.0f;
                    s.ctrl.alt.thr_mid = 0.4f;
                    s.ctrl.alt.thr_hover = 0.45f;
                    s.ctrl.alt.enter_t = 1.5f;
                    s.ctrl.alt.i_term = 0.125f;
                    s.ctrl.alt.thr_out = 0.6f;
                    s.rc.alt_sw = sw;
                    original = s;
                    test_state_t expected = s;
                    expected.baro.stale_pending = 0u;
                    if (stale) {
                        expected.baro.valid = 0u;
                        expected.baro.ts_ticks = 0u;
                        expected.baro.alt_rel_m = 0.0f;
                        expected.baro.vz_mps = 0.0f;
                        expected.ctrl.alt.active = 0u;
                        expected.ctrl.alt.enter_t = 0.0f;
                        expected.ctrl.alt.i_term = 0.0f;
                        expected.ctrl.alt.thr_out = 0.0f;
                        expected.ctrl.alt.was_sw = sw;
                    }
                    events = 0u;
                    entry_mask = mask;
                    irq_begin(mask);
                    restore_observer = observe_consumed_flag;
                    fc_core_service_async();
                    irq_end(mask, 1u);
                    assert(memcmp(&s, &expected, sizeof(s)) == 0);
                    assert(events == (unsigned)(stale && active));

                    /* Already-consumed stale notification cannot emit twice. */
                    original = s;
                    irq_begin(mask);
                    restore_observer = observe_consumed_flag;
                    fc_core_service_async();
                    irq_end(mask, 1u);
                    assert(memcmp(&s, &original, sizeof(s)) == 0);
                    assert(events == (unsigned)(stale && active));
                }
            }
        }
    }
    puts("PASS: exact fc_core_service_async, PRIMASK 0/1, stale/active/switch paths and events");
    return 0;
}
"""

BIAS_STATE = r"""
typedef struct {
    struct {
        uint32_t sentinel_before;
        float acc_bias_g[3], gyr_bias_dps[3];
        uint32_t sentinel_after;
    } cfg;
} IMU_BMI088_FE;
static IMU_BMI088_FE frontend;
static IMU_BMI088_FE *s_fe;
static float expected_acc[3], expected_gyr[3];
static unsigned setter_calls;
/* Compile the actual frontend dependency too, with an observing public wrapper. */
#define IMU_BMI088_FE_SetBias test_real_fe_set_bias
"""

BIAS_WRAPPER = r"""
#undef IMU_BMI088_FE_SetBias
static void IMU_BMI088_FE_SetBias(IMU_BMI088_FE *fe, const float acc[3], const float gyr[3]) {
    assert(irq_mask == 1u);
    assert(irq_gets == 1u && irq_disables == 1u && irq_restores == 0u);
    assert(fe == &frontend && fe == s_fe);
    assert(memcmp(acc, expected_acc, sizeof(expected_acc)) == 0);
    assert(memcmp(gyr, expected_gyr, sizeof(expected_gyr)) == 0);
    ++setter_calls;
    test_real_fe_set_bias(fe, acc, gyr);
}
static void observe_bias_complete(void) {
    assert(irq_mask == 1u && setter_calls == 1u);
    assert(memcmp(frontend.cfg.acc_bias_g, expected_acc, sizeof(expected_acc)) == 0);
    assert(memcmp(frontend.cfg.gyr_bias_dps, expected_gyr, sizeof(expected_gyr)) == 0);
    assert(frontend.cfg.sentinel_before == 0xabcdef01u);
    assert(frontend.cfg.sentinel_after == 0x12345678u);
}
"""

BIAS_TEST = r"""
static void reset_frontend(void) {
    memset(&frontend, 0, sizeof(frontend));
    frontend.cfg.sentinel_before = 0xabcdef01u;
    frontend.cfg.sentinel_after = 0x12345678u;
    frontend.cfg.acc_bias_g[0] = 0.25f;
    frontend.cfg.acc_bias_g[1] = -0.125f;
    frontend.cfg.acc_bias_g[2] = 0.5f;
    frontend.cfg.gyr_bias_dps[0] = 1.0f;
    frontend.cfg.gyr_bias_dps[1] = 2.0f;
    frontend.cfg.gyr_bias_dps[2] = 3.0f;
    setter_calls = 0u;
}
int main(void) {
    for (uint32_t mask = 0u; mask <= 1u; ++mask) {
        const float replacement[3] = { -3.5f, 0.125f, 17.25f };
        for (unsigned null_case = 0u; null_case < 3u; ++null_case) {
            reset_frontend();
            IMU_BMI088_FE original_fe = frontend;
            s_fe = (null_case == 0u || null_case == 2u) ? NULL : &frontend;
            const float *arg = null_case == 0u ? replacement : NULL;
            irq_begin(mask);
            assert(imu_set_gyro_bias_dps(arg) == -1);
            irq_end(mask, 0u);
            assert(setter_calls == 0u);
            assert(memcmp(&frontend, &original_fe, sizeof(frontend)) == 0);
        }
        reset_frontend();
        s_fe = &frontend;
        memcpy(expected_acc, frontend.cfg.acc_bias_g, sizeof(expected_acc));
        memcpy(expected_gyr, replacement, sizeof(expected_gyr));
        irq_begin(mask);
        restore_observer = observe_bias_complete;
        assert(imu_set_gyro_bias_dps(replacement) == 0);
        irq_end(mask, 1u);
        assert(setter_calls == 1u);
        assert(memcmp(replacement, expected_gyr, sizeof(expected_gyr)) == 0);
        float readback[3] = {0};
        assert(imu_get_gyro_bias_dps(readback) == 0);
        assert(memcmp(readback, expected_gyr, sizeof(expected_gyr)) == 0);
        irq_end(mask, 1u); /* Getter adds no interrupt-mask operations. */

        /* Caller may alias the frontend's current array; local copy stays valid. */
        setter_calls = 0u;
        irq_begin(mask);
        restore_observer = observe_bias_complete;
        assert(imu_set_gyro_bias_dps(frontend.cfg.gyr_bias_dps) == 0);
        irq_end(mask, 1u);
        assert(setter_calls == 1u);

        irq_begin(mask);
        assert(imu_get_gyro_bias_dps(NULL) == -1);
        s_fe = NULL;
        assert(imu_get_gyro_bias_dps(readback) == -1);
        irq_end(mask, 0u);
    }
    puts("PASS: exact bias get/set and frontend SetBias, PRIMASK 0/1, null/normal/alias paths");
    return 0;
}
"""


def extract_function(source: bytes, name: str) -> bytes:
    """Balance C braces while preserving original bytes and comment encoding."""
    text = source.decode("latin-1")
    match = re.search(r"\b(?:void|int)\s+" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", text)
    if match is None:
        raise ValueError(f"Definition not found: {name}")
    depth, i = 1, match.end()
    while i < len(text):
        if text.startswith("//", i):
            end = text.find("\n", i + 2)
            i = len(text) if end < 0 else end + 1
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            if end < 0:
                raise ValueError("Unterminated comment")
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
    raise ValueError(f"Unterminated definition: {name}")


def run(command: list[str], directory: Path) -> None:
    result = subprocess.run(command, cwd=directory, capture_output=True, text=True,
                            errors="replace", timeout=60)
    for text, stream in ((result.stdout, sys.stdout), (result.stderr, sys.stderr)):
        if text:
            print(text, end="" if text.endswith("\n") else "\n", file=stream)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command[0]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--cc", default="gcc", help="GCC-compatible host C compiler executable")
    parser.add_argument("--temp-parent", type=Path, help="Existing writable temporary-build parent")
    args = parser.parse_args()
    firmware = args.repo.resolve() / "firmware" / "stm32"
    files = {
        "core": firmware / "FlightController" / "fc_core" / "fc_core.c",
        "bringup": firmware / "Core" / "Src" / "imu_bringup.c",
        "frontend": firmware / "Core" / "Src" / "imu_bmi088_frontend.c",
    }
    for path in files.values():
        if not path.is_file():
            parser.error(f"Source file missing: {path}; use --repo to select the repository")
    compiler = shutil.which(args.cc)
    if compiler is None:
        parser.error(f"Compiler not found: {args.cc}")
    content = {name: path.read_bytes() for name, path in files.items()}
    ascii_bytes = lambda value: value.encode("ascii")
    async_harness = (ascii_bytes(COMMON + ASYNC_STATE)
                     + extract_function(content["core"], "fc_core_service_async")
                     + ascii_bytes(ASYNC_TEST))
    bias_harness = (ascii_bytes(COMMON + BIAS_STATE)
                    + extract_function(content["frontend"], "IMU_BMI088_FE_SetBias")
                    + ascii_bytes(BIAS_WRAPPER)
                    + extract_function(content["bringup"], "imu_get_gyro_bias_dps")
                    + extract_function(content["bringup"], "imu_set_gyro_bias_dps")
                    + ascii_bytes(BIAS_TEST))
    with tempfile.TemporaryDirectory(prefix="flight-followup-irq-", dir=args.temp_parent) as temp:
        directory = Path(temp)
        for name, harness in (("async", async_harness), ("bias", bias_harness)):
            source = directory / (name + ".c")
            executable = directory / (name + ".exe")
            source.write_bytes(harness)
            run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", str(source),
                 "-o", str(executable)], directory)
            run([str(executable)], directory)
    print("LIMIT: host state/mocks do not verify CMSIS instructions, Keil, latency, initialization, or flight safety.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
