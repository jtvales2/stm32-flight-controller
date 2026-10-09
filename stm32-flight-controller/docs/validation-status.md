# Validation status

Recorded on **2026-10-08 (Asia/Shanghai)**. The first four preparation stages are sealed; this record adds a separate follow-up without repeating dependency cleanup.

| Check | Result | Evidence / limit |
| --- | --- | --- |
| Original and backup preservation | Passed | All 1,773 desktop-original and backup files still match the original manifest |
| First-stage preparation | Sealed | [Original audit](verification/release-audit.json), [build log](verification/keil-rebuild.log) and patches 001–003 retained |
| Follow-up source delta | Two files | `fc_core.c` and `imu_bringup.c`; [before/after hashes](verification/phase2-source-changes.json) |
| Current firmware inventory | 500 files, 495 match historical bytes; 5 cumulative edited files | [Current audit](verification/phase2-release-audit.json); all 64 source references and 13 include directories resolve |
| Target full rebuild | Passed, 0 errors / 4 warnings | [New build log](verification/phase2-keil-rebuild.log), ARMCC 5.06 update 5 build 528 |
| Async service and gyro bias host checks | Passed with mocks | [New test log](verification/phase2-host-followup.log): PRIMASK 0/1, stale/active/switch, event, null/normal/alias paths |
| Prior SBUS/barometer regression | Passed again with mocks | [Regression log](verification/phase2-host-regression.log) |
| Application interrupt primitive inventory | Reviewed patterns | [Call-context review](concurrency-review.md); conditional restoration and terminal Error_Handler preserved |
| Whole-firmware concurrency correctness | Not established | Pattern review and host mocks do not prove all shared-state protocols |
| Target interrupt state / latency / DMA scheduling | Pending | Check actual board runtime with propellers removed |
| Clock and actual peripheral rates | Nominal configuration verified; runtime timing pending | HSI 16 MHz, PLLM=8, PLLN=168, PLLP=2: nominal SYSCLK 168 MHz; HSE is not selected. See [BUILD.md](../BUILD.md) |
| IMU, SBUS, arming/faults and motor behavior | Preserved; hardware pending | No PID, sample timing, calibration thresholds or output logic changes |
| MS5611 / altitude hold / flight | Not validated for this copy | Actual sensor identity, test setup and results required |
| Four implementation ownership claims | Owner confirmed independently written | [Provenance](source-provenance.md) and [inventory](source-ownership.csv) |
| Remaining ownership / project license | Pending owner choice | [License scope](license-scope.md); third-party terms retained |
| Bilingual homepage and 3 diagrams | Prepared | Source-derived diagrams are explanations, not measurements |
| Real photos / bench waveforms / flight material | Owner will add | [Evidence guide](evidence-guide.md); no synthetic results supplied |
| Repository-only packaging | Prepared with guarded script | [Packaging procedure](PACKAGING.md); archive excludes Original and generated outputs |

The follow-up build reported `Code=55448 RO-data=8184 RW-data=448 ZI-data=9688`, generated HEX, and took 14 seconds. Keil exit 1 accompanies four unused-function warnings: `vofa_send_att_csv`, `fc_motor_tool_write_all`, `fc_motor_tool_write_one`, `fc_motor_tool_combo`. This is a successful build with warnings, not a warning-free build or a flight validation.

Historical encodings and non-ASCII comments were preserved. The follow-up does not convert source encodings or assign meanings to undocumented historical waveforms. Original copies, first-stage manifests, first-stage build logs and patches remain unchanged.
