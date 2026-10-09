# Real project evidence

The diagrams in `media/` explain code paths. They are not bench captures, measured waveforms, or evidence that this preparation copy has flown. The author will add actual photographs, logs, plots and video separately. Do not substitute synthetic waveforms or assign undocumented meanings to historical trace channels.

## Asset organization

These are suggested destinations for future real assets, not files currently supplied by this repository.

| Asset | Suggested destination | Accompanying record |
| --- | --- | --- |
| Board / wiring / assembled aircraft photos | `media/photos/` | Board revision, chip identities, wiring, motor numbering and photograph date |
| Bench waveform exports and plots | `media/bench/` | Original capture plus channel definitions, units, timebase, conditions and exact firmware revision |
| Flight video or genuine video thumbnail | `media/flight/` or an author-provided video link | Firmware revision, hardware configuration, mode, conditions, observed result and limitations |
| IMU timing and fault-injection logs | `docs/evidence/` | Original log, collection procedure, expected behavior, observed behavior and pass/fail criterion |
| Barometer / altitude-hold records | `docs/evidence/` | Verified sensor model, pressure/temperature/altitude units, reference method and test conditions |

Keep large videos outside Git if appropriate, with a stable author-provided link. Add README media links only after the assets exist and their records are ready.

## Minimum record for each result

Record the date, source revision, build tools, board revision, sensor model, power supply, receiver/ESC setup, and relevant configuration values. State whether the test used propellers, a fixture, a restrained aircraft, or actual flight. Remove propellers before initial board and motor checks.

For every trace, identify each channel, its units, scaling, sampling/export rate and shared time reference. Preserve raw data. Distinguish commanded values, estimated values and independently measured values; record filtering and any alignment applied during plotting. A plot without a confirmed channel map cannot support a named performance claim.

Describe the procedure, the expected response, the observed response, the comparison criterion, and its limitations. A screenshot or video supports only what it visibly records. Historical evidence must name the historical firmware revision and cannot silently be relabeled as validation of this preparation copy.

## Priority checks

1. **Target interrupt state:** observe incoming and outgoing PRIMASK on successful, empty/invalid, and applicable early-return paths. Host mocks cover C behavior but do not establish target assembly or interrupt latency.
2. **IMU acquisition:** capture DRDY timestamps, DMA completion/error/timeout behavior, ring watermarks, merge/sweep drops, pairing age and control lag under a stated workload.
3. **Arming and flight faults:** record startup switch handling, pre-arm calibration, latch conditions, stop pulses, deliberate latch clearing and a fresh guarded ARM transition. Show peripheral recovery separately from re-arming.
4. **Barometer:** confirm the actual sensor identity, initialization, pressure/temperature conversion, reference zeroing and sample freshness before any altitude-hold test.
5. **Altitude hold and flight:** document requested versus active modes, entry/exit events and a reference altitude measurement. State which flight behaviors were exercised and which remain untested.

Update [validation-status.md](validation-status.md) only when the matching record exists. A build pass does not establish hardware operation; a bench pass does not establish flight behavior.
