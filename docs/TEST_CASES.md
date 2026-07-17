# Test Cases — V2X Intersection Assistant

Executable source: `firmware/test/test_cases.c` (runs in CI via `make -C firmware test`).
"Actual" values are measured outputs from running the real functions on the host.

| # | Function under test | Input | Expected result | Actual result | Status |
|---|---|---|---|---|---|
| TC-1 | `kf_predict()` + `kf_update_speed()` | 50 cycles @ dt=20 ms, speed measurement 10 m/s | converges to **10.0 ± 0.01 m/s** | 9.9992 m/s | ✅ PASS |
| TC-2 | `kf_update_gps()` Mahalanobis gate | filter settled at origin; GPS fix at (1000, 1000), 3σ gate | update **rejected** (returns false); nearby fix (0.1, 0.1) accepted | outlier→false, nearby→true | ✅ PASS |
| TC-3 | `v2x_encode()` → `v2x_decode()` | id=42, t=123456789 µs, px=10.5, py=−3.25, vx=8.0, intent=LEFT | 32 bytes written; all fields **bit-exact** after decode | n=32, id=42, px=10.50, intent=LEFT | ✅ PASS |
| TC-4 | `v2x_decode()` CRC check | valid frame with byte 10 XOR 0xFF | decode returns **false** | false | ✅ PASS |
| TC-5 | `v2x_decode()` enum validation | frame with intent=7 (undefined) and valid CRC | decode returns **false** | false | ✅ PASS |
| TC-6 | `brake_emergency()` + `brake_step()` | ramp_rate 2.0/s; step 100 ms, then 500 ms more | **0.200** pressure (RAMP) → **1.000** (FULL), engage_count=1 | 0.200/RAMP → 1.000/FULL, engage=1 | ✅ PASS |
| TC-7 | `brake_release()` + `brake_step()` | release from FULL, 600 ms of stepping | ramps down to **0.000**, state IDLE | 0.000 / IDLE | ✅ PASS |

Suite result: **7/7 passed** (part of 23 total firmware tests).
