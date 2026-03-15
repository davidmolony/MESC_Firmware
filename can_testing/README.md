# CAN TESTING

This is probably the best work area for debugging CAN issues. The most important outcome of this work was to make changes to the electrical layer of the brain board. Namely, I've learned

## CAN NODE DESIGN NOTES - TEENSY 4.0 + TCAN332G ##
NOTES: 
- for a teensy, use 3.3 V CAN transceiver w/ standby (TCAN332G)
- VCC = 3.3 V with local 0.1 µF decoupling at device
- TXD pull-up to 3.3 V (10k) → recessive during MCU reset
- STB pull-up to 3.3 V (10k) → default standby; MCU drives LOW after FlexCAN init
- Split termination 2, 60Ω termination only at physical bus ends (reduces noise)
- CAN TVS (SM24CANB-02) at connector; helps with hot plug protection
- controlLoop() is running at ~1 kHz (loop_dt_us ~ 1000), while ESC POSVEL is expected around ~500 Hz and asynchronous.

## Runtime Log Field Definitions
Use this reference for lines like:

```json
{"t":4061843742,"pos_L_raw":3.517418,"pos_R_raw":2.181321,"vel_L_raw":92.476555,"vel_R_raw":-92.770332,"new_pos":1,"new_pos_L":0,"new_pos_R":1,"dt_pos_us":1003,"dt_pos_L_us":1000,"dt_pos_R_us":1005,"loop_dt_us":997,"loop_hz":1003.01,"dt_err_1khz_us":-3,"dt_ok_1khz":1,"exec_us":2,"ovr":0}
{"cmd":"CAN_TXQ_SUM","attempts":1958,"ok":1958,"fail":0,"fail_pct":0.000,"mode":5,"posvel_age_us":6066}
```

### Runtime sample fields

| Field | Meaning |
|---|---|
| `t` | Timestamp in microseconds (`us`) from MCU boot (or test timer base). |
| `pos_L_raw` | Left motor reported position, raw angle value (typically wrapped angle, in radians). |
| `pos_R_raw` | Right motor reported position, raw angle value (typically wrapped angle, in radians). |
| `vel_L_raw` | Left motor reported velocity (typically radians/second). |
| `vel_R_raw` | Right motor reported velocity (typically radians/second). |
| `new_pos` | At least one new position/velocity update arrived this cycle (`1=yes`, `0=no`). |
| `new_pos_L` | New left update arrived this cycle (`1=yes`, `0=no`). |
| `new_pos_R` | New right update arrived this cycle (`1=yes`, `0=no`). |
| `dt_pos_us` | Time since last "any motor" position update, in `us`. |
| `dt_pos_L_us` | Time since last left position update, in `us`. |
| `dt_pos_R_us` | Time since last right position update, in `us`. |
| `loop_dt_us` | Actual control-loop period for this iteration, in `us`. |
| `loop_hz` | Control-loop rate computed from `loop_dt_us` (`Hz`). |
| `dt_err_1khz_us` | Loop period error vs 1000 `us` target (negative=faster, positive=slower). |
| `dt_ok_1khz` | Loop timing within acceptable tolerance (`1=yes`, `0=no`). |
| `exec_us` | Time spent executing loop work itself (excluding wait), in `us`. |
| `ovr` | Overrun flag (`1` if loop missed schedule/deadline, else `0`). |

### CAN TX summary fields

| Field | Meaning |
|---|---|
| `cmd` | Message type label; here `CAN_TXQ_SUM` means CAN transmit-queue summary. |
| `attempts` | Total CAN transmit attempts during the run window. |
| `ok` | Number of successful CAN transmits. |
| `fail` | Number of failed CAN transmits. |
| `fail_pct` | Failure percentage (`fail / attempts * 100`). |
| `mode` | Firmware supervisor/test mode ID active during run (`5` here). |
| `posvel_age_us` | Age (staleness) of most recent position/velocity data at summary time, in `us`. |

### Quick interpretation example

- Loop is healthy (`loop_hz ~1003`, `dt_ok_1khz=1`, `ovr=0`).
- Both motors are updating (`new_pos=1`, one side flagged each cycle as frames arrive).
- CAN TX is perfect (`1958/1958`, `fail_pct=0.000`).

## Teensy Log Interpretation Workflow

ChatGPT has blown my mind several times with developing a balancing robot. For example it has helped interpret math from control theory publications, write code to implement control theory, and it has even interpretted results I get on my ossciloscope. 

Today I was having trouble with the CAN connections of my robot, and I dumped a bunch of debugging information from the serial. I was cutting and pasting into codex, asking it to interpret the results. Then it hit me: 

```Why dont I just have a python program send all the results to a chatGPT API?``` 

I have never used the API, so I gave it a shot. The steps are: 

1. Create an OpenAI account at [platform.openai.com](https://platform.openai.com).
2. Set up billing before using the API.
3. Add a payment method and purchase credits (or enable paid usage) so API calls do not fail with `insufficient_quota`.
4. Generate an API key from the **API Keys** page.
5. Copy the key once and store it securely.
6. Export the key in your terminal:
   ```bash
   export OPENAI_API_KEY="sk-..."
   ```
7. (Optional) Persist it in `~/.bashrc` or `~/.zshrc`, then restart terminal.
8. Install dependencies:
   ```bash
   python3 -m pip install --upgrade openai pyserial
   ```
9. Verify API connectivity:
   ```bash
   python baby_pi_test.py
   ```
10. Connect Teensy and identify the serial port (example: `/dev/ttyACM0`).
11. Run the interpreter:
   ```bash
   ./interpret_balance_log.py --port /dev/cu.usbmodem175859001 --baud 115200 --capture-timeout 12
   ```
12. Press Enter when prompted; script sends `run` to Teensy.
13. Teensy runs and prints runtime JSON lines.
14. Script captures output until `balance exit` (or timeout), then sends logs to the API.
15. Interpretation is printed in the terminal (timing, motor health, CAN health, anomalies, verdict).
16. If API fails:
   - `insufficient_quota`: add credits/update billing.
   - `401/403`: check API key/project permissions.
   - `429`: retry later and check rate/quota limits.

   This is an example of the output that I got, once all the CAN connectivity was working. (Turns out I was using a 5V transceiver at 3.3V -- dont do that):

   ## debugging output ##
It takes the API a solid 60 seconds to send results

   ```
   1) Loop timing health
- loop_dt_us: 994–1002; loop_hz: 998.00–1006.04; dt_err_1khz_us: -6 to +2; dt_ok_1khz: always 1
- exec_us: 2–3; ovr: 0 (no overruns)
- Jitter is low (<0.6%); CPU headroom huge; run length: 1,000,996 us (timed stop)

2) Per-motor data-path health
- new_pos flags vary per loop (examples):
  - t=410917043: new_pos_L=1, new_pos_R=0
  - t=411217038: new_pos_L=0, new_pos_R=1
  - t=411317037 and 411517037: new_pos_L=0, new_pos_R=0
- dt_pos_L_us: 999–1988 µs; dt_pos_R_us: 1000–1711 µs; dt_pos_us combined: 1000–1494 µs
  - Indicates mostly 1 ms encoder updates with occasional 1.7–2.0 ms intervals (alternating between sides)
- vel_L_raw ~ +91.6 to +92.4; vel_R_raw ~ -92.8 to -92.4 (stable, symmetric magnitude)
- No extended stalls (>2 ms) observed; position signals appear continuous (pos_* values advancing normally)

3) CAN transport health
- CAN_TXQ_SUM: attempts=1960, ok=1960, fail=0, fail_pct=0.000
- posvel_age_us=11 (fresh TX data); mode=5
- Throughput ~2 CAN frames per 1 kHz loop with zero errors/backlog

4) Notable anomalies
- Mode string mismatch: “starting test_can_transmit mode” while “Balance mode started” printed; likely benign labeling/print inconsistency
- Occasional 1.7–2.0 ms encoder update intervals (dt_pos_L_us up to 1988 µs; dt_pos_R_us up to 1711 µs) and some loops with no new_pos from either side; no impact evident on loop timing or velocity stability

5) Bottom-line verdict
- Loop timing: solid 1 kHz with minimal jitter and ample CPU margin
- Motor sensing path: healthy; minor asynchronous encoder cadence (1–2 ms) but no meaningful dropouts
- CAN transport: perfect (0% fail), low latency, adequate bandwidth
- Overall: System timing and comms are green; data path is serviceable for balance; run ended cleanly by time limit.
```