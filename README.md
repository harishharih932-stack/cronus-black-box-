# CHRONOS-X

**Catching the catastrophe before it happens.**
An early-warning and auto-shutdown system for reactor cooling failures, built as a software proof-of-concept, with an ESP32-S3 "black box" for the edge.

> **Status: research prototype.** All results come from a synthetic simulator (seed 7) with *assumed* plant numbers. This is not a certified controller and has not been tested on a real plant.

---

## The idea

A fixed 90 °C alarm rings late, and one bad sensor can trigger a false shutdown.
CHRONOS-X does not wait for the temperature. It watches the **cooling flow and the physics around it**, separates a lying sensor from a failing plant, and picks the safest action.

```
Sensor data -> Kalman -> PCA -> CUSUM -> Bayes -> P(critical) -> CVaR -> ACTION
```

**Decision rule**

```
a* = argmin_a [ E[L] + λ·CVaR0.95(L) + μ·C ]          (λ = 0.5, μ = 1.0)

SHUTDOWN  <=>  temperature rising
               AND >= 2 independent cooling sources abnormal
               AND P(critical within 45 min) >= 0.80
```

The system abstains when it is unsure. The LLM (optional, for explanations only) has **zero decision weight**.

## What is inside

| Part | Idea |
|---|---|
| **Kalman filter** | `v = z − H·x̂`, `NIS = vᵀS⁻¹v`. A huge one-tick jump is a sensor fault, a persistent mismatch is a real fault |
| **PCA** | `T² = Σ tᵢ²/λᵢ`, `SPE = ‖x − PPᵀx‖²`. Learned limits, not fixed thresholds |
| **CUSUM** | `S⁺ = max(0, S⁺ + u − k)`. Ignores one spike, catches persistent drift |
| **Bayes + trust** | `P(H|E) ∝ P(E|H)·P(H)` over normal / sensor_fault / cooling_failure / spoofing. Trust = reliability × signature × timeliness × consistency × plausibility |
| **Monte Carlo + CVaR** | Simulated futures per action, same random seeds for every action |
| **Safety certificate** | PASS / FAIL / UNKNOWN, with counterexamples and abstention |
| **Audit** | Hash-chained log and deterministic replay |

**Training is unsupervised.** Models learn only from normal data (2,880 samples from 8 seeded runs). Fault labels are used for evaluation only, never for training or decisions.

## Repository layout

```
chronos_x_final.py                 PC simulator + Tkinter dashboard + tests + Telegram report
firmware/truck_black_box_v5_1.ino  ESP32-S3 "truck black box" firmware (Arduino IDE, C++)
README.md
```

## 1. PC simulator and dashboard

**Requirements:** Python 3.10+, `numpy scipy matplotlib scikit-learn requests` (Tkinter comes with Python). Runs fine in Thonny.

```bash
python chronos_x_final.py            # dashboard (Thonny: open the file and press F5)
python chronos_x_final.py --test     # headless tests T1-T8
python chronos_x_final.py --serve 8000   # tiny HTTP wrapper
```

**Tabs:** Home (optional 60-second demo, live fault injection), Report, Live Monitor, Fault Injection (12 faults plus ablation switches), Metrics (evaluation suite), Telegram (send the full report), Settings.

**Telegram:** type the bot token and chat id in *Settings* (kept in memory only), then use the *Telegram* tab to send the whole report. Long reports are split into messages under Telegram's 4096-character limit.

## 2. Results (simulator, seed 7)

Lead time is a plain subtraction. The same plant, fault and seed are run twice, once with CHRONOS acting and once with nobody acting:

```
Lead time = t_critical(no action) − t_first_alert
```

| Real pump failure | Value |
|---|---|
| Fault injected | 20.0 min |
| CHRONOS first alert | 22.75 min |
| Auto shutdown | 30.0 min |
| Critical (100 °C) if nobody acts | 52.5 min |
| **Lead time (alert)** | **29.75 min** |
| Safety margin after shutdown | 22.5 min |
| Fixed 90 °C alarm (no-action world) | 43.0 min, lead 9.5 min |

Across the real-failure scenarios in the evaluation suite: mean lead **28.58 min (CHRONOS) vs 9.50 min (fixed alarm)**, false alarms in benign scenarios **0 vs 1**, Brier score 0.07, attack-defense precision 1.00 and recall 1.00 on synthetic attacks. Tests T1-T8 pass.

Numbers depend on the scenario. The Home demo (spike at 20 min, then pump failure plus attacks at 35 min) gives a similar lead of about 28.5 min.

## 3. ESP32-S3 black box

`firmware/truck_black_box_v5_1.ino` is an Arduino IDE sketch for an ESP32-S3 that behaves like a truck-ECU data recorder:

- 150-sensor catalog and scan-tool style menu on an ST7735 display
- Black-box recording to LittleFS (data before and after an incident), fault log that survives power loss
- Telegram alerts, WiFi dashboard, buzzer
- Data input over USB/UART/WiFi from a fake ECU (a laptop or Raspberry Pi Pico sending `D,v0,v1,...` lines)

**Planned / in progress:** running the CHRONOS-X detection and shutdown logic on the ESP32 (Kalman, PCA, CUSUM, Bayes, light Monte Carlo), a shutdown report screen, and a hash-chained incident log. Until that is done, CHRONOS-X results above come from the PC simulator only.

**Before you flash:** open the sketch and set your own `WIFI_SSID`, `WIFI_PASS`, `TG_BOT_TOKEN` and `TG_CHAT_ID`. Never commit real credentials.

## Honest limits

- Validated on our own simulator only; plant parameters are assumptions.
- The shutdown happens after the alert, so the real safety margin (about 22 min) is shorter than the alert lead time (about 29 min).
- Disabling the cross-check makes the system raise a false alert; the safety rule still blocks the shutdown.
- Needs real plant data and independent certification before any deployment.

## License

Add a license of your choice (for example MIT) before making the repository public.
