# Latency & Bandwidth Benchmark

Measured performance of the `Ngseo/hyundai-uiwang-left-flowmatch` FlowMatch
Diffusion Policy, both **in-process (native GPU)** and over the **HTTP server**.

> Reproduce: `conda activate vpi && python scripts/benchmark.py --device cuda --steps 240 --hw 480 640`
> (native). HTTP numbers from a localhost `uvicorn` run + a base64 PNG/JPEG client loop.

---

## Test bench

| | |
|---|---|
| **GPU** | **NVIDIA GeForce RTX 3060 (12 GB)** — driver 590.48.01 |
| CPU | AMD Ryzen 5 5600G (6C/12T) |
| RAM | ~18 GB |
| Stack | torch **2.10.0+cu128**, CUDA 12.8, cuDNN 9.10.02, lerobot 0.5.1 (snu fork), Python 3.11 (conda env `vpi`) |
| Model | FlowMatch diffusion, `num_inference_steps=1`, **chunk `n_action_steps=8`**, `n_obs_steps=2`, `horizon=16` |
| Model VRAM | **~1.4 GB** (of 12 GB — lots of headroom) |
| Control target | **30 Hz → 33.3 ms budget per step** |
| Camera res tested | 480×640 (model's declared shape; it resizes to 240×320 internally) |

---

## 1. Latency — native (in-process, GPU compute only)

The policy keeps an internal action queue: a diffusion **re-plan** runs once every
`n_action_steps` (=8) calls; the other 7 calls just **pop** from the queue. Steps were
labeled by inspecting the queue, not assumed.

| step type | share | mean | median | p95 | p99 | max |
|---|---|---|---|---|---|---|
| **heavy (re-plan)** | 1/8 | 23.6 ms | 23.4 ms | 25.3 ms | 25.7 ms | **25.8 ms** |
| **cheap (queue pop)** | 7/8 | 4.2 ms | 3.9 ms | 5.8 ms | 6.4 ms | 6.6 ms |
| **amortized / control step** | — | **6.65 ms** | | | | |

- **Effective max control rate ≈ 150 Hz** (amortized 6.65 ms/step).
- **Even the worst single re-plan (25.8 ms) fits inside the 33.3 ms 30 Hz budget.** ✅
- The model compute is **not** the bottleneck for 30 Hz.

---

## 2. Latency — HTTP end-to-end (localhost)

Full `/predict` round-trip: encode 2 frames → base64 → JSON → POST localhost → server
decode → inference → response. **Worst-case random-noise images** (see caveat below).

| transport (480×640) | mean | median | p95 | p99 | max |
|---|---|---|---|---|---|
| **PNG** | 24.5 ms | 21.6 ms | **42.2 ms** ⚠️ | 45.3 ms | 49.0 ms |
| **JPEG q90** | 16.3 ms | 13.4 ms | 33.8 ms | 35.5 ms | 66.7 ms |

- HTTP adds **~10–20 ms** over native, dominated by **image encoding + base64 + transfer**.
- With **PNG**, p95 (42 ms) **exceeds** the 30 Hz budget — a heavy re-plan landing on a
  large-payload tick pushes it over. **JPEG roughly halves the payload** and brings the
  median well under budget (p95 borderline).
- These are **localhost** numbers (no network hop). Over a real LAN, add transfer time.

---

## 3. Bandwidth (per `/predict` tick → @30 Hz)

| transport | req size / tick | @30 Hz | notes |
|---|---|---|---|
| raw (reference) | 1.84 MB | 442 Mbps | not actually sent |
| **PNG (480×640)** | 2.46 MB | **591 Mbps** | saturates most of 1 GbE ⚠️ |
| **JPEG q90 (480×640)** | 0.74 MB | **177 Mbps** | comfortable on 1 GbE |
| **PNG (240×320)** | 0.62 MB | 148 Mbps | downscaled before send |
| **JPEG q90 (240×320)** | 0.19 MB | **45 Mbps** | best; model resizes to this anyway |
| **output `action[26]`** | 104 B | 3.1 KB/s | negligible |

> The action response is tiny — **all bandwidth is the two input images**.

### ⚠️ Caveat on the image numbers
The benchmark used **random-noise images**, which are the **worst case** for PNG/JPEG
(nothing to compress). **Real camera frames compress far better** — expect PNG 2–4× smaller
and JPEG smaller still, so real-world bandwidth and encode latency will be **lower** than
the tables above. Treat these as upper bounds.

---

## 4. Conclusions & recommendations

1. **Model compute easily meets 30 Hz** (amortized 6.6 ms, ~150 Hz ceiling, worst re-plan
   25.8 ms < 33.3 ms). VRAM use is ~1.4 GB — the 3060 is comfortable.
2. **The bottleneck for 30 Hz is the HTTP transport, not inference** — specifically image
   encoding + payload size. PNG at full res is risky (p95 42 ms, 591 Mbps).
3. **To hit 30 Hz reliably over HTTP:**
   - **Use JPEG (q85–90), not PNG** → ~3× less data, ~½ latency.
   - **Downscale to 240×320 before sending** (the model resizes to that internally anyway)
     → JPEG drops to ~45 Mbps / ~0.19 MB per tick. Biggest single win.
   - **Co-locate** the server with the robot PC (localhost / same host) to avoid the network hop.
   - For lowest latency, **run the policy in-process** (skip HTTP entirely) — native is ~6.6 ms
     amortized vs ~16–24 ms over HTTP.
4. Because the chunk is 8, an alternative is a **chunk endpoint** (return 8 actions, execute
   over 8 ticks) to cut inference/transport calls 8×; note the model is trained to ingest
   fresh observations each tick (`n_obs_steps=2`), so this trades accuracy for rate.

---

## 5. Raw results

- Native JSON: `/tmp/bench_native.json` (regenerate with `scripts/benchmark.py`)
- HTTP JSON: `/tmp/bench_http.json`
