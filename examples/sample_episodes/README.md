# Sample episodes

One full real episode per arm from the Hyundai Uiwang dataset, for testing the
inference server with realistic input instead of synthetic noise.

| side | source episode | frames | duration | size |
|---|---|---|---|---|
| `left/`  | ep#86 | 583 | ~19.4 s @ 30 Hz | ~59 MB |
| `right/` | ep#91 | 364 | ~12.1 s @ 30 Hz | ~36 MB |

(The shortest episode of each side was chosen to keep the repo small while still
being a complete episode.)

## Layout

```
<side>/
  front_rgb/000000.jpg ... 0000NN.jpg   # scene (zivid) camera, 640x480 (= model input size)
  wrist_rgb/000000.jpg ... 0000NN.jpg   # wrist camera, 640x480
  episode.json                          # per-frame state(26) + action(26) + timestamp, plus meta
```

`episode.json`:

```jsonc
{
  "side": "left", "source_episode_index": 86, "robot_type": "HDRB + DG-5F-M-LEFT",
  "fps": 30, "image_size": [480, 640, 3], "state_dim": 26, "action_dim": 26,
  "num_frames": 583, "cameras": ["front_rgb", "wrist_rgb"],
  "frames": [ { "frame": "000000", "timestamp": 0.0, "state": [/*26*/], "action": [/*26*/] }, ... ]
}
```

Images are JPEG at **640x480** — the model's declared input size (it resizes/crops
internally regardless of input resolution).

## Replay against the server

Start the server (`docker compose up`, or `uvicorn app.server:app`), then:

```bash
python examples/replay_episode.py --side left            # feed the episode to /predict
python examples/replay_episode.py --side right --compare # also report MAE vs the recorded action
```

`--compare` prints the mean absolute error between the predicted and the recorded
action. Feeding the matching model its own training episode yields a small error
(≈0.01–0.02), a quick sanity check that serving + preprocessing are wired correctly.

## Regenerate

These were exported from the LeRobot datasets with
`scripts/export_uiwang_sample_episode.py` in the
[lerobot fork](https://github.com/snu-hyundai-robot-proj/lerobot).
