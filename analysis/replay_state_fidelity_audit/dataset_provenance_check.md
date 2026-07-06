# Dataset Provenance Check

## Host repo episode

- path: `/home/bi/visuomotor-policy-inference/examples/sample_episodes/right/episode.json`
- sha256: `9a1d4bb0fb38125b218a48c3142bd6c72eae3d5f19dd713c2e38f25607c84989`
- frames: 364
- fps: 30
- first frame: `000000`, timestamp `0.0`
- last frame: `000363`, timestamp `12.100000381469727`
- first state sha256: `86fbbcf1fda13864c98843e8b77560bbeb4c905d05de02e685d04f06baa218b7`
- first action sha256: `563a97353874e32bce5039d102266861f4b3a17559f035e7ad9f8650b763fcef`
- last state sha256: `bf3a41a080bf54207569bfec4041397ff2d7fc6a4848b1589156ac3b4680134b`
- last action sha256: `9950ed5444c159746c3e23e065953998aad01afd6e75fe946c3d6234508060ec`

## Container replay source check

Command looked for `/tmp/sample_episodes/right/episode.json` in running containers.

```text
No running container currently exposes /tmp/sample_episodes/right/episode.json
```

At analysis time, the container copy was not available from the running containers. Therefore all numeric results in this folder are labeled as **host repo sample episode 기준**.

`run_episode_replay.sh` copies `examples/sample_episodes` into `/tmp/sample_episodes` only when the container path is missing, and runs `/tmp/drive_arm_hand_replay.py` with default `--episode-dir /tmp/sample_episodes/right`. To prove a future replay exactly, capture SHA256 from inside `ros2_teleop_system` immediately before/after replay.
