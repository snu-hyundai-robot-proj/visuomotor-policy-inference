# Episode Control — frontend

A single self-contained web page to run episodes: **HOME / START / STOP / CLEAR FAULT /
E-STOP** buttons + a live status panel. No build step — it talks to ROS2 over **rosbridge**
(websocket) using roslib.js, calling the Episode Manager's services and subscribing to
`/episode/status`.

```
 browser (index.html, roslib.js)
     │  ws://<host>:9090
     ▼
 rosbridge_server  ──►  ROS2: /episode/{home,start,stop,clear_fault} (std_srvs/Trigger)
                                /episode/estop (std_msgs/Bool, publish)
                                /episode/status (std_msgs/String JSON, subscribe)
```

## Run

1. Install + launch rosbridge (once):
   ```bash
   sudo apt install ros-$ROS_DISTRO-rosbridge-suite
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml      # serves ws://0.0.0.0:9090
   ```
2. Start the Episode Manager (and the rest of the stack — policy server, drivers, cameras).
3. Open the page:
   ```bash
   cd frontend && python -m http.server 8080      # then open http://localhost:8080
   ```
   or just open `index.html` directly in a browser.
4. Set the rosbridge URL (default `ws://localhost:9090`) and click **connect** if needed.

## What the buttons do
| button | action | enabled when |
|---|---|---|
| **HOME** | `/episode/home` → go to init state | not FAULT / not already HOMING |
| **START** | `/episode/start` → READY→RUNNING | state = READY |
| **STOP** | `/episode/stop` → end episode | state = RUNNING |
| **CLEAR FAULT** | `/episode/clear_fault` | state = FAULT |
| **E-STOP** | publishes `/episode/estop` (latched toggle) | always |

Buttons auto enable/disable based on the live `state` so you can't, e.g., START unless READY.

## Status panel
Parses the `/episode/status` JSON (schema in [`../EPISODE_SYSTEM.md`](../EPISODE_SYSTEM.md) §6-D):
current `state` (color-coded), episode elapsed time, episodes remaining, policy ON/OFF,
fault reason / last termination, and health chips (front cam, wrist cam, state, server, FT).

## Note
Requires the **Episode Manager** node (provides the services + status topic) and **rosbridge**
to be running. Until the manager exists, the page connects but shows `—` / disconnected services.
