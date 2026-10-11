# AirMouse: NIDAR GPS-denied indoor search drone

An autonomous indoor drone that enters a maze with no GPS, maps it in 2D, finds up to 6 survivors, tags each one's grid cell, and lands back at the exit. A live web Ground Control Station shows the map, the survivor tags, and the camera feed throughout.

| Layer | Choice |
|---|---|
| Flight controller | ArduPilot (ArduCopter 4.6), EKF3 with external navigation, no GPS or compass |
| SLAM | **FAST-LIO2** (tightly coupled LiDAR + IMU) on a **Livox Mid-360** mounted upside-down |
| Companion computer | Raspberry Pi 5 (16 GB), ROS 2 Jazzy in Docker |
| Simulation | Gazebo Harmonic + ArduPilot SITL; the same autonomy code runs in sim and on the drone |
| Survivor detection | YOLO11n (ultralytics; export to NCNN on the Pi), localised with LiDAR depth |
| GCS | Web dashboard served by the drone (`:8080`), plus Mission Planner over MAVLink |

The full plan, architecture and test strategy are in **[docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md)**.

## Repository layout

```
compose.yaml                 docker services: sim (laptop) and robot (Pi 5)
docker/                      Dockerfile (targets: robot, sim), GPU/WSL overrides, shell env
deps.repos                   third-party ROS packages (FAST_LIO, livox_ros_driver2)
scripts/                     build_deps.sh, build_ws.sh
ros2_ws/src/
  airmouse/                  autonomy nodes (Python)
    pose_bridge_node.py        FAST-LIO -> map frame, ArduPilot vision pose, SLAM health
    mapper_node.py             3D scans -> 2D occupancy grid
    commander_node.py          mission state machine, frontier exploration, return to exit
    survivor_node.py           person detection + 3D localisation + grid tagging
    gcs_server_node.py         web GCS (map, grid, survivors, MJPEG video, START/RECALL/ABORT)
    web/index.html             GCS front-end (no internet needed)
  airmouse_sim/              ASCII maze -> Gazebo world generator, drone sensor model, bridge config
  airmouse_bringup/          launch files (sim, robot, core) and all configuration
docs/IMPLEMENTATION_PLAN.md
```

## 1. Run the simulation on a laptop

You need Docker. The images are about 8–10 GB, and the first build takes 30–60 minutes.

### Windows 11
1. Install WSL2 with Ubuntu 24.04: `wsl --install -d Ubuntu-24.04` (in an admin PowerShell), then reboot.
2. Install **Docker Desktop**. Enable *Settings → Resources → WSL integration → Ubuntu-24.04*.
3. Open the **Ubuntu** terminal. Clone the repo **inside the Linux filesystem**, not under `/mnt/c/...` or OneDrive, because a Windows path makes builds slow and breaks symlinks:
   ```bash
   cd ~ && git clone <your-repo-url> airmouse && cd airmouse
   docker compose build sim
   docker compose -f compose.yaml -f docker/compose.wsl.yaml up -d sim
   ```

### Linux (Ubuntu 22.04/24.04)
```bash
git clone <your-repo-url> airmouse && cd airmouse
docker compose build sim
xhost +local:docker
docker compose up -d sim          # add  -f docker/compose.nvidia.yaml  for an NVIDIA GPU
```

### Then, on any OS
```bash
docker compose exec sim bash      # open as many terminals as you like this way
am-build                          # colcon build + source (run again after code changes)
am-sim                            # Gazebo + ArduPilot SITL + FAST-LIO + MAVROS + AirMouse
```
- Open **http://localhost:8080** and press **START**. The drone takes off, explores, tags survivors, and lands at the exit.
- Optional: in Mission Planner on the host, connect with **TCP, 127.0.0.1, port 5762**.
- Headless (no Gazebo window): `ros2 launch airmouse_bringup sim.launch.py headless:=true`
- Other mazes: draw a layout (see `ros2_ws/src/airmouse_sim/mazes/maze_01.txt`), then `am-sim maze:=/airmouse/my_maze.txt`
- VS Code: *Dev Containers: Reopen in Container* uses the same `sim` service.

## 2. Run on the drone (Raspberry Pi 5)

1. Flash **Raspberry Pi OS Lite 64-bit (Bookworm)**. Install Docker with `curl -fsSL https://get.docker.com | sh`.
2. Give the Ethernet port a static IP of `192.168.1.50/24` for the Mid-360. Set the LiDAR IP (`192.168.1.1XX`, where XX is the last two digits of its serial) in `ros2_ws/src/airmouse_bringup/config/livox/MID360_config.json`.
3. Build and run:
   ```bash
   git clone <your-repo-url> airmouse && cd airmouse
   docker compose build robot           # builds natively on arm64, ~1 h
   docker compose up -d robot           # restarts automatically on boot
   docker compose exec robot bash
   am-build
   # one-off: faster detector on the Pi
   yolo export model=/opt/models/yolo11n.pt format=ncnn imgsz=416
   am-robot
   ```
4. On the GCS laptop, joined to the drone's local Wi-Fi, open `http://<pi-ip>:8080`. Mission Planner listens on UDP 14550.
5. Load `config/ardupilot/real.parm` onto the flight controller, then measure and fill in every `MEASURE` value in `config/airmouse.yaml`.

## 3. Sharing and moving to another laptop

- Everything needed is in this repo plus Docker. There are no host installs beyond Docker (and WSL on Windows).
- Third-party sources are pinned in `deps.repos`. After a known-good build, freeze exact commits with
  `vcs export --exact /opt/airmouse_deps/src > deps.repos`.
- Line endings are forced to LF (`.gitattributes`), so Windows checkouts still run in Linux containers.
- To ship a prebuilt image instead of rebuilding: `docker save airmouse:sim | gzip > airmouse_sim.tar.gz`, then `docker load < airmouse_sim.tar.gz` on the other machine.
