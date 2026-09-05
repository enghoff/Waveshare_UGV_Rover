# Current rover host

The only deployment target is the Jetson Orin Nano mounted on the rover. General
deployment instructions are in [`deploy.md`](deploy.md).

## Host

| Item | Current value |
|---|---|
| SSH alias | `orin` |
| Hostname | `jetson-orin` |
| User | `jetson` |
| OS | Ubuntu 24.04 on NVIDIA L4T/JetPack 7 |
| Python | system CPython 3.12 |
| ROS | Jazzy in `~/miniforge3/envs/ros` |
| GPU path | TensorRT for world-state perception |
| Voice model | Alibaba-hosted Qwen Omni |
| Service address | `192.168.1.80` |
| mDNS fallback | `jetson-orin.local` |

The Orin replaced the Banana Pi M4 Zero on 2026-08-31. The Banana Pi is no
longer a deployment target. Chassis measurements remain applicable; device names
and computer-specific runtime assets do not.

World-state perception uses the Orin GPU through TensorRT. Its CPU fallback uses
ONNX Runtime, and observations identify the backend because their vectors are not
interchangeable. No local language model runs on the Orin.

## Hardware

| Device | Connection | Owner |
|---|---|---|
| General Driver board | `/dev/ttyTHS1`, 115200 | `rover_daemon` |
| D500 lidar | stable `/dev/serial/by-id/...`, 230400 | `ros_nav/lidar_node.py` |
| Xitech gimbal camera | stable `/dev/v4l/by-id/...-video-index0` | `rover_daemon` |
| OAK-D-Lite | USB2, DepthAI 2.32.0.0 | `oak_depth` |

ROS never opens the driver-board UART. The daemon lends odometry and motor
commands over loopback port 8772. A lidar serial node can exist while the rover's
main switch is off because the USB adapter is bus-powered; a live port with no
packets can still mean the lidar motor has no rover power.

Face detection is local YuNet. The model is tracked and deployed with
`face_tracking`. Capture also requires the host package `v4l-utils`.

The OAK uploads firmware from the DepthAI wheel on every open. It serves aligned
depth on loopback 8770 and may be intentionally powered off while the HTTP
service remains healthy.

## Network

NetworkManager manages the onboard Wi-Fi radio. `192.168.1.80` is the stable
service address written into each supported house-network profile. DHCP leases
such as `.88` are useful recovery addresses but can change. mDNS is the fallback
when the service address is unavailable.

The onboard radio is the active link. The USB Realtek dongle and its DKMS driver
are present but deliberately unmanaged; no current service depends on it. Wired
Ethernet has no carrier in the normal installation.

The privileged helper `/usr/local/sbin/wifi_ctl.sh` lists, scans and switches
profiles for the web console. Only the default profile autoconnects after boot;
switching to another profile briefly drops the browser connection.

```bash
ssh orin '/usr/local/sbin/wifi_ctl.sh status'
ssh orin 'journalctl -u NetworkManager -n 20'
```

Profile credentials live at `~/.ugv/wifi.key`, outside the deploy tree. The
installer reads them without placing secrets in command lines.

## Services

| Port | Binding/owner | Purpose |
|---:|---|---|
| 8769 | rover LAN / daemon | hardware and tool protocol |
| 8770 | loopback / OAK depth | aligned depth and ranges |
| 8771 | rover LAN / web | HTTPS console and audio WebSocket |
| 8772 | loopback / daemon | board bridge for ROS |
| 8773 | loopback / ROS | navigation bridge for daemon |
| 8774 | loopback / voice | image handoff for `look` |
| 8776 | loopback / world state | perception sidecar |

The console is `https://192.168.1.80:8771/`. TLS material is generated per host
and stored under `~/.ugv/tls/`; a new rover computer therefore requires trusting
its new CA on the workstation.

The `jetson` crontab starts the daemon, web console, OAK depth, ROS navigation and
world-state perception supervisors. Re-run the component installer if a crontab
entry is stale; do not edit the deployed source tree on the rover.

## Runtime state

These files are intentionally outside source deployment:

```text
~/.ugv/alibaba.key
~/.ugv/wifi.key
~/.ugv/tls/
~/.ugv/deploy-state.json
~/.ugv/world/
```

`~/ugv/odometry.json` is the chassis calibration and survives because flattened
component deployment is additive. It is the only host-side file that should be
copied when replacing the computer; if lost, ROS refuses to drive until the
chassis is recalibrated.

## Access and recovery

Use the SSH alias `orin` and edit source only in this repository. Deployment
credentials are local ignored files under `secrets/`. For a rover that has left
the network, follow [`rover-unresponsive.md`](rover-unresponsive.md); do not
guess at interface or profile changes while the only recovery link is remote.
