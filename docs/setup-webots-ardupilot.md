# Webots and ArduPilot SITL Setup

This repo assumes Ubuntu 24.04, Python 3.12, Webots, and an external ArduPilot checkout.

## External ArduPilot Checkout

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
```

Build ArduCopter SITL:

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

## Baseline SITL Check

Before using Webots, validate plain SITL:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --map --console -w
```

In MAVProxy:

```text
mode guided
arm throttle
takeoff 1
mode land
```

This is only a plain SITL sanity check. The autonomy mission now uses the same
high-level approach: send `MAV_CMD_NAV_TAKEOFF` to `1.0 m`, then wait for
telemetry to settle before starting the gate mission.

## Webots SITL Check

This repository vendors a full copy of ArduPilot's `Webots_Python` example under
`webots/`. Use this local copy for baseline tests and future custom worlds.

For the current gate-perception simulation, open Webots first and load:

```text
<repo>/webots/worlds/iris_camera.wbt
```

Use `<repo>/webots/worlds/iris.wbt` only for camera-free baseline checks.

Then run from this repository:

```bash
cp configs/sitl_webots.env.example configs/sitl_webots.env
scripts/run_sitl_webots.sh
```

`ARDUPILOT_HOME` still points to the external ArduPilot checkout because SITL and
`sim_vehicle.py` run from ArduPilot. `WEBOTS_EXAMPLE_HOME` points to this repo's
vendored `webots/` directory.

The companion app expects MAVLink telemetry on:

```text
udp:127.0.0.1:14551
```

## Smoke Test

```bash
drone-autonomy --connection udp:127.0.0.1:14551 --mode heartbeat
```

Expected result: the app prints heartbeat and telemetry messages from ArduPilot.

Fallback if the console script is not installed in `PATH`:

```bash
python -m drone_autonomy.cli --connection udp:127.0.0.1:14551 --mode heartbeat
```
