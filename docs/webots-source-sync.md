# Webots Source Sync

This repository vendors the ArduPilot Webots Python example under:

```text
webots/
```

The source of truth is the official ArduPilot tree:

```text
libraries/SITL/examples/Webots_Python
```

Official upstream URL:

```text
https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Webots_Python
```

## Required Tree

The vendored tree should contain these directories:

```text
webots/controllers/
webots/params/
webots/protos/
webots/scripts/
webots/worlds/
```

Required baseline files include:

```text
webots/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py
webots/controllers/ardupilot_vehicle_controller/webots_vehicle.py
webots/params/iris.parm
webots/protos/Iris.proto
webots/protos/meshes/iris.dae
webots/protos/meshes/iris_prop_ccw.dae
webots/protos/meshes/iris_prop_cw.dae
webots/worlds/iris.wbt
webots/worlds/iris_camera.wbt
webots/worlds/iris_depth_camera.wbt
```

Do not copy only `worlds/` or only `params/`. Webots worlds depend on matching
controllers, PROTO files, meshes, textures, and scripts.

## Current Sync Method

The current tree was reset from the local ArduPilot checkout at:

```text
D:\ardupilot\libraries\SITL\examples\Webots_Python
```

Only files tracked by ArduPilot Git were copied. Generated files such as
`*.wbproj`, `__pycache__`, and hidden Webots project previews are intentionally
excluded.

## How to Re-Sync on Windows

From `D:\WeBots_Ardupilot`, after confirming `D:\ardupilot` is a valid
ArduPilot checkout:

```powershell
$sourceRoot = "D:\ardupilot"
$sourceWebots = "$sourceRoot\libraries\SITL\examples\Webots_Python"
$target = "D:\WeBots_Ardupilot\webots"
Remove-Item -LiteralPath $target -Recurse -Force
New-Item -ItemType Directory -Path $target | Out-Null
$files = git -c safe.directory=D:/ardupilot -C $sourceRoot ls-files libraries/SITL/examples/Webots_Python
foreach ($file in $files) {
  $relative = $file -replace '^libraries/SITL/examples/Webots_Python/', ''
  $src = Join-Path $sourceWebots ($relative -replace '/', '\')
  $dst = Join-Path $target ($relative -replace '/', '\')
  New-Item -ItemType Directory -Path (Split-Path -Parent $dst) -Force | Out-Null
  Copy-Item -LiteralPath $src -Destination $dst -Force
}
```

Review `git status` after sync before committing.
