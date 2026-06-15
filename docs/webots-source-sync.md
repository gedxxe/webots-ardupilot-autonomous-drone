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

## Current Audit Result

Final audit result for the vendored tree:

```text
official upstream tracked files: 36
local repo tracked files:        36
missing upstream files:          0
extra local tracked files:       0
```

The upstream head used for the internet verification was:

```text
ArduPilot master: 988d7b01c75fc0d76990308d10a867f646b8e1e2
```

Folder timestamps are not a reliable completeness signal. Windows copy
operations and Git checkout behavior can preserve file modification timestamps
from the source tree. Use `git ls-files`, file counts, and upstream comparison
instead.

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

## How to Audit Against Upstream

Use this PowerShell check from `D:\WeBots_Ardupilot` when still on Windows:

```powershell
$api='https://api.github.com/repos/ArduPilot/ardupilot/git/trees/master?recursive=1'
$tree=(Invoke-RestMethod -Headers @{ 'User-Agent'='webots-audit' } -Uri $api).tree
$prefix='libraries/SITL/examples/Webots_Python/'
$official=$tree |
  Where-Object { $_.type -eq 'blob' -and $_.path.StartsWith($prefix) } |
  ForEach-Object { $_.path.Substring($prefix.Length).Replace('/','\') } |
  Sort-Object
$local=git ls-files webots |
  ForEach-Object { $_.Substring('webots/'.Length).Replace('/','\') } |
  Sort-Object
Compare-Object $official $local
```

Expected output is empty. Empty output means the local `webots/` tracked files
match the official ArduPilot `Webots_Python` tracked files.
