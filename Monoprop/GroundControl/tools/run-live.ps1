[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Target,
    [string]$BindAddress = '0.0.0.0:9999',
    [string]$TelemetryMirror = '',
    [string]$Executable = ''
)
$ErrorActionPreference = 'Stop'
$Host.UI.RawUI.WindowTitle = 'Monoprop LIVE commands - TVC Ready checkout'
$exe = Join-Path $env:LOCALAPPDATA 'Monoprop\ground-control-target\x86_64-pc-windows-msvc\release\GroundControl.exe'
if ($Executable) { $exe = $Executable }
$config = Join-Path $PSScriptRoot '..\ground.toml'
if (!(Test-Path $exe)) { throw 'Build the Windows GroundControl executable first.' }
Write-Host 'LIVE transmission enabled. No commands sent automatically.'
Write-Host 'After fresh Ready telemetry: tvc 1 0, tvc 0 1, tvc 0 0. Angles are degrees.'
Write-Host 'TVC checkout always requests zero motor thrust. stop ends streaming; disarm returns to Standby.'
Write-Host 'MPC uses telemetry position/velocity, fresh navigation and a successful solve. Descent tracks a gradual return to the origin.'
# Windows PowerShell represents native stderr diagnostics as ErrorRecords.
# Keep reporting those lines without terminating the running controller.
$ErrorActionPreference = 'Continue'
$controllerArgs = @("--config", $config, "--target", $Target, "--bind", $BindAddress, "--send")
if ($TelemetryMirror) { $controllerArgs += @("--telemetry-mirror", $TelemetryMirror) }
& $exe @controllerArgs 2>&1 |
    Tee-Object -FilePath (Join-Path $env:LOCALAPPDATA 'Monoprop\ground-control-live.log')
$controllerExit = $LASTEXITCODE
$ErrorActionPreference = 'Stop'
if ($controllerExit -ne 0) { throw "GroundControl exited with code $controllerExit" }
