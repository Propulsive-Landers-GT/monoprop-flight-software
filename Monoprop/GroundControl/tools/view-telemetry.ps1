[CmdletBinding()]
param([string]$BindAddress = '127.0.0.1:10000')
$Host.UI.RawUI.WindowTitle = 'Monoprop incoming navigation telemetry'
$decoder = Join-Path $PSScriptRoot '..\..\..\..\monoprop-firmware\esp32Firmware\.pio\host-tools\monoprop-telemetry-monitor.exe'
if (!(Test-Path $decoder)) { throw 'Build the existing Postcard telemetry monitor first.' }
Write-Host "Receiving MPC's local telemetry copy on $BindAddress. No commands are sent."
Write-Host 'Position/velocity are the incoming Teensy estimates. Gimbal feedback is commanded, not measured angle.'
$nextDisplay = [DateTime]::MinValue
& $decoder --bind $BindAddress --schema modern | ForEach-Object {
    $now = [DateTime]::Now
    if ($now -ge $nextDisplay) {
        $record = $_ | ConvertFrom-Json
        $t = $record.telemetry
        $v = $t.vehicle_state
        $a = $t.actuator_command
        Write-Host ("{0:HH:mm:ss.fff} seq={1} t_us={2} phase={3} link={4}" -f $now,$t.sequence,$t.sensor_data.timestamp_us,$t.phase,$t.link_ok)
        Write-Host ("  position_m=[{0:F3}, {1:F3}, {2:F3}] velocity_mps=[{3:F3}, {4:F3}, {5:F3}]" -f $v.position_m[0],$v.position_m[1],$v.position_m[2],$v.velocity_mps[0],$v.velocity_mps[1],$v.velocity_mps[2])
        Write-Host ("  quaternion_wxyz=[{0}] angular_velocity_rps=[{1}]" -f ($v.attitude_quat -join ', '),($v.angular_velocity_rps -join ', '))
        Write-Host ("  throttle={0:F3} gimbal_deg=[{1:F2}, {2:F2}] GPS={3} lidar_m={4}" -f $a.thrust_norm,($a.gimbal_rad[0]*180/[Math]::PI),($a.gimbal_rad[1]*180/[Math]::PI),($t.sensor_data.gps_data -join ', '),$t.sensor_data.lidar_range_m)
        $nextDisplay = $now.AddMilliseconds(200)
    }
}
