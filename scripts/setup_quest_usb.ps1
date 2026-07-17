$ErrorActionPreference = 'Stop'

$adbCommand = Get-Command adb -ErrorAction SilentlyContinue
if ($null -eq $adbCommand) {
    $fallback = Join-Path $env:LOCALAPPDATA 'Android\Sdk\platform-tools\adb.exe'
    if (-not (Test-Path -LiteralPath $fallback)) {
        throw 'ADB was not found. Install Android SDK Platform Tools or Meta Quest Developer Hub.'
    }
    $adb = $fallback
} else {
    $adb = $adbCommand.Source
}

& $adb start-server | Out-Host
$deviceLines = & $adb devices
$authorized = @($deviceLines | Where-Object { $_ -match "\sdevice$" })
if ($authorized.Count -eq 0) {
    & $adb devices -l | Out-Host
    throw 'No authorized Quest was found. Enable Developer Mode, reconnect a data-capable USB cable, and accept Allow USB debugging inside the headset.'
}

& $adb reverse tcp:8443 tcp:8443 | Out-Host
Write-Host 'Quest USB forwarding is ready. Open http://localhost:8443/ in the Quest Browser.' -ForegroundColor Green

