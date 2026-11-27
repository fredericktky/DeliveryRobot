# flash.ps1 - build and flash the DeliveryRobot (ESP32-S3 native USB).
#
#   powershell -ExecutionPolicy Bypass -File tools\flash.ps1            # build + flash on COM3
#   powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -Port COM5
#   powershell -ExecutionPolicy Bypass -File tools\flash.ps1 -NoBuild   # flash the last build
#
# Before running: battery OFF (motors float during download and break USB),
# hold BOOT, tap RESET, release BOOT. Afterwards tap RESET to start the new code.
# Uses esptool --no-stub at 115200: plain "pio run -t upload" often fails with
# "No serial data received" on this board.
param(
  [string]$Port = "COM3",
  [switch]$NoBuild
)
$ErrorActionPreference = "Stop"
$root  = Split-Path -Parent $PSScriptRoot
$pio   = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
$py    = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$tool  = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"
$boot0 = Join-Path $env:USERPROFILE ".platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"
$build = Join-Path $root ".pio\build\esp32-s3-devkitc-1"

if (-not $NoBuild) {
  Push-Location $root
  try { & $pio run; if ($LASTEXITCODE -ne 0) { throw "Build failed" } }
  finally { Pop-Location }
}

Write-Host "`nFlashing on $Port (battery OFF, board in download mode: hold BOOT, tap RESET)..." -ForegroundColor Cyan
& $py $tool --chip esp32s3 --port $Port --baud 115200 --before default_reset --after hard_reset --no-stub `
  write_flash -z `
  0x0     (Join-Path $build "bootloader.bin") `
  0x8000  (Join-Path $build "partitions.bin") `
  0xe000  $boot0 `
  0x10000 (Join-Path $build "firmware.bin")
if ($LASTEXITCODE -ne 0) { throw "Flash failed - check battery OFF, BOOT+RESET, and the COM port" }
Write-Host "`nDone. Tap RESET, switch the battery on, then: auto loop / auto pid" -ForegroundColor Green
