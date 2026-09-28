<#
.SYNOPSIS
    Auto-detect flasher (J-Link or CMSIS-DAP) and flash firmware.

.DESCRIPTION
    Same single shortcut, "plug in and flash". Scans USB for a Segger J-Link
    (VID 0x1366). If present, flashes with Segger's own JLink.exe (native,
    fast, uses the installed Segger driver). Otherwise falls back to OpenOCD
    with the CMSIS-DAP config.

    NOTE: keep this file pure ASCII. Windows PowerShell 5.1 reads .ps1 using
    the system ANSI code page, so UTF-8 non-ASCII comments break parsing.

.PARAMETER Elf
    Path to the .elf (OpenOCD/DAP path). Default build/Debug/twitch_gimbal.elf
.PARAMETER Hex
    Path to the .hex (JLink.exe path). Default build/Debug/twitch_gimbal.hex
#>
param(
    [string]$Elf = "build/Debug/twitch_gimbal.elf",
    [string]$Hex = "build/Debug/twitch_gimbal.hex"
)

$ErrorActionPreference = "Stop"

# Project root = parent of this script's directory (scripts/ -> root)
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot

# --- Detect J-Link (VID_1366) --------------------------------------------
# -PresentOnly: only physically connected devices (avoids ghost devices).
# Do NOT filter on Status: J-Link composite child nodes flap OK/Unknown.
$jlink = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
    Where-Object { $_.InstanceId -match "VID_1366" } |
    Select-Object -First 1

if ($jlink) {
    # ---- J-Link path: use Segger's native JLink.exe -----------------------
    Write-Output "[flash] Detected J-Link (VID_1366), using Segger JLink.exe"

    $jlinkExe = (Get-Command JLink.exe -ErrorAction SilentlyContinue).Source
    if (-not $jlinkExe) {
        $fallbacks = @(
            "D:\Keil_v5\ARM\Segger\JLink.exe",
            "C:\Program Files\SEGGER\JLink\JLink.exe",
            "C:\Program Files (x86)\SEGGER\JLink\JLink.exe"
        )
        foreach ($fb in $fallbacks) {
            if (Test-Path -LiteralPath $fb) { $jlinkExe = $fb; break }
        }
        if (-not $jlinkExe) {
            Write-Error "[flash] JLink.exe not found in PATH or known install dirs"
            exit 1
        }
    }
    if (-not (Test-Path -LiteralPath $Hex)) {
        Write-Error "[flash] HEX not found: $Hex (build first?)"; exit 1
    }

    # Build a JLink CommanderScript: connect, program+verify, reset+run, exit
    $hexAbs = (Resolve-Path -LiteralPath $Hex).Path
    $script = @"
si SWD
speed 4000
device STM32F405RG
connect
loadfile "$hexAbs"
r
g
exit
"@
    $tmp = Join-Path $env:TEMP "flash_jlink_$PID.jlink"
    $script | Out-File -FilePath $tmp -Encoding ascii
    try {
        & $jlinkExe -CommanderScript $tmp -ExitOnError 1 -NoGui 1
        $code = $LASTEXITCODE
    } finally {
        Remove-Item $tmp -ErrorAction SilentlyContinue
    }
    exit $code
}

# ---- DAP path: OpenOCD ---------------------------------------------------
Write-Output "[flash] No J-Link found, using CMSIS-DAP + OpenOCD"

$openocd = (Get-Command openocd -ErrorAction SilentlyContinue).Source
if (-not $openocd) {
    $fallback = "C:\Tools\xpack-openocd-0.12.0-7-win32-x64\xpack-openocd-0.12.0-7\bin\openocd.exe"
    if (Test-Path -LiteralPath $fallback) { $openocd = $fallback }
    else { Write-Error "[flash] openocd not found in PATH or $fallback"; exit 1 }
}
if (-not (Test-Path -LiteralPath $Elf)) {
    Write-Error "[flash] ELF not found: $Elf (build first?)"; exit 1
}

Write-Output "[flash] Config: openocd.cfg | ELF: $Elf"
& $openocd -f "openocd.cfg" -c "program `"$Elf`" verify reset exit"
exit $LASTEXITCODE
