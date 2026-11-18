<#
.SYNOPSIS
  SMART-LINK C++17 Launcher & Build Script (Windows — HP ProBook / HP Victus)
.EXAMPLE
  .\smartlink.ps1 sync
  .\smartlink.ps1 build
  .\smartlink.ps1 test
  .\smartlink.ps1 hq -Name HP-ProBook -Id 4 -Serial COM3
  .\smartlink.ps1 hq -Name HP-Victus  -Id 5 -FmRx
#>
param(
  [Parameter(Position = 0)]
  [ValidateSet("sync", "build", "test", "hq", "relay", "ports")]
  [string]$Command = "hq",

  [string]$Name = $env:COMPUTERNAME,
  [int]$Id = 4,
  [int]$Port = 5050,
  [int]$Baud = 38400,
  [string[]]$Serial = @(),
  [string[]]$Peer = @(),
  [switch]$FmRx,
  [switch]$NoFmTx,
  [switch]$LogVerbose
)

$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot
$CoreSrc = Join-Path $Root "firmware\common\smartlink_core.h"
$Exe = Join-Path $Root "engine\build\Release\smartlink.exe"

function Sync-CoreHeader {
  foreach ($dir in @("firmware\esp32_node", "firmware\uno_node")) {
    $dst = Join-Path $Root "$dir\smartlink_core.h"
    Copy-Item -Path $CoreSrc -Destination $dst -Force
    Write-Host "[SYNC] Copied smartlink_core.h -> $dir" -ForegroundColor Green
  }
}

function Build-Engine {
  Sync-CoreHeader
  $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
  if (-not (Test-Path $vswhere)) { throw "Visual Studio Installer (vswhere.exe) not found." }
  $vsPath = & $vswhere -latest -products * -property installationPath
  $vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
  if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }

  $srcDir = Join-Path $Root "engine"
  $bldDir = Join-Path $Root "engine\build"
  cmd /c "`"$vcvars`" >nul && cmake -S `"$srcDir`" -B `"$bldDir`" && cmake --build `"$bldDir`" --config Release"
  if ($LASTEXITCODE -ne 0) { throw "CMake build failed." }
  Write-Host "[BUILD] Ready: $Exe" -ForegroundColor Green
}

switch ($Command) {
  "sync" {
    Sync-CoreHeader
  }
  "build" {
    Build-Engine
  }
  "test" {
    if (-not (Test-Path $Exe)) { Build-Engine }
    & $Exe --self-test
  }
  "ports" {
    [System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Host $_ }
  }
  default {
    if (-not (Test-Path $Exe)) { Build-Engine }
    $role = if ($Command -eq "relay") { "relay" } else { "hq" }
    $argsList = @("--role", $role, "--name", $Name, "--id", $Id, "--port", $Port, "--baud", $Baud)
    foreach ($s in $Serial) { $argsList += @("--serial", $s) }
    foreach ($p in $Peer)   { $argsList += @("--peer", $p) }
    if ($FmRx)    { $argsList += "--fm-rx" }
    if ($NoFmTx)  { $argsList += "--no-fm-tx" }
    if ($LogVerbose) { $argsList += "--verbose" }
    & $Exe @argsList
  }
}
