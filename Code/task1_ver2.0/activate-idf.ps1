[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

# This ESP-IDF installation was created by Espressif Installation Manager (EIM).
# EIM keeps its Python environment and tools under C:\Espressif\tools, so the
# generic esp-idf\export.ps1 (which searches the user's .espressif directory)
# is not the correct entry point for this machine.
$eimProfile = 'C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
$idfPython = 'C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe'
$idfProject = Split-Path -Parent $MyInvocation.MyCommand.Path

foreach ($requiredPath in @($eimProfile, $idfPython)) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "ESP-IDF required file not found: $requiredPath`nRepair ESP-IDF v5.4.4 with Espressif Installation Manager."
    }
}

# Dot-source the EIM-generated profile so its environment variables, aliases,
# and command wrappers remain available in the current PowerShell process.
. $eimProfile

if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    throw 'ESP-IDF activation finished, but idf.py is still unavailable.'
}

Set-Location -LiteralPath $idfProject
Write-Host ''
Write-Host "Project: $idfProject"
Write-Host 'ESP-IDF environment is ready. Try: idf.py build'
