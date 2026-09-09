[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$profilePath = 'C:\Espressif\tools\Microsoft.v5.4.4.PowerShell_profile.ps1'
if (!(Test-Path -LiteralPath $profilePath)) {
    throw 'ESP-IDF EIM v5.4.4 profile not found. Use your installed ESP-IDF terminal instead.'
}
. $profilePath
Set-Location -LiteralPath $PSScriptRoot
