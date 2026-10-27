# Parse only: do not execute the USB preparation script.
$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot 'prepare_windows.ps1'
$tokens = $null
$parseErrors = $null
[System.Management.Automation.Language.Parser]::ParseFile($target, [ref]$tokens, [ref]$parseErrors) | Out-Null
if ($parseErrors.Count -gt 0) {
    $parseErrors | Format-List
    exit 1
}
Write-Host "PASS: prepare_windows.ps1 parses in PowerShell $($PSVersionTable.PSVersion). No USB operation performed."
