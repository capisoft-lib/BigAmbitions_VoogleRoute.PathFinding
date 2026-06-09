param(
    [Parameter(Mandatory = $true)]
    [string] $ModRoot
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path $PSScriptRoot -Parent
$source = Join-Path $repoRoot "data"
$dest = Join-Path $ModRoot "Data"

if (-not (Test-Path $source)) {
    throw "Missing data folder: $source"
}

New-Item -ItemType Directory -Force -Path $dest | Out-Null

Get-ChildItem $source -Filter "*.csv" | ForEach-Object {
    Copy-Item $_.FullName -Destination (Join-Path $dest $_.Name) -Force
    Write-Host "Copied -> $(Join-Path $dest $_.Name)"
}
