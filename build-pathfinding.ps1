# Build VoogleRoute.Pathfinding.dll (netstandard2.1) for Unity mod + Blazor.
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location $root
try {
    dotnet build -c Release
    $dll = Join-Path $root "bin\Release\netstandard2.1\VoogleRoute.Pathfinding.dll"
    if (-not (Test-Path $dll)) { throw "Build output missing: $dll" }

    $targets = @(
        (Join-Path $root "..\Dependencies")
    )
    foreach ($dir in $targets) {
        if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
        Copy-Item $dll (Join-Path $dir "VoogleRoute.Pathfinding.dll") -Force
        Write-Host "Copied -> $dir"
    }
}
finally {
    Pop-Location
}
