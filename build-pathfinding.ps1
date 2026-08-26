# Build the net48 player-runtime variant. The netstandard2.1 target remains
# available to the Blazor debugger and .NET tests.
param(
    [string]$BigAmbitionsManagedPath = $env:BA_GAME_MANAGED_PATH,
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $root "VoogleRoute.Pathfinding.csproj"

function Resolve-BigAmbitionsManagedPath {
    param([string]$RequestedPath)

    $candidates = @()
    if (-not [string]::IsNullOrWhiteSpace($RequestedPath)) {
        $candidates += $RequestedPath
    }

    $sdkRoot = [System.IO.Path]::GetFullPath((Join-Path $root "..\..\..\.."))
    $metadataPath = Join-Path $sdkRoot "UserSettings\BAModBuilder.ImportedDlls.json"
    if (Test-Path -LiteralPath $metadataPath) {
        try {
            $metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
            if (-not [string]::IsNullOrWhiteSpace([string]$metadata.installPath)) {
                $candidates += Join-Path ([string]$metadata.installPath) "Big Ambitions_Data\Managed"
            }
        }
        catch {
            throw "Invalid Big Ambitions DLL import metadata at '$metadataPath': $($_.Exception.Message)"
        }
    }

    $candidates += "C:\Program Files (x86)\Steam\steamapps\common\Big Ambitions\Big Ambitions_Data\Managed"
    $required = @("mscorlib.dll", "System.dll", "System.Core.dll")
    foreach ($candidate in $candidates | Select-Object -Unique) {
        $missing = @($required | Where-Object { -not (Test-Path -LiteralPath (Join-Path $candidate $_)) })
        if ($missing.Count -eq 0) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw "Big Ambitions Mono player assemblies were not found. Set BA_GAME_MANAGED_PATH or re-import the game DLLs in the SDK."
}

$BigAmbitionsManagedPath = Resolve-BigAmbitionsManagedPath $BigAmbitionsManagedPath
$managedArg = "-p:BigAmbitionsManagedPath=$BigAmbitionsManagedPath"
$tests = Join-Path $root "Tests\VoogleRoute.Pathfinding.Tests.csproj"
Push-Location $root
try {
    if (-not $SkipTests) {
        dotnet test $tests -c Release --nologo
        if ($LASTEXITCODE -ne 0) { throw "PathFinding tests failed." }
    }

    dotnet build $project -c Release -f net48 $managedArg --nologo
    if ($LASTEXITCODE -ne 0) { throw "PathFinding player-runtime build failed." }
    $dll = Join-Path $root "bin\Release\net48\VoogleRoute.Pathfinding.dll"
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

    # This .NET project lives below Unity's Assets folder. Leaving bin/obj here
    # makes Unity import duplicate VoogleRoute.Pathfinding assemblies and can
    # cause an older generated DLL to win over Dependencies. They are disposable
    # build artifacts; the authoritative player DLL has already been copied above.
    foreach ($relativeArtifactDir in @(
        "bin",
        "obj",
        "Tests\bin",
        "Tests\obj",
        "DiagRunner\bin",
        "DiagRunner\obj"
    )) {
        $artifactDir = [System.IO.Path]::GetFullPath((Join-Path $root $relativeArtifactDir))
        $rootPrefix = [System.IO.Path]::GetFullPath($root).TrimEnd("\", "/") + [System.IO.Path]::DirectorySeparatorChar
        if (-not $artifactDir.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to clean generated artifacts outside PathFinding: $artifactDir"
        }
        if (Test-Path -LiteralPath $artifactDir) {
            Remove-Item -LiteralPath $artifactDir -Recurse -Force
            Write-Host "Cleaned generated Unity-visible artifacts -> $artifactDir"
        }
    }
}
