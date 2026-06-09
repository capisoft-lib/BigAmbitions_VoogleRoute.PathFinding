# VoogleRoute.Pathfinding

Shared A* pathfinding library for [Voogle Route](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute) (Unity mod) and [Voogle Route Web](https://github.com/capisoft-lib/BigAmbitions_VoogleRoute.Web).

| Property | Value |
|----------|-------|
| **Target** | .NET Standard 2.1 |
| **Assembly** | `VoogleRoute.Pathfinding.dll` |

## Build

```powershell
dotnet build VoogleRoute.Pathfinding.csproj -c Release
```

Output: `bin/Release/netstandard2.1/VoogleRoute.Pathfinding.dll`

## Consumers

| Project | Integration |
|---------|-------------|
| **VoogleRoute mod** | git submodule at `PathFinding/` → `tools/build-pathfinding.ps1` copies DLL to `Dependencies/` |
| **VoogleRoute.Web** | `<ProjectReference>` to this `.csproj` |

## License

MIT
