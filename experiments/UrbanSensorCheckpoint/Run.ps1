#requires -Version 7.0
param(
    [string]$OutputDirectory = '',
    [string]$PythonPath = 'python',
    [string]$RepositoryRoot = ''
)
$ErrorActionPreference = 'Stop'
if (!$RepositoryRoot) { $RepositoryRoot = Join-Path $PSScriptRoot '..\..' }
$taskRepo = [IO.Path]::GetFullPath($RepositoryRoot)
$taskCompiler = Get-Command cl.exe -ErrorAction SilentlyContinue
if (!$taskCompiler) { throw 'Use an x64 VS developer terminal with SDK 10.0.26100.0 and v145.' }
if (!$env:WindowsSDKVersion -or $env:WindowsSDKVersion.TrimEnd('\') -ne '10.0.26100.0') {
    throw 'Initialize the VS developer terminal with Windows SDK 10.0.26100.0.'
}
if (!(Test-Path -LiteralPath (Join-Path $taskRepo 'models\helios_urban_static\Terrain.obj')) -or
    !(Test-Path -LiteralPath (Join-Path $taskRepo 'data\materials\spectra\helios\asphalt.txt'))) {
    throw 'Fetch the pinned Urban assets and HELIOS spectra first; this script never downloads implicitly.'
}
& $PythonPath -c 'import numpy; from PIL import Image'
if ($LASTEXITCODE -ne 0) { throw 'Python needs numpy and Pillow for verification/plots.' }
if (!$OutputDirectory) {
    $OutputDirectory = Join-Path $taskRepo ('out\UrbanSensorCheckpoint\run-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,6))
}
$taskOutput = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $taskOutput) { throw 'Choose a fresh output directory; no previous run is overwritten.' }
New-Item -ItemType Directory -Path $taskOutput | Out-Null
$taskBuild = Join-Path $taskOutput 'build'
New-Item -ItemType Directory -Path $taskBuild | Out-Null
Push-Location -LiteralPath $taskBuild
try {
    foreach ($taskName in @('Scan','Render')) {
        $taskExe = Join-Path $taskBuild ($taskName + '.exe')
        $taskArguments = @('/nologo','/std:c++20','/EHsc','/utf-8','/W3','/MD','/O2',
            ('/I'+(Join-Path $taskRepo 'src')),('/I'+$taskRepo),
            (Join-Path $PSScriptRoot ($taskName+'.cpp')),
            (Join-Path $taskRepo 'src\scene\import\TinyObjLoaderImplementation.cpp'),
            ('/Fe:'+$taskExe),'/link','/INCREMENTAL:NO')
        & $taskCompiler.Source @taskArguments *> (Join-Path $taskOutput ($taskName+'-build.log'))
        if ($LASTEXITCODE -ne 0 -or !(Test-Path -LiteralPath $taskExe)) { throw "$taskName build failed; see build log." }
    }
    & (Join-Path $taskBuild 'Scan.exe') $taskRepo (Join-Path $taskOutput 'scan') *> (Join-Path $taskOutput 'Scan-run.log')
    if ($LASTEXITCODE -ne 0) { throw 'Scan failed; see Scan-run.log.' }
    & (Join-Path $taskBuild 'Render.exe') $taskRepo (Join-Path $taskOutput 'scan\urban_world.pcd') (Join-Path $taskOutput 'rgb') *> (Join-Path $taskOutput 'Render-run.log')
    if ($LASTEXITCODE -ne 0) { throw 'Render failed; see Render-run.log.' }
    foreach ($taskPlot in @(@('PlotScan.py','scan'),@('Compose.py','rgb'),@('Summarize.py','.'))) {
        & $PythonPath (Join-Path $PSScriptRoot $taskPlot[0]) (Join-Path $taskOutput $taskPlot[1]) *> (Join-Path $taskOutput ($taskPlot[0]+'.log'))
        if ($LASTEXITCODE -ne 0) { throw "$($taskPlot[0]) failed; see plot/validation log." }
    }
    $taskSources = @(Get-ChildItem -LiteralPath (Join-Path $taskRepo 'src') -Recurse -File)
    $taskSources += @(Get-ChildItem -LiteralPath $PSScriptRoot -File)
    $taskHashes = @($taskSources | Sort-Object FullName | ForEach-Object {
        [pscustomobject]@{path=[IO.Path]::GetRelativePath($taskRepo,$_.FullName); sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
    })
    $taskHashes | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $taskOutput 'source-hashes.json') -Encoding utf8
    Get-Content -LiteralPath (Join-Path $taskOutput 'scan\summary.txt')
    Get-Content -LiteralPath (Join-Path $taskOutput 'rgb\summary.txt')
    Write-Output "Verified local outputs: $taskOutput"
} finally { Pop-Location }
