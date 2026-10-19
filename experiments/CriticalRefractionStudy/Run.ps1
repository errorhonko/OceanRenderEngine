param(
    [string]$OutputDirectory = '',
    [string]$MSBuildPath = '',
    [string]$PythonPath = 'python',
    [string[]]$Configurations = @('Debug','Release')
)
$ErrorActionPreference = 'Stop'
$taskRepo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
if (!$MSBuildPath) {
    $taskTool = Get-Command MSBuild.exe -ErrorAction SilentlyContinue
    if (!$taskTool) { throw 'Pass -MSBuildPath or use a Visual Studio developer terminal.' }
    $MSBuildPath = $taskTool.Source
}
if (!$OutputDirectory) {
    $OutputDirectory = Join-Path $taskRepo ('out\CriticalRefractionStudy\run-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,6))
}
$taskOutput = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $taskOutput) { throw 'Choose a fresh output directory; no experiment outputs are overwritten.' }
New-Item -ItemType Directory -Path $taskOutput | Out-Null
foreach ($taskConfiguration in $Configurations) {
    if ($taskConfiguration -notin @('Debug','Release')) { throw 'Only Debug and Release supported.' }
    $taskBuildLog = Join-Path $taskOutput "$taskConfiguration-build.log"
    & $MSBuildPath (Join-Path $PSScriptRoot 'CriticalRefractionStudy.vcxproj') /t:Build /nologo /v:minimal "/p:Configuration=$taskConfiguration" /p:Platform=x64 *> $taskBuildLog
    if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath $taskBuildLog -Tail 35; throw 'Experiment build failed.' }
    $taskMatch = [regex]::Match((Get-Content -LiteralPath $taskBuildLog -Raw), '(?m)CriticalRefractionStudy\.vcxproj -> ([^\r\n]+\.exe)')
    $taskExe = $taskMatch.Groups[1].Value.Trim()
    if (!$taskExe) { throw 'No fresh executable identified from build log.' }
    $taskRaw = Join-Path $taskOutput "$taskConfiguration-raw.jsonl"
    & $taskExe $taskRaw *> (Join-Path $taskOutput "$taskConfiguration-replay.log")
    if ($LASTEXITCODE -ne 0) { throw 'Replay verification failed.' }
    & $PythonPath (Join-Path $PSScriptRoot 'Analyze.py') $taskRaw (Join-Path $taskOutput "$taskConfiguration-analysis.json") *> (Join-Path $taskOutput "$taskConfiguration-analysis.log")
    if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath (Join-Path $taskOutput "$taskConfiguration-analysis.log") -Tail 25; throw 'Reference analysis failed.' }
    $taskReport = Get-Content -LiteralPath (Join-Path $taskOutput "$taskConfiguration-analysis.json") -Raw | ConvertFrom-Json
    Write-Output "$taskConfiguration : $taskExe"
    Write-Output "Mixed max f=$($taskReport.mixedPrecision.allAcceptedMaxPairRelative) max PDF=$($taskReport.mixedPrecision.allAcceptedMaxPdfRelative) oracle=$($taskReport.mixedPrecision.selectedMaxQueryRelativeToIndependentReference)"
    Write-Output "Accepted=$($taskReport.replay.accepted) Critical=$($taskReport.replay.critical) Selected=$($taskReport.selectedSampleCount) WorstPair=$($taskReport.worst.currentRelative) DecimalConvergence=$($taskReport.reference80_vs_120_maxRelative)"
}
$taskSources = @('src\reflection\bxdfs\DielectricBxDF.h','src\reflection\bxdfs\BxDF.h','src\reflection\fresnel\FresnelUtils.h','src\reflection\microfacet\TrowbridgeReitzDistribution.h','src\core\math\Vector3f.h','experiments\CriticalRefractionStudy\main.cpp','experiments\CriticalRefractionStudy\Analyze.py')
$taskSources += 'experiments/CriticalRefractionStudy/LegacyFloatReference.h'
$taskHashes = foreach ($taskSource in $taskSources) {
    $taskHash = Get-FileHash -LiteralPath (Join-Path $taskRepo $taskSource) -Algorithm SHA256
    [pscustomobject]@{ path=$taskSource; sha256=$taskHash.Hash }
}
$taskHashes | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $taskOutput 'source-hashes.json') -Encoding utf8
Write-Output "Output: $taskOutput"
