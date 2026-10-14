param([string]$OutputDirectory = '')
$ErrorActionPreference = 'Stop'
$taskCompiler = Get-Command cl.exe -ErrorAction SilentlyContinue
if (-not $taskCompiler) { throw 'Run in a Visual Studio x64 Native Tools terminal; cl.exe is required.' }
$taskRepo = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $taskRepo ('out\UrbanStaticImport\run-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,6))
}
$taskOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $taskOutput) { throw 'Choose a fresh output directory; existing reports are not overwritten.' }
New-Item -ItemType Directory -Path $taskOutput | Out-Null
Push-Location -LiteralPath $taskOutput
try {
    $taskArguments = @('/nologo','/std:c++20','/EHsc','/utf-8','/FC','/MD','/O2',('/I' + (Join-Path $taskRepo 'src')),('/I' + $taskRepo),'/Fe:UrbanStaticImport.exe',
        (Join-Path $PSScriptRoot 'main.cpp'),
        (Join-Path $taskRepo 'tests\OceanRenderEngine.Tests\HeliosUrbanStaticSceneAcceptance.cpp'),
        (Join-Path $taskRepo 'src\scene\import\TinyObjLoaderImplementation.cpp'))
    & $taskCompiler.Source @taskArguments 2>&1 | Tee-Object -FilePath 'build.log'
    if ($LASTEXITCODE -ne 0) { throw 'Urban static import compilation failed; see build.log.' }
    & '.\UrbanStaticImport.exe' $taskOutput 2>&1 | Tee-Object -FilePath 'run.log'
    if ($LASTEXITCODE -ne 0) { throw 'Urban static import verification failed; see run.log.' }
    Write-Output ('Import reports: ' + $taskOutput)
} finally { Pop-Location }
