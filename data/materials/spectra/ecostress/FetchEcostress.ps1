param([string]$Destination = $PSScriptRoot)
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$sourceUrl = 'https://zenodo.org/records/15881238/files/ecostress_orig.zip?download=1'
$expectedMd5 = '8df79582088b5dccc2d56907261b8449'
New-Item -ItemType Directory -Path $Destination -Force | Out-Null
$archivePath = Join-Path $Destination 'ecostress_orig.zip'
if (!(Test-Path -LiteralPath $archivePath)) {
    $temporaryPath = $archivePath + '.partial-' + [guid]::NewGuid().ToString('N')
    & curl.exe --fail --location --connect-timeout 30 --max-time 300 --retry 2 --output $temporaryPath $sourceUrl
    if ($LASTEXITCODE -ne 0) { throw "Download failed with curl exit code $LASTEXITCODE" }
    $actualMd5 = (Get-FileHash -LiteralPath $temporaryPath -Algorithm MD5).Hash.ToLowerInvariant()
    if ($actualMd5 -ne $expectedMd5) { throw "Downloaded archive checksum mismatch: $actualMd5" }
    Move-Item -LiteralPath $temporaryPath -Destination $archivePath
}
$actualMd5 = (Get-FileHash -LiteralPath $archivePath -Algorithm MD5).Hash.ToLowerInvariant()
if ($actualMd5 -ne $expectedMd5) { throw 'Existing archive differs from the published checksum; refusing to overwrite.' }
$sha256 = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
Add-Type -AssemblyName System.IO.Compression.FileSystem
$archive = [System.IO.Compression.ZipFile]::OpenRead($archivePath)
try {
    $rawPath = [IO.Path]::GetFullPath((Join-Path $Destination 'raw'))
    $prefix = $rawPath + [IO.Path]::DirectorySeparatorChar
    $entries = @($archive.Entries | ForEach-Object {
        $name = $_.FullName.Replace('/', [IO.Path]::DirectorySeparatorChar)
        $resolved = [IO.Path]::GetFullPath((Join-Path $rawPath $name))
        if (!$resolved.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) { throw "Unsafe archive entry: $name" }
        [pscustomobject]@{path=$_.FullName; bytes=$_.Length; compressedBytes=$_.CompressedLength}
    })
    $entries | Export-Csv -LiteralPath (Join-Path $Destination 'archive-contents.csv') -NoTypeInformation -Encoding utf8
    $manifest = [ordered]@{
        recordUrl='https://zenodo.org/records/15881238'; sourceUrl=$sourceUrl
        publisher='Harrison Nicholls / PROTEUS (public mirror, not a NASA-hosted download)'
        recordVersion='v3'; recordDate='2025-07-14'; retrievedUtc=[DateTime]::UtcNow.ToString('o')
        archive='ecostress_orig.zip'; archiveBytes=(Get-Item -LiteralPath $archivePath).Length
        expectedMd5=$expectedMd5; md5=$actualMd5; sha256=$sha256
        entryCount=$entries.Count
        fileCount=@($entries | Where-Object { !$_.path.EndsWith('/') }).Count
        spectrumFileCount=@($entries | Where-Object { $_.path.StartsWith('ecostress_raw/') -and !$_.path.EndsWith('/') }).Count
        uncompressedBytes=($entries | Measure-Object bytes -Sum).Sum
        officialSite='https://speclib.jpl.nasa.gov/'
        note='Original archive preserved byte-for-byte. No converted ecostress.zip was downloaded. No claim of equivalence to the current NASA full set.'
    }
    $manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $Destination 'download-manifest.json') -Encoding utf8
} finally { $archive.Dispose() }
if (!(Test-Path -LiteralPath $rawPath)) {
    [System.IO.Compression.ZipFile]::ExtractToDirectory($archivePath, $rawPath)
}
$manifest | ConvertTo-Json -Depth 5
