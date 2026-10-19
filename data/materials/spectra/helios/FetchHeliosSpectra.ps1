param([string]$Destination = $PSScriptRoot)
$ErrorActionPreference = 'Stop'
$revision = '2b3585aa766213ae9a46c4790ad4f3594ffa213d'
$spectraPath = 'python/pyhelios/data/spectra/'
# Git blob hashes pin exact upstream bytes, including original headers and units.
$items = @(
    @{ Name='aluminum.txt'; Path=$spectraPath+'aluminum.txt'; Blob='79e93b5ce712f57dceb6444305add2c701228444' }
    @{ Name='asphalt.txt'; Path=$spectraPath+'asphalt.txt'; Blob='6de7ecac1781262b66532bfb5a7eca6f1f4b7eaf' }
    @{ Name='conifer.txt'; Path=$spectraPath+'conifer.txt'; Blob='b8702e1989f162a0e0dd5e8eab741e462cdee5bb' }
    @{ Name='copper.txt'; Path=$spectraPath+'copper.txt'; Blob='c102e5e97c388a646eb957162a7f4d0a72ab4d90' }
    @{ Name='deciduous.txt'; Path=$spectraPath+'deciduous.txt'; Blob='62ac998b9edb618a6c9709a60478b95ee1cf5294' }
    @{ Name='grass.txt'; Path=$spectraPath+'grass.txt'; Blob='22f85674b2d7afa83fe03f9db2d82a84d7f96e58' }
    @{ Name='grass_dry.txt'; Path=$spectraPath+'grass_dry.txt'; Blob='79c62af752785ab78b37067e908c8eb5b32c4592' }
    @{ Name='shingle_gray.txt'; Path=$spectraPath+'shingle_gray.txt'; Blob='32ba357bca7822d6a15d0e606747523bf3561587' }
    @{ Name='shingle_red.txt'; Path=$spectraPath+'shingle_red.txt'; Blob='782fe86547e82ead8bcc8f373899f7d6d6a4ab4c' }
    @{ Name='wood.txt'; Path=$spectraPath+'wood.txt'; Blob='b90ada8df30b7385657360c7ff9e343860f41ce8' }
    @{ Name='UPSTREAM_LICENSE.md'; Path='LICENSE.md'; Blob='0402f89bf0fa539f80cbdcc0e250ac7732fc6c85' }
    @{ Name='COPYING'; Path='COPYING'; Blob='f288702d2fa16d3cdf0035b15a9fcbc552cd88e7' }
    @{ Name='COPYING.LESSER'; Path='COPYING.LESSER'; Blob='0a041280bd00a9d068f503b8ee7ce35214bd24a1' }
)
function Get-GitBlobHash([string]$Path) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    $header = [Text.Encoding]::ASCII.GetBytes("blob $($bytes.Length)`0")
    $sha = [Security.Cryptography.SHA1]::Create()
    try {
        [void]$sha.TransformBlock($header, 0, $header.Length, $header, 0)
        [void]$sha.TransformFinalBlock($bytes, 0, $bytes.Length)
        return [BitConverter]::ToString($sha.Hash).Replace('-','').ToLowerInvariant()
    } finally { $sha.Dispose() }
}
$destinationRoot = [IO.Path]::GetFullPath($Destination)
New-Item -ItemType Directory -Path $destinationRoot -Force | Out-Null
$records = foreach ($item in $items) {
    $target = Join-Path $destinationRoot $item.Name
    $url = 'https://raw.githubusercontent.com/3dgeo-heidelberg/helios/' + $revision + '/' + $item.Path
    if (Test-Path -LiteralPath $target) {
        if ((Get-GitBlobHash $target) -ne $item.Blob) { throw "Existing file differs; refusing to overwrite: $target" }
    } else {
        $temporary = Join-Path $destinationRoot ('.download-' + [guid]::NewGuid().ToString('N'))
        Invoke-WebRequest -Uri $url -OutFile $temporary -TimeoutSec 45
        if ((Get-GitBlobHash $temporary) -ne $item.Blob) { throw "Hash mismatch; untrusted download retained at $temporary" }
        Move-Item -LiteralPath $temporary -Destination $target
    }
    Write-Host ('Verified ' + $item.Name)
    [ordered]@{
        file=$item.Name; upstreamPath=$item.Path; url=$url; gitBlobSha1=$item.Blob
        sha256=(Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash.ToLowerInvariant()
        bytes=(Get-Item -LiteralPath $target).Length
    }
}
$manifest = [ordered]@{
    schemaVersion=1; sourceRepository='https://github.com/3dgeo-heidelberg/helios'
    upstreamCommit=$revision; fetchedUtc=[DateTime]::UtcNow.ToString('o')
    note='Original source bytes. Project GPL/LGPL notices do not establish a separate license for the underlying ASTER/JPL measurements. Review source headers and data terms before redistribution.'
    files=@($records)
}
# Generated acquisition manifest, not an input containing model parameters.
[IO.File]::WriteAllText((Join-Path $destinationRoot 'download-manifest.json'),
    ($manifest | ConvertTo-Json -Depth 5), [Text.UTF8Encoding]::new($false))
