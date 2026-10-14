$ErrorActionPreference = 'Stop'
$taskCommit = '2b3585aa766213ae9a46c4790ad4f3594ffa213d'
$taskFiles = @(
    @{ Source = 'data/sceneparts/urban_dyn/Building.mtl'; Local = 'Building.mtl'; Blob = 'ced7c932fa065a2dd0f3b869aaabbd9bbb9a75b3' }
    @{ Source = 'data/sceneparts/urban_dyn/Inliner-Bahn.obj'; Local = 'Inliner-Bahn.obj'; Blob = 'd6fd6b577f3710f3122b7204a435cfcf595293bc' }
    @{ Source = 'data/sceneparts/urban_dyn/Terrain.mtl'; Local = 'Terrain.mtl'; Blob = '9f56a2e2a34e06074087e255e820f72f70a5b0f6' }
    @{ Source = 'data/sceneparts/urban_dyn/Terrain.obj'; Local = 'Terrain.obj'; Blob = 'c3c24e8efc6e9de7bab32b3b42565caad02c6940' }
    @{ Source = 'data/sceneparts/urban_dyn/element.003.obj'; Local = 'element.003.obj'; Blob = '6c50dd7e469bbeb5bfdab036f75225834bf5e43a' }
    @{ Source = 'data/sceneparts/urban_dyn/element.005.obj'; Local = 'element.005.obj'; Blob = '23b90c3ef099a0fd8fc077ad54c9fd9fab0c93db' }
    @{ Source = 'data/sceneparts/urban_dyn/element.006.obj'; Local = 'element.006.obj'; Blob = '8a0c9034bf59f4487f1103653e81329922bc509a' }
    @{ Source = 'data/sceneparts/urban_dyn/element.008.obj'; Local = 'element.008.obj'; Blob = '63665c6f944b8eab35ba15e627de076269994907' }
    @{ Source = 'data/sceneparts/urban_dyn/element.013.obj'; Local = 'element.013.obj'; Blob = '75c6c791b72410141695a7112a7a844f1f393255' }
    @{ Source = 'data/sceneparts/urban_dyn/element.016.obj'; Local = 'element.016.obj'; Blob = '0e98c4fe275626ec69a98f97410d42f0c1d4cb46' }
    @{ Source = 'data/sceneparts/urban_dyn/element.017.obj'; Local = 'element.017.obj'; Blob = 'e45d21a552366c820bb8c585f7c14cdabdee9dba' }
    @{ Source = 'data/sceneparts/urban_dyn/element.018.obj'; Local = 'element.018.obj'; Blob = '2517eba0f65dc609889a4e630bf49a478e498423' }
    @{ Source = 'data/sceneparts/urban_dyn/element.019.obj'; Local = 'element.019.obj'; Blob = '6925370658c53685ba8e2549bd9cf0af985c49f2' }
    @{ Source = 'data/sceneparts/urban_dyn/element.020.obj'; Local = 'element.020.obj'; Blob = 'e7f52207a90e6e8970ee2fc2694c2854d5301fd1' }
    @{ Source = 'data/sceneparts/urban_dyn/element.021.obj'; Local = 'element.021.obj'; Blob = 'e9eab50fc10fd1d1f5833abc0b10671e5b274f9e' }
    @{ Source = 'data/sceneparts/urban_dyn/element.028.obj'; Local = 'element.028.obj'; Blob = 'afecc54dedd6b0d4278b182775ecca004b68784b' }
    @{ Source = 'data/sceneparts/urban_dyn/element.029.obj'; Local = 'element.029.obj'; Blob = '48339101851b1ff48955fca8a0a3689e9bdef728' }
    @{ Source = 'data/sceneparts/urban_dyn/element.030.obj'; Local = 'element.030.obj'; Blob = '6e67fb58580e69bd7ab13710d006442f3e717106' }
    @{ Source = 'data/sceneparts/urban_dyn/element.031.obj'; Local = 'element.031.obj'; Blob = '63a276d54bca5b1b329e6581c5dc8cd5e778bff7' }
    @{ Source = 'data/sceneparts/urban_dyn/element.032.obj'; Local = 'element.032.obj'; Blob = '1bd552a1ddd33772386781340cfcdb920c538416' }
    @{ Source = 'data/sceneparts/urban_dyn/element.033.obj'; Local = 'element.033.obj'; Blob = '12ec0322fec104ef14da0003029d43eee3e23b9b' }
    @{ Source = 'data/sceneparts/urban_dyn/element.036.obj'; Local = 'element.036.obj'; Blob = '1f01760d7ca14f95937deac0d4f859791f741880' }
    @{ Source = 'data/sceneparts/urban_dyn/element.037.obj'; Local = 'element.037.obj'; Blob = 'fde1372cf87e241db152fff104580d06d719d91a' }
    @{ Source = 'data/sceneparts/urban_dyn/element.obj'; Local = 'element.obj'; Blob = 'e7604ea3b6c982cf4ab6a88fdbc207b214d0dd13' }
    @{ Source = 'data/sceneparts/urban_dyn/small_maple.mtl'; Local = 'small_maple.mtl'; Blob = '61f548fc6083694d5685fdf18c41d144d8cf135b' }
    @{ Source = 'data/sceneparts/urban_dyn/small_maple.obj'; Local = 'small_maple.obj'; Blob = '00b8f3a78b9f97c0bc6f4e9d82f3544503be00d5' }
    @{ Source = 'data/sceneparts/urban_dyn/vivaro.000.obj'; Local = 'vivaro.000.obj'; Blob = '23bd7d2206df6f5cec7a8ccf1e768f60af7b3080' }
    @{ Source = 'data/sceneparts/urban_dyn/vivaro.001.obj'; Local = 'vivaro.001.obj'; Blob = '62c6a4a01313b51170af547c3e7029c9834cef2a' }
    @{ Source = 'data/sceneparts/urban_dyn/vivaro.mtl'; Local = 'vivaro.mtl'; Blob = 'bad9e84c72988075c48c0215608d13591a92f943' }
    @{ Source = 'data/sceneparts/urban_dyn/vivaro.obj'; Local = 'vivaro.obj'; Blob = '1fb72bbdd1544e3d814bd0005890a2ca25845dda' }
    @{ Source = 'data/sceneparts/urban_dyn/white_birch.mtl'; Local = 'white_birch.mtl'; Blob = '61f548fc6083694d5685fdf18c41d144d8cf135b' }
    @{ Source = 'data/sceneparts/urban_dyn/white_birch.obj'; Local = 'white_birch.obj'; Blob = 'd598085e7fd0a7cc7d9b50b1c50418fd3a74ba27' }
    @{ Source = 'COPYING'; Local = 'COPYING'; Blob = 'f288702d2fa16d3cdf0035b15a9fcbc552cd88e7' }
    @{ Source = 'COPYING.LESSER'; Local = 'COPYING.LESSER'; Blob = '0a041280bd00a9d068f503b8ee7ce35214bd24a1' }
    @{ Source = 'LICENSE.md'; Local = 'UPSTREAM_LICENSE.md'; Blob = '0402f89bf0fa539f80cbdcc0e250ac7732fc6c85' }
    @{ Source = 'data/scenes/dyn/urban_dyn_scene.xml'; Local = 'urban_dyn_scene.xml'; Blob = '53675a1b21fa63f02f3e156db7e0044ebb2fa070' }
)
foreach ($taskFile in $taskFiles) {
    $taskTarget = Join-Path $PSScriptRoot $taskFile.Local
    if (-not (Test-Path -LiteralPath $taskTarget)) {
        Write-Output ('Downloading ' + $taskFile.Local)
        Invoke-WebRequest -Uri ('https://raw.githubusercontent.com/3dgeo-heidelberg/helios/' + $taskCommit + '/' + $taskFile.Source) -OutFile $taskTarget
    }
    $taskBytes = [System.IO.File]::ReadAllBytes($taskTarget)
    $taskHeader = [System.Text.Encoding]::ASCII.GetBytes('blob ' + $taskBytes.Length + [char]0)
    $taskHash = [System.Security.Cryptography.SHA1]::Create()
    try {
        [void]$taskHash.TransformBlock($taskHeader, 0, $taskHeader.Length, $taskHeader, 0)
        [void]$taskHash.TransformFinalBlock($taskBytes, 0, $taskBytes.Length)
        $taskActual = [System.BitConverter]::ToString($taskHash.Hash).Replace('-', '').ToLowerInvariant()
    } finally { $taskHash.Dispose() }
    if ($taskActual -ne $taskFile.Blob) {
        throw ('Git blob hash mismatch; local file preserved, not overwritten: ' + $taskTarget)
    }
    Write-Output ('Verified ' + $taskFile.Local)
}
