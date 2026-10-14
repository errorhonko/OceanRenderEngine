$ErrorActionPreference = 'Stop'
$taskCommit = '2b3585aa766213ae9a46c4790ad4f3594ffa213d'
$taskFiles = @(
    @('cube.obj','data/sceneparts/toyblocks/cube.obj','702d3fe8f1e7ffab7268a9b7f9892eaa17887335492d5223234ff84a814e67ee'),
    @('cube.mtl','data/sceneparts/toyblocks/cube.mtl','c90fda5a7ad51fb934b0aa7af032bc739ab4b7cf6d28d11672f5693237de8f96'),
    @('sphere.obj','data/sceneparts/toyblocks/sphere.obj','808b2760b007da74d8449ec696d95ad7dc62059a5f75e19a6d03e2991516f132'),
    @('sphere.mtl','data/sceneparts/toyblocks/sphere.mtl','9b6a7d84b00bab88f974dd6d9a74d7bd87491c3c81f35f2f06389a3c10d38390'),
    @('cylinder.obj','data/sceneparts/toyblocks/cylinder.obj','77d65209b09c05747f0965acdf801b3ee9f4f5367946d92312a219a2d5802eef'),
    @('cylinder.mtl','data/sceneparts/toyblocks/cylinder.mtl','db976d7884db36bb76005666dd7f1b5743204c3d0afab7015b44f844925db0f6'),
    @('groundplane.obj','data/sceneparts/basic/groundplane/groundplane.obj','7b8f5abfa24227c382c64130a33283bc53f9db0f653a4ec13cc707ba933ea283'),
    @('groundplane.mtl','data/sceneparts/basic/groundplane/groundplane.mtl','014cf181d18886695bcdf806fbb69ee80ded5aed5f1d42561c02532e228fe24f'),
    @('toyblocks_scene.xml','data/scenes/toyblocks/toyblocks_scene.xml','f2a065e23bcfab581782958351df6cdd95ad69b597110d38d09e0c55ac54dd30'),
    @('UPSTREAM_LICENSE.md','LICENSE.md','32111428865a68db182b87c224f84b374bae3cca9eb2957c868b33e352836855'),
    @('COPYING','COPYING','3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986'),
    @('COPYING.LESSER','COPYING.LESSER','e3a994d82e644b03a792a930f574002658412f62407f5fee083f2555c5f23118')
)
foreach ($taskFile in $taskFiles) {
    $taskDestination = Join-Path $PSScriptRoot $taskFile[0]
    if (!(Test-Path -LiteralPath $taskDestination)) {
        Invoke-WebRequest -Uri ('https://raw.githubusercontent.com/3dgeo-heidelberg/helios/' + $taskCommit + '/' + $taskFile[1]) -OutFile $taskDestination
    }
    if ((Get-FileHash -LiteralPath $taskDestination -Algorithm SHA256).Hash -ne $taskFile[2]) {
        throw ('Asset hash mismatch; preserving local file: ' + $taskDestination)
    }
    '[VERIFIED] ' + $taskFile[0]
}
