param([string]$DestinationRoot = (Join-Path $PSScriptRoot '..\archives\deliveries'))
$ErrorActionPreference = 'Stop'
$sourceRoot = $PSScriptRoot
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$deliveryRoot = Join-Path $DestinationRoot "导航交付_$stamp"
New-Item -ItemType Directory -Path $DestinationRoot -Force | Out-Null
New-Item -ItemType Directory -Path $deliveryRoot | Out-Null
$extensions = @('.py','.sh','.yaml','.xml','.ps1','.md','.json','.png','.svg','.log')
$paths = @(Get-ChildItem -LiteralPath $sourceRoot -File | Where-Object Extension -in $extensions)
foreach ($folder in @('generated','results','ws\src')) {
    $paths += @(Get-ChildItem -LiteralPath (Join-Path $sourceRoot $folder) -File -Recurse | Where-Object { $_.FullName -notmatch '[\\/]__pycache__[\\/]' })
}
foreach ($item in $paths) {
    $relative = $item.FullName.Substring($sourceRoot.Length).TrimStart('\','/')
    $target = Join-Path (Join-Path $deliveryRoot 'tracer_navigation') $relative
    New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
    Copy-Item -LiteralPath $item.FullName -Destination $target
}
Copy-Item -LiteralPath (Join-Path $sourceRoot '..\明天先读_自主导航.md') -Destination $deliveryRoot
$mapRoot = Join-Path $sourceRoot '..\reconstruction_20261017_113920\raytrace'
$mapTarget = Join-Path $deliveryRoot 'fixed_map_original'
New-Item -ItemType Directory -Path $mapTarget | Out-Null
foreach ($name in @('global_map.pc2.lcm','global_costmap.lcm')) {
    Copy-Item -LiteralPath (Join-Path $mapRoot $name) -Destination $mapTarget
}
@'
本包保存导航代码、配置、固定地图副本和测试结果。
离线模拟通过不表示实车已经完成方向、制动或导航测试。
不包含 WSL 系统、Python 虚拟环境、整个 DimOS 库或编译依赖。
在当前电脑上继续使用原路径；请勿覆盖原始采集数据。
恢复时先读取操作卡；变更代码后必须重新现场验收。
'@ | Set-Content -LiteralPath (Join-Path $deliveryRoot '备份说明.txt') -Encoding utf8
$manifest = @(Get-ChildItem -LiteralPath $deliveryRoot -Recurse -File | ForEach-Object {
    [PSCustomObject]@{path=$_.FullName.Substring($deliveryRoot.Length+1); bytes=$_.Length; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
})
$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $deliveryRoot 'manifest_sha256.json') -Encoding utf8
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zipPath = "$deliveryRoot.zip"
[System.IO.Compression.ZipFile]::CreateFromDirectory($deliveryRoot,$zipPath)
$zip = [System.IO.Compression.ZipFile]::OpenRead($zipPath)
try {
    if ($zip.Entries.Count -ne $manifest.Count+1) { throw 'ZIP 条目数与清单不匹配。' }
    foreach ($record in $manifest) {
        $entry = $zip.GetEntry($record.path.Replace('\','/'))
        if (-not $entry) { throw "ZIP 缺少文件：$($record.path)" }
        $stream = $entry.Open()
        $hasher = [System.Security.Cryptography.SHA256]::Create()
        try { $digest = ([BitConverter]::ToString($hasher.ComputeHash($stream))).Replace('-','').ToLowerInvariant() }
        finally { $stream.Dispose(); $hasher.Dispose() }
        if ($digest -ne $record.sha256) { throw "ZIP 文件校验失败：$($record.path)" }
    }
} finally { $zip.Dispose() }
Get-FileHash -LiteralPath $zipPath -Algorithm SHA256 | Format-List
Write-Output "BACKUP=$zipPath"
