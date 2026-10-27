param([switch]$Connect)
$ErrorActionPreference = 'Stop'
if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) { throw 'usbipd 未安装或不在 PATH 中。' }
$usbState = (& usbipd state | ConvertFrom-Json)
if ($LASTEXITCODE -ne 0) { throw '无法读取 USB 设备状态。' }
$canDevices = @($usbState.Devices | Where-Object { $_.InstanceId -match 'VID_1D50&PID_606F' -and $_.BusId })
if ($canDevices.Count -ne 1) { throw '请连接唯一的 candleLight USB-CAN。不会自动选择或转接雷达网卡。' }
$canDevice = $canDevices[0]
Write-Host "USB-CAN: $($canDevice.Description), BUSID=$($canDevice.BusId)"
$lidarHost = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object IPAddress -eq '192.168.1.111'
if (-not $lidarHost) { Write-Warning '未找到雷达网卡主机地址 192.168.1.111；先连接雷达网卡并检查静态 IP。' }
if (-not $Connect) { Write-Host '仅检查。需要转接时运行本脚本并加 -Connect。'; exit 0 }
if (-not $canDevice.PersistedGuid) {
    & usbipd bind --busid $canDevice.BusId
    if ($LASTEXITCODE -ne 0) { throw '共享失败；请在管理员 PowerShell 中运行本脚本。不要直接 bind --force。' }
}
if (-not $canDevice.ClientIPAddress) {
    & usbipd attach --wsl Ubuntu-22.04 --busid $canDevice.BusId
    if ($LASTEXITCODE -ne 0) { throw 'WSL 转接失败，请保留报错。' }
}
Write-Host 'USB-CAN 转接完成。进入 Ubuntu 后运行 prepare_wsl.sh；此处没有发送底盘指令。'
