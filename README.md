# 运行测试

安装 `ostool`

```bash
cargo install ostool
```

运行测试

```bash
cargo test --test test -- tests --show-output
# 带uboot的开发板测试
cargo test --test test -- tests --show-output --uboot 
```
# 在WSL使用带uboot的开发板测试
1. 共享串口到wsl
```cmd
<!-- 使用管理员模式运行 -->
<!-- 获取已连接设备 -->
usbipd list
<!-- 共享 USB 设备共享到WSL -->
usbipd bind --busid BUSID
<!-- 将 USB 设备连接到 WSL -->
usbipd attach --wsl --busid BUSID
<!-- 断开 USB 设备连接 -->
usbipd detach --busid BUSID
<!-- 取消 USB 设备共享 -->
 usbipd unbind --busid BUSID
```

2. 使用tftp传输数据
```bash
# 查看 WSL 当前监听的端口
sudo ss -tulnp
# 在windows防火墙设置入站规则UDP端口
```

