# 在 ARM 板上从源码编译并打包

把**整份源码目录**拷到 ARM64 控机（`aarch64`）后，用和 x86 相同的命令编译、打包。不要把 x86 编出来的 `.so` 拷到 ARM 上用。

板端建议：Ubuntu 20.04/22.04 aarch64，已装 IgH 时前缀多为 **`/usr/local/etherlab`**（x86 开发机多为 `/opt/etherlab`）。CMake 与打包脚本会自动探测这两处。

---

## 1. 拷贝源码

在开发机上（示例）：

```bash
rsync -a --exclude build --exclude build-* --exclude dist \
  ./controller_sdk/  sunrise@<板IP>:~/controller_sdk/
```

也可用 `scp -r`。需要带上 `include/` `src/` `cmake/` `scripts/` `CMakeLists.txt` 等完整树；不需要已有的 `build/`。

SSH 建议走 **wlan0**；EtherCAT 绑 **eth0**，不要把 SSH 网卡配成主站网卡。

---

## 2. 板端依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev
# cmake 需要 ≥ 3.14（Ubuntu 20.04 自带 3.16 即可）
```

真控还需要 IgH 用户态（`ecrt.h` + `libethercat.so`）：

```bash
# 若已经能找到，可跳过
ls /usr/local/etherlab/include/ecrt.h /opt/etherlab/include/ecrt.h 2>/dev/null

# 没有则安装（ARM 默认装到 /usr/local/etherlab，内核模块按需）
sudo ./scripts/install_igh.sh --prefix=/usr/local/etherlab          # 用户态；有头文件则编当前内核模块
# sudo ./scripts/install_igh.sh --prefix=/usr/local/etherlab --no-modules
# 换内核后: sudo ./scripts/install_igh.sh --prefix=/usr/local/etherlab --modules-only
```

---

## 3. 编译

```bash
cd ~/controller_sdk
./scripts/build.sh
# 产物: build/lib/libcontroller_sdk.so*   build/bin/controller_runtime
```

ARM 上默认不编 Controller Studio。没有 IgH 时：`./scripts/build.sh --core-only`。

---

## 4. 打 ARM 二进制 SDK 包

IgH 已装好时，在板上一句即可：

```bash
./scripts/package_binary_sdk.sh
# 产物: dist/controller-sdk-binary_<ver>_aarch64.tar.gz
# 内容: include/  lib/*.so  bin/  lib/cmake/
```

没有 IgH、只要规划/Mock：

```bash
./scripts/package_binary_sdk.sh --core-only
```

打 `.deb`（可选）：

```bash
sudo ./scripts/bootstrap_all.sh --no-modules   # 已有 IgH 时会自动跳过安装
# 产物: dist/controller-sdk_<ver>_aarch64.deb
```

---

## 5. 真机网卡

部分镜像把主站配置写在 `/etc/sysconfig/ethercat`，也有写在 `/etc/ethercat.conf`。以本机实际文件为准：

```bash
ip a
sudo nano /etc/sysconfig/ethercat    # 或 /etc/ethercat.conf
# MASTER0_DEVICE="eth0 的 MAC"
# DEVICE_MODULES="generic"

sudo /etc/init.d/ethercat start
ethercat master
ethercat slaves
```

---

## 6. 不要做的事

| 错误做法 | 结果 |
|----------|------|
| 把 x86 的 `libcontroller_sdk.so` 拷到 ARM | 无法加载（ELF 架构不对） |
| 在 x86 上交叉编译却链到宿主机 `/opt/etherlab` | ARM 上跑会缺库或 SIGILL |
| EtherCAT 绑 SSH 那个网卡 | 远程断开 |
| 在 ARM 上开 `--studio=ON` 又没装 Qt | 仅跳过 Studio，SDK 仍可编；一般不必开 |

交叉编译仅在你已有 **aarch64 sysroot + ARM 版 IgH** 时才用  
`cmake/toolchain-aarch64-linux-gnu.cmake`。日常请在板子上本机编。
