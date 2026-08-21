#!/usr/bin/env bash
# 安装 IgH EtherCAT Master（用户态 + 按本机内核编译模块）
#
# 用法:
#   sudo ./scripts/install_igh.sh                  # 用户态 + 能编模块则编
#   sudo ./scripts/install_igh.sh --no-modules     # 仅用户态（打包机 / Docker）
#   sudo ./scripts/install_igh.sh --with-modules   # 必须编出与当前内核匹配的模块
#   sudo ./scripts/install_igh.sh --modules-only   # 内核升级后只重编模块
#
# 环境变量: IGH_VERSION  ETHERLAB_ROOT  IGH_SRC_DIR  IGH_GIT_URL  JOBS
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib.sh
if [[ -f "$SCRIPT_DIR/lib.sh" ]]; then
  source "$SCRIPT_DIR/lib.sh"
else
  echo "错误: 找不到 $SCRIPT_DIR/lib.sh" >&2
  exit 1
fi

IGH_VERSION="${IGH_VERSION:-1.6.0}"
IGH_GIT_URL="${IGH_GIT_URL:-https://gitlab.com/etherlab.org/ethercat.git}"
PREFIX="${ETHERLAB_ROOT:-$(sdk_default_etherlab_prefix)}"
SRC_DIR="${IGH_SRC_DIR:-/usr/src/ethercat-${IGH_VERSION}}"
JOBS="${JOBS:-$(sdk_nproc)}"
# auto | yes | no
MODULES=auto
MODULES_ONLY=0
FORCE=0
PREFIX_EXPLICIT=0

usage() {
  cat <<EOF
用法: $0 [选项]

按本机 \`uname -r\` 探测内核头文件，能编则编 IgH 内核模块；编不了就只装用户态。

  --with-modules     必须编模块；头文件缺失则失败
  --no-modules       不编内核模块
  --modules-only     已有用户态，只针对当前内核重编模块（换内核后用）
  --force            清源码缓存并重新 configure / 编译
  --prefix=PATH      安装前缀（x86 默认 /opt/etherlab，ARM 默认 /usr/local/etherlab）

环境变量:
  IGH_VERSION        默认 1.6.0
  IGH_GIT_URL        源码仓库（默认 gitlab.com/etherlab.org/ethercat）
  IGH_SRC_DIR        源码目录（默认 /usr/src/ethercat-<ver>）
  ETHERLAB_ROOT      同 --prefix
  JOBS               并行数

换内核后:
  sudo $0 --modules-only
EOF
}

for arg in "$@"; do
  case "$arg" in
    --no-modules) MODULES=no ;;
    --with-modules) MODULES=yes ;;
    --modules-only) MODULES_ONLY=1; MODULES=yes ;;
    --force) FORCE=1 ;;
    --prefix=*) PREFIX="${arg#*=}"; PREFIX_EXPLICIT=1 ;;
    -h|--help) usage; exit 0 ;;
    *) sdk_die "未知参数: $arg（见 --help）" ;;
  esac
done

sdk_is_root || sdk_die "请使用 root 运行: sudo $0 $*"

if ! sdk_is_debian_like; then
  sdk_warn "本脚本按 Debian/Ubuntu/Raspberry Pi OS 编写，其它发行版请自行装依赖和内核头文件。"
fi

KERNEL_REL="$(sdk_kernel_release)"
KERNEL_ARCH="$(uname -m)"
KERNEL_FLAVOR="$(sdk_kernel_flavor)"
KERNEL_MM="$(sdk_kernel_major_minor)"
MODULE_DIR="/lib/modules/${KERNEL_REL}"

sdk_log "内核 ${KERNEL_REL}  (${KERNEL_ARCH}, ${KERNEL_FLAVOR}, ${KERNEL_MM})"

if sdk_in_container && [[ "$MODULES" == auto ]]; then
  sdk_warn "检测到容器，默认不编内核模块（容器内核 ≠ 编译机内核）。控机上请再跑 --with-modules。"
  MODULES=no
fi

if [[ "$KERNEL_FLAVOR" == xenomai ]]; then
  sdk_warn "当前是 Xenomai 内核。本脚本仍按 IgH 原生模块编译，不配 Xenomai RTDM；不确定请改用 --no-modules，周期线程走 IgH generic。"
fi

if ! sdk_ver_ge "$KERNEL_MM" "4.4"; then
  sdk_warn "内核 ${KERNEL_MM} 偏旧，IgH ${IGH_VERSION} 可能编不过。"
fi
if sdk_ver_ge "$KERNEL_MM" "6.16"; then
  sdk_warn "内核 ${KERNEL_MM} 较新，若 modules 编译失败可设 IGH_VERSION 试更新的 1.6.x 标签。"
fi

if [[ -d /sys/firmware/efi ]] && sdk_has_cmd mokutil; then
  if mokutil --sb-state 2>/dev/null | grep -qi 'SecureBoot enabled'; then
    sdk_warn "Secure Boot 已开启，未签名的 ec_master 可能无法加载。请关安全启动，或自行签名模块。"
  fi
fi

install_build_deps() {
  sdk_is_debian_like || return 0
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  local pkgs=(build-essential autoconf automake libtool git ca-certificates wget pkg-config kmod)
  apt-get install -y "${pkgs[@]}"
}

# 尝试安装与 uname -r 一致的头文件
install_kernel_headers() {
  local rel="$1"
  if sdk_kernel_build_dir "$rel" >/dev/null; then
    return 0
  fi
  sdk_is_debian_like || return 1

  export DEBIAN_FRONTEND=noninteractive
  local pkgs=() cand
  cand="linux-headers-${rel}"
  if apt-cache show "$cand" >/dev/null 2>&1; then
    pkgs+=("$cand")
  fi

  if [[ -r /proc/device-tree/model ]] && grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
    for cand in raspberrypi-kernel-headers linux-headers-raspi linux-raspi-headers; do
      if apt-cache show "$cand" >/dev/null 2>&1; then
        pkgs+=("$cand")
      fi
    done
  fi

  if [[ ${#pkgs[@]} -eq 0 ]]; then
    sdk_warn "apt 里没有 linux-headers-${rel}。当前内核若来自主线/自制包，请自行安装匹配的 headers。"
    return 1
  fi

  sdk_log "安装内核头文件: ${pkgs[*]}"
  if ! apt-get install -y "${pkgs[@]}"; then
    sdk_warn "内核头文件安装失败。"
    return 1
  fi
  sdk_kernel_build_dir "$rel" >/dev/null
}

resolve_kbuild() {
  local kbuild=""
  kbuild="$(sdk_kernel_build_dir "$KERNEL_REL" || true)"
  if [[ -n "$kbuild" ]]; then
    echo "$kbuild"
    return 0
  fi
  if ! install_kernel_headers "$KERNEL_REL"; then
    return 1
  fi
  sdk_kernel_build_dir "$KERNEL_REL"
}

WANT_MODULES=0
case "$MODULES" in
  yes) WANT_MODULES=1 ;;
  no) WANT_MODULES=0 ;;
  auto) WANT_MODULES=1 ;;  # 先当要编，头文件没有再降级
esac

install_build_deps

KBUILD=""
if [[ "$WANT_MODULES" -eq 1 ]]; then
  if KBUILD="$(resolve_kbuild)"; then
    sdk_log "内核头文件: $KBUILD"
  else
    if [[ "$MODULES" == yes ]]; then
      sdk_die "找不到内核 ${KERNEL_REL} 的头文件（需要 ${MODULE_DIR}/build）。装 linux-headers-${KERNEL_REL} 后再跑，或改用 --no-modules。"
    fi
    sdk_warn "无匹配头文件，跳过内核模块。控机装好 headers 后执行: sudo $0 --modules-only"
    WANT_MODULES=0
  fi
fi

if [[ "$MODULES_ONLY" -eq 1 && "$WANT_MODULES" -eq 0 ]]; then
  sdk_die "--modules-only 需要能编模块。"
fi

if [[ "$MODULES_ONLY" -eq 1 ]]; then
  [[ -f "$PREFIX/include/ecrt.h" ]] || sdk_die "--modules-only 需要已安装用户态（$PREFIX/include/ecrt.h）。先不加该参数跑一遍。"
fi

fetch_source() {
  if [[ "$FORCE" -eq 1 && -d "$SRC_DIR" ]]; then
    sdk_log " --force：删除 $SRC_DIR"
    rm -rf "$SRC_DIR"
  fi
  if [[ -d "$SRC_DIR" && ( -f "$SRC_DIR/configure" || -f "$SRC_DIR/bootstrap" ) ]]; then
    sdk_log "使用已有源码 $SRC_DIR"
    return 0
  fi
  mkdir -p "$(dirname "$SRC_DIR")"
  rm -rf "$SRC_DIR"
  sdk_log "克隆 IgH ${IGH_VERSION}  →  $SRC_DIR"
  if git clone --depth 1 --branch "$IGH_VERSION" "$IGH_GIT_URL" "$SRC_DIR"; then
    return 0
  fi
  sdk_warn "git clone 失败，改下 tarball。"
  local tar="${TMPDIR:-/tmp}/ethercat-${IGH_VERSION}.tar.gz"
  local url="https://gitlab.com/etherlab.org/ethercat/-/archive/${IGH_VERSION}/ethercat-${IGH_VERSION}.tar.gz"
  if ! wget -q -O "$tar" "$url"; then
    sdk_die "无法获取 IgH 源码。检查网络/代理，或设置 IGH_GIT_URL / 预先放到 $SRC_DIR"
  fi
  local unpack="${TMPDIR:-/tmp}/ethercat-unpack-$$"
  mkdir -p "$unpack"
  tar -xzf "$tar" -C "$unpack"
  local top
  top="$(find "$unpack" -mindepth 1 -maxdepth 1 -type d | head -1)"
  [[ -n "$top" ]] || sdk_die "tarball 内容异常"
  mv "$top" "$SRC_DIR"
  rm -rf "$unpack" "$tar"
}

fetch_source
cd "$SRC_DIR"

if [[ ! -f configure ]]; then
  [[ -x ./bootstrap ]] || sdk_die "$SRC_DIR 里没有 bootstrap/configure"
  ./bootstrap
fi

CONFIG_ARGS=(
  --prefix="$PREFIX"
  --enable-generic
  --enable-8139too=no
  --enable-cycles
  --enable-hrtimer
)
if [[ "$WANT_MODULES" -eq 1 && -n "$KBUILD" ]]; then
  CONFIG_ARGS+=(--with-linux-dir="$KBUILD")
  CONFIG_ARGS+=(--with-module-dir="$MODULE_DIR")
fi

# Intel 原厂驱动只在 x86 有意义；真控默认仍建议 DEVICE_MODULES=generic
if sdk_is_arm; then
  CONFIG_ARGS+=(--enable-e1000e=no)
else
  CONFIG_ARGS+=(--enable-e1000e)
fi

sdk_log "configure ${CONFIG_ARGS[*]}"
./configure "${CONFIG_ARGS[@]}"

if [[ "$MODULES_ONLY" -eq 0 ]]; then
  make -j"$JOBS"
  make install
else
  sdk_log "跳过用户态 make install（--modules-only）"
fi

if [[ "$WANT_MODULES" -eq 1 ]]; then
  if lsmod | grep -q '^ec_'; then
    sdk_log "卸载正在使用的 EtherCAT 模块"
    if [[ -x /etc/init.d/ethercat ]]; then
      /etc/init.d/ethercat stop || true
    fi
    # 顺序：从站驱动 → master
    for m in ec_generic ec_e1000e ec_igb ec_igc ec_8139too ec_master; do
      rmmod "$m" 2>/dev/null || true
    done
  fi
  make modules
  make modules_install
  depmod -a "${KERNEL_REL}"
  if ! modinfo ec_master >/dev/null 2>&1; then
    sdk_die "modules_install 之后仍找不到 ec_master。看上面的 make modules 输出。"
  fi
  _vermagic="$(modinfo -F vermagic ec_master 2>/dev/null || true)"
  sdk_log "ec_master vermagic: ${_vermagic}"
  if [[ -n "$_vermagic" ]] && ! echo "$_vermagic" | grep -q "^${KERNEL_REL}"; then
    sdk_warn "模块 vermagic 与当前内核 ${KERNEL_REL} 不一致，加载会失败。确认 --with-linux-dir=$KBUILD"
  fi
fi

mkdir -p /etc/sysconfig
if [[ -f "$PREFIX/etc/sysconfig/ethercat" && ! -f /etc/sysconfig/ethercat ]]; then
  cp "$PREFIX/etc/sysconfig/ethercat" /etc/sysconfig/ethercat
fi
if [[ -f "$PREFIX/etc/ethercat.conf" && ! -f /etc/ethercat.conf ]]; then
  cp "$PREFIX/etc/ethercat.conf" /etc/ethercat.conf
fi
if [[ -f "$PREFIX/etc/init.d/ethercat" ]]; then
  cp "$PREFIX/etc/init.d/ethercat" /etc/init.d/ethercat
  chmod a+x /etc/init.d/ethercat
fi
ln -sfn "$PREFIX/bin/ethercat" /usr/local/bin/ethercat

if [[ -d "$PREFIX/lib" ]]; then
  echo "$PREFIX/lib" > /etc/ld.so.conf.d/ethercat.conf
  ldconfig
fi
if [[ -d "$PREFIX/lib64" ]]; then
  echo "$PREFIX/lib64" >> /etc/ld.so.conf.d/ethercat.conf
  ldconfig
fi

RULES_SRC=""
if [[ -f "$SCRIPT_DIR/../packaging/udev/99-EtherCAT.rules" ]]; then
  RULES_SRC="$SCRIPT_DIR/../packaging/udev/99-EtherCAT.rules"
elif [[ -f /usr/share/controller-sdk/udev/99-EtherCAT.rules ]]; then
  RULES_SRC=/usr/share/controller-sdk/udev/99-EtherCAT.rules
fi
if [[ -n "$RULES_SRC" ]]; then
  cp "$RULES_SRC" /etc/udev/rules.d/99-EtherCAT.rules
  udevadm control --reload-rules 2>/dev/null || true
fi

echo
echo "=== IgH 安装完成 ==="
echo "  前缀:     $PREFIX"
echo "  版本:     $IGH_VERSION"
echo "  内核:     $KERNEL_REL  ($KERNEL_FLAVOR)"
echo "  头文件:   ${KBUILD:-（未用）}"
echo "  内核模块: $([[ $WANT_MODULES -eq 1 ]] && echo "已安装 → $MODULE_DIR" || echo 未安装)"
echo
echo "下一步（真控）:"
echo "  1. 编辑网卡 MAC（二选一，以本机实际文件为准）:"
echo "       /etc/sysconfig/ethercat"
echo "       /etc/ethercat.conf"
echo "     MASTER0_DEVICE=<网卡MAC>   DEVICE_MODULES=generic"
echo "  2. sudo /etc/init.d/ethercat start"
echo "  3. ethercat master && ethercat slaves"
echo
echo "换内核之后请再跑: sudo $0 --modules-only"
export ETHERLAB_ROOT="$PREFIX"
