#!/usr/bin/env bash
# 脚本公共函数。由同目录其它 .sh source，不要单独执行。
# 安装后路径: /usr/share/controller-sdk/scripts/lib.sh

sdk_log() { echo "==> $*"; }
sdk_warn() { echo "警告: $*" >&2; }
sdk_die() { echo "错误: $*" >&2; exit 1; }

sdk_nproc() {
  nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4
}

sdk_normalize_arch() {
  case "$1" in
    x86_64|amd64|x64|AMD64) echo x86_64 ;;
    aarch64|arm64) echo aarch64 ;;
    armv7l|armhf|arm) echo armhf ;;
    i686|i386|x86) echo i686 ;;
    *) echo "$1" ;;
  esac
}

sdk_host_arch() { sdk_normalize_arch "$(uname -m)"; }

sdk_is_arm() {
  case "$(uname -m)" in
    aarch64|arm64|armv7l|armv6l) return 0 ;;
    *) return 1 ;;
  esac
}

sdk_default_etherlab_prefix() {
  if sdk_is_arm; then
    echo /usr/local/etherlab
  else
    echo /opt/etherlab
  fi
}

# 探测已安装的 IgH 用户态（ecrt.h）
sdk_detect_etherlab_root() {
  local p prefer="${1:-}"
  if [[ -n "$prefer" && -f "$prefer/include/ecrt.h" ]]; then
    echo "$prefer"
    return 0
  fi
  for p in \
    "${ETHERLAB_ROOT:-}" \
    "$(sdk_default_etherlab_prefix)" \
    /usr/local/etherlab \
    /opt/etherlab \
    /usr/local \
    /usr
  do
    [[ -n "$p" ]] || continue
    if [[ -f "$p/include/ecrt.h" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

sdk_is_debian_like() {
  [[ -f /etc/os-release ]] && grep -qiE 'debian|ubuntu|raspbian' /etc/os-release
}

sdk_in_container() {
  [[ -f /.dockerenv ]] && return 0
  [[ -f /run/.containerenv ]] && return 0
  if [[ -r /proc/1/cgroup ]] && grep -Eq 'docker|lxc|containerd|kubepods' /proc/1/cgroup; then
    return 0
  fi
  return 1
}

sdk_is_root() { [[ "$(id -u)" -eq 0 ]]; }

# 需要 root 的步骤：已是 root 则直接跑，否则 sudo
sdk_as_root() {
  if sdk_is_root; then
    "$@"
  else
    sudo "$@"
  fi
}

# 编译/测试：sudo 进来时交回原用户，避免 root 占用 build/ 和 /tmp
sdk_as_build_user() {
  if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
    sudo -u "$SUDO_USER" -H -- "$@"
  else
    "$@"
  fi
}

sdk_chown_build_user() {
  [[ -e "$1" ]] || return 0
  if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]] && sdk_is_root; then
    chown -R "$SUDO_USER:" "$1"
  fi
}

sdk_kernel_release() { uname -r; }

# vanilla | preempt-rt | xenomai
sdk_kernel_flavor() {
  local ver
  ver="$(uname -v 2>/dev/null || true) $(cat /proc/version 2>/dev/null || true)"
  if echo "$ver" | grep -qi xenomai; then
    echo xenomai
  elif echo "$ver" | grep -qiE 'PREEMPT[ _]RT|PREEMPT_RT'; then
    echo preempt-rt
  elif [[ -d /usr/xenomai ]] || [[ -e /dev/rtdm ]]; then
    echo xenomai
  else
    echo vanilla
  fi
}

# 当前内核的 kbuild 目录（有 Makefile 才算可用）
sdk_kernel_build_dir() {
  local rel="${1:-$(uname -r)}" c
  for c in "/lib/modules/${rel}/build" "/usr/src/linux-headers-${rel}"; do
    if [[ -f "$c/Makefile" || -f "$c/Kconfig" ]]; then
      echo "$c"
      return 0
    fi
  done
  return 1
}

# 内核主.次 版本，如 6.8
sdk_kernel_major_minor() {
  uname -r | awk -F '[.-]' '{print $1 "." $2}'
}

sdk_ver_ge() {
  # $1 >= $2 （主.次）
  printf '%s\n%s\n' "$2" "$1" | sort -V | head -1 | grep -qx "$2"
}

sdk_has_cmd() { command -v "$1" >/dev/null 2>&1; }
