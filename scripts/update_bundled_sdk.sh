#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SDK_ROOT="${PACKAGE_DIR}/SDK"
readonly ROS_CONFIG_FILE="${PACKAGE_DIR}/config/OrbbecSDKConfig_v2.0.xml"

dry_run=false
work_dir=""
include_backup_dir=""
arm64_backup_dir=""
x64_backup_dir=""
config_backup_file=""

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--dry-run] <arm64-or-x86_64-sdk-directory-or.tar.gz>

Replace the bundled Orbbec SDK files used by the ROS 1 package.
The other SDK directory or archive is inferred by swapping the architecture
suffix between "_arm64" and "_x86_64" (before .tar.gz for archives).
Both architectures must be present in the same input format.
Archives are extracted into temporary directories and cleaned up on exit.

The following files are updated:
  - include/libobsensor/ -> SDK/include/libobsensor/
  - lib/libOrbbecSDK.so* -> SDK/lib/<architecture>/
  - lib/OrbbecSDKConfig-release.cmake -> SDK/lib/<architecture>/
  - lib/OrbbecSDKConfig.cmake -> SDK/lib/<architecture>/
  - lib/OrbbecSDKVersion.cmake -> SDK/lib/<architecture>/
  - lib/extensions/ -> SDK/lib/<architecture>/extensions/
  - lib/OrbbecSDKConfig.xml -> config/OrbbecSDKConfig_v2.0.xml

SDK/licenses/ is left unchanged.

Example:
  $(basename "$0") \\
    /home/user/Downloads/SDK/OrbbecSDK_v2.10.2_linux_arm64.tar.gz
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

cleanup_work_dir() {
  if [[ -n "${work_dir}" && "${work_dir}" == "${PACKAGE_DIR}"/.sdk-update.* ]]; then
    rm -rf -- "${work_dir}"
  fi
}

backup_exists() {
  [[ -n "${include_backup_dir}" && -d "${include_backup_dir}" ]] ||
    [[ -n "${arm64_backup_dir}" && -d "${arm64_backup_dir}" ]] ||
    [[ -n "${x64_backup_dir}" && -d "${x64_backup_dir}" ]] ||
    [[ -n "${config_backup_file}" && -e "${config_backup_file}" ]]
}

restore_directory() {
  local backup_path=$1
  local target_path=$2
  local failed_name=$3

  [[ -d "${backup_path}" ]] || return 0
  if [[ -e "${target_path}" ]]; then
    mv -- "${target_path}" "${work_dir}/${failed_name}" || return 1
  fi
  mv -- "${backup_path}" "${target_path}"
}

restore_backup() {
  backup_exists || return 0

  echo "Update failed; restoring the original SDK files and runtime configuration..." >&2
  if [[ -n "${config_backup_file}" && -e "${config_backup_file}" ]]; then
    if [[ -e "${ROS_CONFIG_FILE}" ]]; then
      mv -- "${ROS_CONFIG_FILE}" "${work_dir}/OrbbecSDKConfig.failed.xml" || return 1
    fi
    mv -- "${config_backup_file}" "${ROS_CONFIG_FILE}" || return 1
  fi

  restore_directory "${x64_backup_dir}" "${SDK_ROOT}/lib/x64" x64.failed || return 1
  restore_directory "${arm64_backup_dir}" "${SDK_ROOT}/lib/arm64" arm64.failed || return 1
  restore_directory "${include_backup_dir}" "${SDK_ROOT}/include" include.failed
}

on_exit() {
  local status=$?
  trap - EXIT

  if ((status != 0)) && backup_exists; then
    if ! restore_backup; then
      echo "Error: automatic rollback failed; recovery files remain in ${work_dir}" >&2
      exit "${status}"
    fi
  fi

  if ! backup_exists; then
    cleanup_work_dir
  fi
  exit "${status}"
}

trap on_exit EXIT
trap 'exit 130' INT TERM HUP

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=true
  shift
fi

[[ $# -eq 1 ]] || {
  usage >&2
  exit 2
}

for command_name in basename cat cmp cp diff dirname file find mkdir mktemp mv readlink rm wc; do
  command -v "${command_name}" >/dev/null 2>&1 || die "required command not found: ${command_name}"
done

[[ -d "${SDK_ROOT}" && ! -L "${SDK_ROOT}" ]] ||
  die "bundled SDK must be a real directory: ${SDK_ROOT}"
[[ -d "${SDK_ROOT}/include" && ! -L "${SDK_ROOT}/include" ]] ||
  die "bundled SDK include directory must be a real directory: ${SDK_ROOT}/include"
[[ -d "${SDK_ROOT}/lib/arm64" && ! -L "${SDK_ROOT}/lib/arm64" ]] ||
  die "bundled ARM64 SDK must be a real directory: ${SDK_ROOT}/lib/arm64"
[[ -d "${SDK_ROOT}/lib/x64" && ! -L "${SDK_ROOT}/lib/x64" ]] ||
  die "bundled x86-64 SDK must be a real directory: ${SDK_ROOT}/lib/x64"
[[ -d "${SDK_ROOT}/licenses" && ! -L "${SDK_ROOT}/licenses" ]] ||
  die "bundled SDK licenses must be a real directory: ${SDK_ROOT}/licenses"
[[ -f "${ROS_CONFIG_FILE}" && ! -L "${ROS_CONFIG_FILE}" ]] ||
  die "ROS SDK configuration must be a real file: ${ROS_CONFIG_FILE}"

archive_suffix=""
if [[ -d "$1" ]]; then
  :
elif [[ -f "$1" && "$1" == *.tar.gz ]]; then
  archive_suffix=".tar.gz"
  command -v tar >/dev/null 2>&1 || die "required command not found: tar"
else
  die "SDK directory or .tar.gz archive not found: $1"
fi
input_source="$(readlink -f -- "$1")"
input_source="${input_source%"${archive_suffix}"}"

case "${input_source}" in
  *_arm64)
    arm64_source="${input_source}"
    x64_source="${input_source%_arm64}_x86_64"
    ;;
  *_x86_64)
    x64_source="${input_source}"
    arm64_source="${input_source%_x86_64}_arm64"
    ;;
  *)
    die "SDK name must end with _arm64 or _x86_64 (before .tar.gz): ${input_source}"
    ;;
esac

if [[ -n "${archive_suffix}" ]]; then
  [[ -f "${arm64_source}${archive_suffix}" ]] ||
    die "paired ARM64 SDK archive not found: ${arm64_source}${archive_suffix}"
  [[ -f "${x64_source}${archive_suffix}" ]] ||
    die "paired x86-64 SDK archive not found: ${x64_source}${archive_suffix}"
else
  [[ -d "${arm64_source}" ]] || die "paired ARM64 SDK directory not found: ${arm64_source}"
  [[ -d "${x64_source}" ]] || die "paired x86-64 SDK directory not found: ${x64_source}"
fi
case "${arm64_source}" in
  "${SDK_ROOT}" | "${SDK_ROOT}"/*) die "the ARM64 source cannot be inside ${SDK_ROOT}" ;;
esac
case "${x64_source}" in
  "${SDK_ROOT}" | "${SDK_ROOT}"/*) die "the x86-64 source cannot be inside ${SDK_ROOT}" ;;
esac

core_library_path() {
  local library_dir=$1
  local core_link="${library_dir}/libOrbbecSDK.so"

  [[ -e "${core_link}" ]] || die "missing core SDK library: ${core_link}"
  readlink -f -- "${core_link}"
}

sdk_version() {
  local core_library
  core_library="$(core_library_path "$1/lib")"
  local filename=${core_library##*/}

  [[ "${filename}" == libOrbbecSDK.so.* ]] ||
    die "cannot determine SDK version from core library: ${core_library}"
  echo "${filename#libOrbbecSDK.so.}"
}

verify_library_architecture() {
  local library_dir=$1
  local expected_arch=$2
  local core_library
  local description
  core_library="$(core_library_path "${library_dir}")"
  description="$(file -Lb -- "${core_library}")"

  case "${expected_arch}" in
    arm64)
      [[ "${description}" == *"ARM aarch64"* || "${description}" == *"AArch64"* ]] ||
        die "${library_dir} is not an ARM64 SDK (${description})"
      ;;
    x64)
      [[ "${description}" == *"x86-64"* ]] ||
        die "${library_dir} is not an x86-64 SDK (${description})"
      ;;
    *)
      die "internal error: unsupported architecture ${expected_arch}"
      ;;
  esac
}

verify_symlinks_within() {
  local root_dir
  local link_path
  local resolved_path
  root_dir="$(readlink -f -- "$1")"

  while IFS= read -r -d '' link_path; do
    if ! resolved_path="$(readlink -f -- "${link_path}")"; then
      die "broken symbolic link: ${link_path}"
    fi
    case "${resolved_path}" in
      "${root_dir}"/*) ;;
      *) die "symbolic link points outside ${root_dir}: ${link_path} -> ${resolved_path}" ;;
    esac
  done < <(find "${root_dir}" -type l -print0)
}

verify_source_layout() {
  local source_dir=$1
  local expected_arch=$2
  local required_path

  for required_path in \
    include/libobsensor/ObSensor.h \
    include/libobsensor/ObSensor.hpp \
    lib/OrbbecSDKConfig-release.cmake \
    lib/OrbbecSDKConfig.cmake \
    lib/OrbbecSDKVersion.cmake \
    lib/OrbbecSDKConfig.xml; do
    [[ -f "${source_dir}/${required_path}" ]] ||
      die "missing required ${expected_arch} SDK file: ${source_dir}/${required_path}"
  done

  [[ -d "${source_dir}/lib/extensions" ]] ||
    die "missing ${expected_arch} SDK extensions directory: ${source_dir}/lib/extensions"

  verify_library_architecture "${source_dir}/lib" "${expected_arch}"
  verify_symlinks_within "${source_dir}/lib"
}

copy_architecture_libraries() {
  local source_dir=$1
  local target_arch=$2
  local destination_dir=$3
  local cmake_file
  local copied_core_library=false

  mkdir -p -- "${destination_dir}"

  while IFS= read -r -d '' library; do
    cp -a -- "${library}" "${destination_dir}/"
    copied_core_library=true
  done < <(find "${source_dir}/lib" -maxdepth 1 \( -type f -o -type l \) \
    -name 'libOrbbecSDK.so*' -print0)
  [[ "${copied_core_library}" == true ]] ||
    die "no core SDK libraries were copied for ${target_arch}"

  for cmake_file in \
    OrbbecSDKConfig-release.cmake \
    OrbbecSDKConfig.cmake \
    OrbbecSDKVersion.cmake; do
    cp -a -- "${source_dir}/lib/${cmake_file}" "${destination_dir}/"
  done

  cp -a -- "${source_dir}/lib/extensions" "${destination_dir}/"
}

extract_sdk_archive() {
  local archive=$1
  local destination=$2
  local expected_root=${archive##*/}
  expected_root=${expected_root%.tar.gz}

  mkdir -p -- "${destination}" || return 1
  tar --extract --gzip --file="${archive}" --directory="${destination}" --no-same-owner || return 1
  # Official archives contain a directory named after the archive. Also accept
  # archives with include/ and lib/ directly at the root.
  if [[ -d "${destination}/${expected_root}" ]]; then
    printf '%s\n' "${destination}/${expected_root}"
  else
    printf '%s\n' "${destination}"
  fi
}

work_dir="$(mktemp -d -- "${PACKAGE_DIR}/.sdk-update.XXXXXX")"
if [[ -n "${archive_suffix}" ]]; then
  arm64_source="$(extract_sdk_archive "${arm64_source}${archive_suffix}" "${work_dir}/source-arm64")" ||
    die "failed to extract ARM64 SDK archive"
  x64_source="$(extract_sdk_archive "${x64_source}${archive_suffix}" "${work_dir}/source-x64")" ||
    die "failed to extract x86-64 SDK archive"
fi

verify_source_layout "${arm64_source}" arm64
verify_source_layout "${x64_source}" x64

arm64_version="$(sdk_version "${arm64_source}")"
x64_version="$(sdk_version "${x64_source}")"
[[ "${arm64_version}" == "${x64_version}" ]] ||
  die "SDK versions do not match: ARM64=${arm64_version}, x86-64=${x64_version}"

diff -qr -- "${arm64_source}/include/libobsensor" "${x64_source}/include/libobsensor" \
  >/dev/null || die "ARM64 and x86-64 libobsensor headers do not match"
cmp -s -- "${arm64_source}/lib/OrbbecSDKConfig.xml" \
  "${x64_source}/lib/OrbbecSDKConfig.xml" ||
  die "ARM64 and x86-64 OrbbecSDKConfig.xml files do not match"

staged_sdk="${work_dir}/SDK.new"
staged_config="${work_dir}/OrbbecSDKConfig_v2.0.xml.new"
mkdir -p -- "${staged_sdk}/include" "${staged_sdk}/lib"

cp -a -- "${arm64_source}/include/libobsensor" "${staged_sdk}/include/"
copy_architecture_libraries "${arm64_source}" arm64 "${staged_sdk}/lib/arm64"
copy_architecture_libraries "${x64_source}" x64 "${staged_sdk}/lib/x64"
cp -a -- "${x64_source}/lib/OrbbecSDKConfig.xml" "${staged_config}"

verify_library_architecture "${staged_sdk}/lib/arm64" arm64
verify_library_architecture "${staged_sdk}/lib/x64" x64
verify_symlinks_within "${staged_sdk}"

sdk_file_count="$(find "${staged_sdk}" \( -type f -o -type l \) | wc -l)"
file_count=$((sdk_file_count + 1))
echo "Validated Orbbec SDK ${arm64_version} for ARM64 and x86-64."
echo "Staged ${file_count} ROS 1 package files, including ${ROS_CONFIG_FILE}."

if [[ "${dry_run}" == true ]]; then
  echo "Dry run complete; SDK files, licenses, and runtime configuration were not changed."
  exit 0
fi

include_backup_dir="${work_dir}/include.old"
arm64_backup_dir="${work_dir}/arm64.old"
x64_backup_dir="${work_dir}/x64.old"
config_backup_file="${work_dir}/OrbbecSDKConfig_v2.0.xml.old"
mv -- "${SDK_ROOT}/include" "${include_backup_dir}"
mv -- "${SDK_ROOT}/lib/arm64" "${arm64_backup_dir}"
mv -- "${SDK_ROOT}/lib/x64" "${x64_backup_dir}"
mv -- "${ROS_CONFIG_FILE}" "${config_backup_file}"
mv -- "${staged_sdk}/include" "${SDK_ROOT}/include"
mv -- "${staged_sdk}/lib/arm64" "${SDK_ROOT}/lib/arm64"
mv -- "${staged_sdk}/lib/x64" "${SDK_ROOT}/lib/x64"
mv -- "${staged_config}" "${ROS_CONFIG_FILE}"

# The verified SDK files and runtime configuration are now active. Mark the
# transaction as committed; the exit handler removes the temporary backups.
include_backup_dir=""
arm64_backup_dir=""
x64_backup_dir=""
config_backup_file=""

echo "Updated the bundled ROS 1 SDK files and runtime configuration to Orbbec SDK ${arm64_version}."
echo "Left ${SDK_ROOT}/licenses unchanged."
