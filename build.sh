#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
build_type=
build_args=()

for arg in "$@"; do
    case "$arg" in
        -h|--help)
            cat <<'EOF'
Usage: ./build.sh [--debug | --release] [CMake build arguments]

Put your built MOOS-IvP checkout's bin directory on PATH.
Set MOOS_IVP_ROOT to select a checkout explicitly. The adjacent moos-ivp is also searched.
Set MAVMOOS_MAVLINK_ARCHIVE to a local copy of the pinned headers for an offline build.
The executable and plugins are built in build/bin and build/plugins.

Examples: ./build.sh -j4
          ./build.sh --debug --target iMAVLink
EOF
            exit 0
            ;;
        -d|--debug) build_type=Debug ;;
        -r|--release) build_type=Release ;;
        *) build_args+=("$arg") ;;
    esac
done

configure=(-S "$root" -B "$root/build")
if [[ -n "${MOOS_IVP_ROOT:-}" ]]; then
    moos_root=$(cd "$MOOS_IVP_ROOT" && pwd -P)
    configure+=("-DMOOS_IVP_ROOT=$moos_root")
fi
if [[ ! -f "$root/build/CMakeCache.txt" ]]; then
    configure+=(-G Ninja)
fi
if [[ -n "$build_type" ]]; then
    configure+=("-DCMAKE_BUILD_TYPE=$build_type")
fi
if [[ -n "${MAVMOOS_MAVLINK_ARCHIVE:-}" ]]; then
    archive_dir=$(cd "$(dirname "$MAVMOOS_MAVLINK_ARCHIVE")" && pwd -P)
    configure+=("-DMAVMOOS_MAVLINK_ARCHIVE=$archive_dir/$(basename "$MAVMOOS_MAVLINK_ARCHIVE")")
fi

cmake "${configure[@]}"
cmake --build "$root/build" ${build_args[@]+"${build_args[@]}"}
