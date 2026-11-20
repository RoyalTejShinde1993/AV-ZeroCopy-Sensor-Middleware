#!/usr/bin/env bash
set -euo pipefail

# Default configuration
BUILD_TYPE="Release"
ENABLE_CUDA="OFF"
RUN_DURATION=5
CLEAN_FIRST=true

# Parse optional CLI arguments
for arg in "$@"; do
    case $arg in
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --cuda)
            ENABLE_CUDA="ON"
            ;;
        --no-clean)
            CLEAN_FIRST=false
            ;;
        --duration=*)
            RUN_DURATION="${arg#*=}"
            ;;
        --help|-h)
            echo "Usage: ./build.sh [--debug] [--cuda] [--no-clean] [--duration=<seconds>]"
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $arg"
            echo "Usage: ./build.sh [--debug] [--cuda] [--no-clean] [--duration=<seconds>]"
            exit 1
            ;;
    esac
done

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "================================================================"
echo "  AV-ZeroCopy-Sensor-Middleware Build & Verification Runner"
echo "  Mode: ${BUILD_TYPE} | CUDA: ${ENABLE_CUDA} | Duration: ${RUN_DURATION}s"
echo "================================================================"

if [ "$CLEAN_FIRST" = true ]; then
    echo "[1/5] Cleaning previous build artifacts and binary captures..."
    rm -rf "${BUILD_DIR}" "${PROJECT_ROOT}"/*.bin
fi

echo "[2/5] Configuring CMake (${BUILD_TYPE})..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DAV_ENABLE_CUDA="${ENABLE_CUDA}" ..

echo "[3/5] Compiling with $(nproc) CPU cores..."
make -j"$(nproc)"

echo "[4/5] Running C++ unit test suite (ctest)..."
ctest --output-on-failure

echo "[5/5] Running live AV Middleware Node (${RUN_DURATION}s) & Python Log Analyzer..."
./av_middleware_node "${RUN_DURATION}"
python3 "${PROJECT_ROOT}/tools/inspect_recording.py" av_fleet_capture.bin

echo "================================================================"
echo "  [SUCCESS] Build, CTest, and Fleet Log Verification Complete!"
echo "================================================================"
