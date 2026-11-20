# AV-ZeroCopy-Sensor-Middleware

**A deterministic, zero-copy C++20/C sensor abstraction, PTP time-synchronization, and fleet data-recording middleware engine engineered for heterogeneous Autonomous Vehicle (AV) compute platforms.**

---

## Architectural Overview

```text
+---------------------------------------------------------------------------------------------------+
|                             SENSOR ABSTRACTION LAYER (SAL - C/C++20 ABI)                          |
|  +-----------------------+       +-----------------------+       +-----------------------------+  |
|  |  Front Camera (4K)    |       |  Roof LIDAR (128-Ch)  |       |  Vehicle CAN / IMU (200Hz)  |  |
|  |  512 KB @ 30 FPS      |       |  256 KB @ 20 Hz       |       |  256 B @ 200 Hz             |  |
|  +-----------+-----------+       +-----------+-----------+       +--------------+--------------+  |
|              |                               |                                  |                 |
|              +-------------------------------+----------------------------------+                 |
|                                              | Direct Zero-Copy Payload Write                     |
|                                              v                                                    |
|  +---------------------------------------------------------------------------------------------+  |
|  |             POSIX SHARED MEMORY POOL (/av_drive_sensor_pool - 256 MB Pre-Faulted)           |  |
|  |   [ Slot 0 ]  [ Slot 1 ]  [ Slot 2 ]  ...  [ Slot 255 ]   (Atomic Lock-Free Ref-Counting)   |  |
|  +-------------------------------------------+-------------------------------------------------+  |
+----------------------------------------------|----------------------------------------------------+
                                               | 64-Byte Cache-Aligned Descriptors (SPSC Ring)
                                               v
+---------------------------------------------------------------------------------------------------+
|                        REAL-TIME FUSION, GPU HANDOFF & FLEET RECORDER ENGINE                      |
|  +---------------------------------+     +-----------------------------------------------------+  |
|  |   PTP Timestamp Synchronizer    |---->|  GPU Zero-Copy Handoff (cudaHostAlloc / mlock DMA)  |  |
|  |   (Nearest-Neighbor Skew Check) |     +-----------------------------------------------------+  |
|  +---------------------------------+                                                              |
|                                    +-----------------------------------------------------------+  |
|                                    |  Binary Fleet Recorder (CRC32 + Zero-Alloc Packet Stream) |  |
|                                    +-----------------------------+-----------------------------+  |
+------------------------------------------------------------------|--------------------------------+
                                                                   v
                                                     [ av_fleet_capture.bin ]
                                                                   |
                                                                   v
                                                   [ tools/inspect_recording.py ]
```

---

## Build & Run Instructions (Linux)

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# 1. Run automated verification unit tests
ctest --output-on-failure

# 2. Run the live AV Middleware node for 5 seconds
./av_middleware_node 5

# 3. Inspect the recorded binary fleet stream with the Python CLI tool
python3 ../tools/inspect_recording.py av_fleet_capture.bin
```
