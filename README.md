# AV-ZeroCopy-Sensor-Middleware

[![AV Middleware CI](https://github.com)](https://github.com)

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

## Key Systems Engineering Features

1. **Zero-Copy POSIX Shared Memory Pool (`shm_open` + `mmap`)**
   * Pre-faults 256 MB of shared memory at boot to eliminate minor page faults during real-time execution, with automatic container-safe fallback when `/dev/shm` is quota-restricted.
   * Uses lock-free atomic reference counting (`compare_exchange_strong`) per memory slot so sensor producers and downstream consumers pass 64-byte descriptors rather than copying megabytes of raw frames.
2. **Cache-Line Aligned Lock-Free SPSC Ring Buffers**
   * Separates `head_` and `tail_` atomic indices onto distinct 64-byte cache lines (`alignas(64)`) to eliminate CPU false sharing across pinned producer/consumer cores.
3. **Pure C ABI Sensor Plugin Interface (`sensor_c_abi.h`)**
   * Decouples low-level kernel/V4L2/PCIe hardware driver modules from high-level C++20 perception and recording services.
4. **Multi-Sensor PTP Time Synchronization (`time_sync.hpp`)**
   * Aligns asynchronous Camera, LIDAR, and CAN/IMU streams using hardware PTP timestamps within bounded skew windows before GPU tensor assembly.
5. **CUDA Pinned-Memory / DMA GPU Handoff (`gpu_handoff.hpp`)**
   * Supports `cudaHostAllocMapped` for zero-copy GPU device pointer mapping with automatic POSIX `mlock` page-locked fallback on CPU-only development machines.
6. **Deterministic Fast-Boot DAG Orchestrator (`service_manager.hpp`)**
   * Staged dependency bring-up with microsecond initialization profiling and reverse-order graceful teardown.

---

## Quick Start (`build.sh`)

```bash
# Clean build, run unit tests, execute 5s live capture, and verify binary log
./build.sh

# Enable CUDA Pinned Memory (on GPU-equipped hosts) or Debug Sanitizers
./build.sh --cuda
./build.sh --debug --duration=2
```

---

## Verified Runtime Telemetry (Linux x86_64)

* **Full AV Stack Cold-Boot:** `0.360 ms` across 6 staged services
* **Sensor Ingest Latencies (0 Dropped Frames, 0 CRC32 Errors):**
  * `VEHICLE_CAN` (200 Hz): **0.36 µs** avg ingest latency
  * `LIDAR_128CH` (20 Hz): **8.33 µs** avg ingest latency
  * `CAMERA_4K` (30 FPS): **16.02 µs** avg ingest latency
* **Multi-Sensor PTP Synchronization:** `9.23 ms` average inter-sensor alignment skew across Camera, LIDAR, and CAN/IMU bundles

---

## JD Alignment Matrix

| NVIDIA AV Platform Requirement | Implementation in This Repository |
| :--- | :--- |
| **Sensor Integration & SAL** (Cameras, LIDAR, IMU, CAN) | `include/sal/sensors.hpp` + `include/abi/sensor_c_abi.h` |
| **Startup User Experience** | `include/runtime/service_manager.hpp` (Sub-millisecond staged bring-up) |
| **Data Processing & Zero-Copy Performance** | `include/core/shm_pool.hpp` + `include/core/lockfree_ring.hpp` |
| **Data Serialization & Fleet Collection** | `include/core/serializer.hpp` + `tools/inspect_recording.py` |
| **CPU/GPU Memory & Concurrency Architecture** | `include/core/gpu_handoff.hpp` + `pthread_setaffinity_np` core pinning |
