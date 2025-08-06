# BSP Error Fix

## 1. Interrupt priority definition `configUNIQUE_INTERRUPT_PRIORITIES`

Compiling with default BSP:

```
In file included from /data/work/fpga/prj/germ/vitis/ZynqDetector/src/detector/Germanium/GermaniumDetector.cpp:3:
/data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h:240:6: error: #error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting. configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware
  240 |     #error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting.  configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware
      |      ^~~~~
```

Add
```c++
#pragma message("Using FreeRTOSConfig.h from [path]")
```

to each `FreeRTOSConfig.h` and comile. It confirms `./GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/FreeRTOSConfig.h` is being used.

## Solution

`/data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h` doesn't seem to have included `FreeRTOSConfig.h` before looking at `configUNIQUE_INTERRUPT_PRIORITIES`, so the branching is not expected:

```c++
224 #if configUNIQUE_INTERRUPT_PRIORITIES == 16
225     #define portPRIORITY_SHIFT 4
226     #define portMAX_BINARY_POINT_VALUE  3
227 #elif configUNIQUE_INTERRUPT_PRIORITIES == 32
228     #define portPRIORITY_SHIFT 3
229     #define portMAX_BINARY_POINT_VALUE  2
230 #elif configUNIQUE_INTERRUPT_PRIORITIES == 64
231     #define portPRIORITY_SHIFT 2
232     #define portMAX_BINARY_POINT_VALUE  1
233 #elif configUNIQUE_INTERRUPT_PRIORITIES == 128
234     #define portPRIORITY_SHIFT 1
235     #define portMAX_BINARY_POINT_VALUE  0
236 #elif configUNIQUE_INTERRUPT_PRIORITIES == 256
237     #define portPRIORITY_SHIFT 0
238     #define portMAX_BINARY_POINT_VALUE  0
239 #else
240     #error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting.  configUNIQUE_INTERRUPT_PRIORITIES must be set to the numbe    r of unique priorities implemented by the target hardware
241 #endif
```

Add:

```c++
#define configUNIQUE_INTERRUPT_PRIORITIES 32
```

for now.

Revisit later!!!

!!! After the revision, at some point the file was reverted and definition was gone! Was that caused by building the platform?
The file was shown to be modified on Aug 27  2024
$ ll /data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h
-rw-rw-r-- 1 liji liji 11070 Aug 27  2024 /data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h

Changed again:
$ ll /data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h
-rw-rw-r-- 1 liji liji 11116 Aug  6 11:56 /data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h



## 2. Support static initialization of queues and tasks `configSUPPORT_STATIC_ALLOCATION`

The default BSP doesn't support static initialization of queues and tasks. Thus `xTaskCreateStatic()` is not supported.

To support this feature, `configSUPPORT_STATIC_ALLOCATION` must be defined as 1 (0 by defult).

Edit `./GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/FreeRTOSConfig.h`, and modify the definition. Rebuild the platform.
