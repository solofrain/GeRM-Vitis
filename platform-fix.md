# BSP Error Fix

# Error:

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
