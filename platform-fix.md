# BSP Error Fix

# Error:

```
In file included from /data/work/fpga/prj/germ/vitis/ZynqDetector/src/detector/Germanium/GermaniumDetector.cpp:3:
/data/work/fpga/prj/germ/vitis/GeRM-192-384/export/GeRM-192-384/sw/freertos_ps7_cortexa9_0/include/portmacro.h:240:6: error: #error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting. configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware
  240 |     #error Invalid configUNIQUE_INTERRUPT_PRIORITIES setting.  configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware
      |      ^~~~~
```

- This occurs when the `configUNIQUE_INTERRUPT_PRIORITIES` value doesn't match the number of bits used for interrupt priority by the processor's GIC (Generic Interrupt Controller).

- The ARM Cortex-A9 (like in Zynq-7000) typically supports 256 priority levels, but only the upper 4 bits are implemented in practice. That means:

```
#define configUNIQUE_INTERRUPT_PRIORITIES    16
```

## Solution (ChatGPT):

- Edit `./GeRM-192-384/ps7_cortexa9_0/freertos_ps7_cortexa9_0/bsp/include/FreeRTOSConfig.h`, change `configUNIQUE_INTERRUPT_PRIORITIES` from `32` to `16`.

- Rebuild Platform and then rebuild App.
