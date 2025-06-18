# Using the repository

This repository contains the scripts and source files to create FreeRTOS platform `GeRM-192-384` platform and FreeRTOS app `ZynqDetector`.

## 1. Prepration

- Use `create-platform.py` to create the platform.
- Use `build-platform.py` to build the platform.
- Use `create-app.py` to create the app.
- Use `build-app.py` to build the app.
- Copy the source files/folders to `ZynqDetector/src/`, including:
  - `ZynqDetector/src/detector_main.cpp`
  - `ZynqDetector/src/common/`
  - `ZynqDetector/src/device/`
  - `ZynqDetector/src/detector/`

- Copy the following content to `ZynqDetector/src/UserConfig.cmake` (replace the empty list):

```
set(USER_INCLUDE_DIRECTORIES
${CMAKE_CURRENT_SOURCE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/common/base
${CMAKE_CURRENT_SOURCE_DIR}/common/log
${CMAKE_CURRENT_SOURCE_DIR}/common/queue
${CMAKE_CURRENT_SOURCE_DIR}/common/task_wrap
${CMAKE_CURRENT_SOURCE_DIR}/device/I2C/ADC
${CMAKE_CURRENT_SOURCE_DIR}/device/I2C/DAC
${CMAKE_CURRENT_SOURCE_DIR}/device/I2C/I2CDevice
${CMAKE_CURRENT_SOURCE_DIR}/device/I2C/TMP100
${CMAKE_CURRENT_SOURCE_DIR}/device/Network
${CMAKE_CURRENT_SOURCE_DIR}/device/ZYNQ/I2CBus
${CMAKE_CURRENT_SOURCE_DIR}/device/ZYNQ/PSI2C
${CMAKE_CURRENT_SOURCE_DIR}/device/ZYNQ/PSXADC
${CMAKE_CURRENT_SOURCE_DIR}/device/ZYNQ/Register
${CMAKE_CURRENT_SOURCE_DIR}/detector/Germanium
)

set(USER_COMPILE_SOURCES
detector_main.cpp
#"common/log/Logger.cpp"
#"common/task_wrap/task_wrap.cpp"
#"device/I2C/ADC/LTC2309.cpp"
#"device/I2C/DAC/DAC7678.cpp"
#"device/I2C/I2CDevice/I2CDevice.cpp"
#"device/I2C/TMP100/TMP100.cpp"
#"device/ZYNQ/I2CBus/I2CBus.cpp"
#"device/ZYNQ/PSI2C/PSI2C.cpp"
#"device/ZYNQ/PSXADC/PSXADC.cpp"
#"device/ZYNQ/Zynq.cpp"
#"detector/Germanium/GermaniumDetector.cpp"
#"detector/Germanium/GermaniumNetwork.cpp"
#"detector/Germanium/GermaniumRegister.cpp"
#"detector/Germanium/GermaniumZYNQ.cpp"
)
```


## 2. Build the app

Execution of `build-app.py` takes long time as Vitis need to start a server and prepare for compilation. To save time:

- Run `vitis -s` to start Vitis shell.
- Run `exec(open("build-app.py").read())` to prepare compilation, including starting a Vitis server. This will also run compliation once.
- If errors encounted, revise the code, and run `app.build()`.