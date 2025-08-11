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

- Add the following lines to `ZynqDetector/src/UserConfig.cmake` (replace the empty list):


```
set(USER_INCLUDE_DIRECTORIES
${CMAKE_CURRENT_SOURCE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/common/base
${CMAKE_CURRENT_SOURCE_DIR}/common/concepts
${CMAKE_CURRENT_SOURCE_DIR}/common/log
${CMAKE_CURRENT_SOURCE_DIR}/common/queue
${CMAKE_CURRENT_SOURCE_DIR}/common/task_wrap
${CMAKE_CURRENT_SOURCE_DIR}/device/Adc/Ad9252
${CMAKE_CURRENT_SOURCE_DIR}/device/Adc/Ltc2309
${CMAKE_CURRENT_SOURCE_DIR}/device/Dac/Dac7678
${CMAKE_CURRENT_SOURCE_DIR}/device/I2cDevice
${CMAKE_CURRENT_SOURCE_DIR}/device/Mars
${CMAKE_CURRENT_SOURCE_DIR}/device/Zddm
${CMAKE_CURRENT_SOURCE_DIR}/device/Temperature/Tmp100
${CMAKE_CURRENT_SOURCE_DIR}/device/Network
${CMAKE_CURRENT_SOURCE_DIR}/device/Zynq
${CMAKE_CURRENT_SOURCE_DIR}/device/Zynq/PsI2c
${CMAKE_CURRENT_SOURCE_DIR}/device/Zynq/PsXadc
${CMAKE_CURRENT_SOURCE_DIR}/device/Zynq/Register
${CMAKE_CURRENT_SOURCE_DIR}/detector/Germanium
)

set(USER_COMPILE_SOURCES
"detector_main.cpp"
"common/log/Logger.cpp"
"device/Zynq/Register/Register.cpp"
"device/Zynq/PsI2c/PsI2c.cpp"
"device/Zynq/PsXadc/PsXadc.cpp"
"detector/Germanium/GermaniumDetector.cpp"
"detector/Germanium/GermaniumNetwork.cpp"
"detector/Germanium/GermaniumZynq.cpp"
)
```

- Add the following lines to `CMakeList.txt` to support C++20 support

```
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

```

## 2. Build the app

Execution of `build-app.py` takes long time as Vitis need to start a server and prepare for compilation. To save time:

- Run `vitis -s` to start Vitis shell.
- Run `exec(open("build-app.py").read())` to prepare compilation, including starting a Vitis server. This will also run compliation once.
- If errors encounted, revise the code, and run `app.build()`.
