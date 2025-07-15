#pragma once

#include <cstdint>

#include "FreeRTOS.h"

#include "PLInterface.hpp"

//=========================================
// SPI Interface class
//=========================================
class PLSPI : public PLInterface
{
public:
    using PLInterface::PLInterface;
};
//=========================================
