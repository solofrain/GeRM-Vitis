#pragma once

#include <cstdint>

#include "FreeRTOS.h"

#include "PLInterface.hpp"


//=========================================
// I2C Interface class
//=========================================
class PLI2C: public PLInterface
{
public:
    using PLInterface::PLInterface;
};
//=========================================

