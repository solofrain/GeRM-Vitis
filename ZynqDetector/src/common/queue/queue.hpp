#pragma once

#include <cstdint>
#include <type_traits>  // for std::is_same

class PsI2c;
//class PlI2c;

//====================================
// Requests
//====================================
struct AccessReq
{
    uint16_t op;
};

const uint16_t WRITE = 0;
const uint16_t READ  = 1;

//====================================
// Responses
//====================================
struct AccessResp
{
    uint16_t op;
};

