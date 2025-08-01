#pragma once

#include "queue.hpp"


//====================================
// Requests
//====================================

//-----------------------------
template<typename T>
struct AccessReqTypeSelector;

template<>
struct AccessReqTypeSelector<PsI2c>
{
    using type = PsI2cAccessReq;
};

//template<>
//struct AccessReqTypeSelector<PlI2c> {
//    using type = PlI2cAccessReq;
//};

//====================================
// Responses
//====================================
////-----------------------------
//struct PlInterfaceSingleAccessResp
//{
//    uint16_t  op;
//    uint32_t  data;
//};
////-----------------------------
//struct PlInterfaceMultiAccessResp
//{
//    uint16_t op;
//    uint32_t leng;
//    uint32_t data[4096/4 - 1];
//};
//-----------------------------


