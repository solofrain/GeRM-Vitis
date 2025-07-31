#pragma once

#include "queue.hpp"


//====================================
// Requests
//====================================

struct RegisterSingleAccessReqStruct : AccessReq
{
    uint32_t data;
};
using RegisterSingleAccessReq = RegisterSingleAccessReqStruct;
//-----------------------------
struct PsI2cAccessReqStruct : AccessReq
{
    uint8_t  length;
    uint8_t  addr;
    uint8_t  read;
    uint8_t  data[4];
};
using PsI2cAccessReq = PsI2cAccessReqStruct;
//-----------------------------
struct PsXadcAccessReqStruct : AccessReq
{
    uint32_t data;
};
using PsXadcAccessReq= PsXadcAccessReqStruct;
//-----------------------------
struct __attribute__((__packed__)) Ad9252CfgStruct
{
    uint16_t  chip_num;
    //uint16_t  addr;
    uint16_t  data;
};
using Ad9252Cfg = Ad9252CfgStruct;
using Ad9252AccessReq = Ad9252CfgStruct;
//-----------------------------
struct __attribute__((__packed__)) ZddmArmStruct
{
    uint16_t  mode;
    uint16_t  val;
};
using ZddmArm = ZddmArmStruct;
using ZddmAccessReq = ZddmArmStruct;
//-----------------------------
struct MarsLoadStruct
{
    uint32_t  loads[12][14];
};
using MarsLoad = MarsLoadStruct;
using MarsAccessReq = MarsLoadStruct;
//-----------------------------
union UdpRxMsgPayloadStruct
{
    RegisterSingleAccessReq  reg_single_acc_req;
    PsI2cAccessReq           psi2c_access_req;
    PsXadcAccessReq          psxadc_access_req;
    ZddmArm                  zddm_arm_data;
    Ad9252Cfg                ad9252_cfg_data;
    MarsLoad                 mars_load_data;
};
using UdpRxMsgPayload = UdpRxMsgPayloadStruct;
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
struct AccessResp
{
    uint16_t op;
};
//-----------------------------
struct RegisterSingleAccessRespStruct : AccessResp
{
    uint32_t  data;
};
using RegisterSingleAccessResp = RegisterSingleAccessRespStruct;
//-----------------------------
struct RegisterMultiAccessRespStruct : AccessResp
{
    uint32_t  data;
};
using RegisterMultiAccessResp = RegisterMultiAccessRespStruct;
//-----------------------------
struct PsI2cAccessRespStruct : AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
using PsI2cAccessResp = PsI2cAccessRespStruct;
//-----------------------------
struct PsXadcAccessRespStruct : AccessResp
{
    uint8_t  length;
    uint8_t  data[4];
};
using PsXadcAccessResp = PsXadcAccessRespStruct;
//-----------------------------
struct PlInterfaceSingleAccessRespStruct : AccessResp
{
    uint16_t  op;
    uint32_t  data;
};
using PlInterfaceSingleAccessResp = PlInterfaceSingleAccessRespStruct;
//-----------------------------
struct PlInterfaceMultiAccessRespStruct : AccessResp
{
    uint16_t op;
    uint32_t leng;
    uint32_t data[4096/4 - 1];
};
using PlInterfaceMultiAccessResp = PlInterfaceMultiAccessRespStruct;
//-----------------------------


