#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

typedef struct SsiPsramState {
    SSIPeripheral parent_obj;
    uint32_t size_mbytes;
    uint32_t dummy;
    int command;
    int byte_count;
    /* Array representing the RAM */
    uint8_t* data;
} SsiPsramState;

#define TYPE_SSI_PSRAM "ssi_psram"
OBJECT_DECLARE_SIMPLE_TYPE(SsiPsramState, SSI_PSRAM)

