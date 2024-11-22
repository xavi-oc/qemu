#ifndef HW_SENSOR_MQ131_H
#define HW_SENSOR_MQ131_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"

#define TYPE_MQ131 "mq131"
#define ADC_RESOLUTION 0x0FFF

typedef struct {
    SysBusDevice parent_obj;
    uint16_t value;
    MemoryRegion iomem;
    MemoryRegion userSpace;
} mq131State;

DECLARE_INSTANCE_CHECKER(mq131State, MQ131, TYPE_MQ131)

#endif