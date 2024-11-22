
#include "qemu/osdep.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sensor/mq131.h"

static void mq131_reset(DeviceState *dev) {
    mq131State *s = MQ131(dev);
    s->value = 0;
}

static uint64_t mq131_read(void *opaque, hwaddr addr, unsigned size) {
    mq131State *s = MQ131(opaque);
    s->value = rand() & ADC_RESOLUTION;
    // qemu_log("mq131_read is being executed\n");
    return s->value;
}

static const MemoryRegionOps mq131_ops = {
    .read = mq131_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mq131_instance_init (Object *obj) {
    mq131State *s = MQ131(obj);
    s->value = 0;
    memory_region_init_io(&s->iomem, OBJECT(s), &mq131_ops, s, TYPE_MQ131, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    

}
static void mq131_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = mq131_reset;
    // dc->realize = mq131_realize;
}

static void mq131_register_types(void) {

    static const TypeInfo mq131_info = {
        .name = TYPE_MQ131,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(mq131State),
        .instance_init = mq131_instance_init,
        .class_init = mq131_class_init,
    };

    type_register_static(&mq131_info);
}

type_init(mq131_register_types);