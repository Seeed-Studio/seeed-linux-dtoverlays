/*
 * Copyright (C) 2020 Seeed Studio
 * Peter Yang <turmary@126.com>
 *
 */
#ifndef __SPI_CAN_COMPATIBLE_H__
#define __SPI_CAN_COMPATIBLE_H__

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
#define __KER_INC_CAN_RX_OFFLOAD     1
#else
#define __KER_INC_CAN_RX_OFFLOAD     0
#endif

#if __KER_INC_CAN_RX_OFFLOAD
#include <linux/can/rx-offload.h>
#else
#include "rx-offload.h"
#endif

#endif//__SPI_CAN_COMPATIBLE_H__

