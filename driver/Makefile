# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2024 Arm Technology (China) Co. Ltd.

MODULE_NAME := aipu
SRC_DIR := .
ccflags-y += -DKMD_VERSION=\"$(COMPASS_DRV_BTENVAR_KMD_VERSION)\" \
                   -I$(M)/armchina-npu/ -I$(M)/armchina-npu/include -I$(M)/armchina-npu/zhouyi \
                   -D$(BUILD_AIPU_VERSION_KMD)

COMM_OBJ := armchina-npu/aipu.o \
            armchina-npu/aipu_common.o \
            armchina-npu/aipu_io.o \
            armchina-npu/aipu_irq.o \
            armchina-npu/aipu_job_manager.o \
            armchina-npu/aipu_mm.o \
            armchina-npu/aipu_dma_buf.o \
            armchina-npu/aipu_priv.o \
            armchina-npu/aipu_tcb.o \
            armchina-npu/zhouyi/zhouyi.o

ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_V1)
    AIPU_OBJ := armchina-npu/zhouyi/v1.o \
                armchina-npu/zhouyi/v1v2_priv.o
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V1
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_V2)
    AIPU_OBJ := armchina-npu/zhouyi/v2.o \
                armchina-npu/zhouyi/v1v2_priv.o
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V2
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_V3)
    AIPU_OBJ := armchina-npu/zhouyi/v3.o \
                armchina-npu/zhouyi/v3_priv.o
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V3
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_V3_1)
    AIPU_OBJ := armchina-npu/zhouyi/v3_1.o \
                armchina-npu/zhouyi/v3_1_priv.o
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V3_1
else
    #build all
    AIPU_OBJ := armchina-npu/zhouyi/v1.o \
                armchina-npu/zhouyi/v2.o \
                armchina-npu/zhouyi/v3.o \
                armchina-npu/zhouyi/v3_1.o \
                armchina-npu/zhouyi/v1v2_priv.o \
                armchina-npu/zhouyi/v3_priv.o \
                armchina-npu/zhouyi/v3_1_priv.o
    ccflags-y += -DCONFIG_ARMCHINA_NPU_ARCH_V1 \
                    -DCONFIG_ARMCHINA_NPU_ARCH_V2 \
                    -DCONFIG_ARMCHINA_NPU_ARCH_V3 \
                    -DCONFIG_ARMCHINA_NPU_ARCH_V3_1
endif

ifeq ($(BUILD_TARGET_PLATFORM_KMD), BUILD_PLATFORM_R329)
    INIT_OBJ  := armchina-npu/r329/r329.o
else ifeq ($(BUILD_TARGET_PLATFORM_KMD), BUILD_PLATFORM_SKY1)
    INIT_OBJ := armchina-npu/sky1/sky1.o
    ccflags-y += -DCONFIG_SKY1
else ifeq ($(BUILD_TARGET_PLATFORM_KMD), BUILD_PLATFORM_SKY1_ANDROID)
    INIT_OBJ := armchina-npu/sky1/sky1.o
    ccflags-y += -DCONFIG_SKY1
    ccflags-y += -DCONFIG_ANDROID
else ifeq ($(BUILD_TARGET_PLATFORM_KMD), BUILD_PLATFORM_SKY1_ANDROID_DFT)
    INIT_OBJ := armchina-npu/default/default.o
    ccflags-y += -DCONFIG_ANDROID
else
    INIT_OBJ := armchina-npu/default/default.o
endif

ifneq ($(BUILD_NPU_DEVFREQ), n)
    ccflags-y  += -DCONFIG_ENABLE_DEVFREQ
endif

OBJS     := $(COMM_OBJ) $(AIPU_OBJ) $(SOC_OBJ)

ifneq ($(KERNELRELEASE),)
	obj-m := $(MODULE_NAME).o
	$(MODULE_NAME)-objs := $(INIT_OBJ) $(OBJS)
else
	COMPASS_DRV_BTENVAR_KPATH ?= /lib/modules/`uname -r`/build
	PWD :=$(shell pwd)

all:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) modules
clean:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) clean
endif
