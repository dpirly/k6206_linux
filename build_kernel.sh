#!/bin/bash


#export CROSS_COMPILE=<path to cross compiler prefix> (e.g.,
#/opt/poky/1.4.1/sysroots/i686-pokysdk-linux/usr/bin/cortexa9hf-vfp-neon-poky-linuxgnueabi/

source /opt/fsl-imx-fb/4.1.15-2.1.0/environment-setup-cortexa7hf-neon-poky-linux-gnueabi


export ARCH=arm
export CROSS_COMPILER=arm-poky-linux-gnueabi-
unset LDFLAGS

make imx_v7_defconfig
make -j8 uImage LOADADDR=0x10008000

echo ""
echo "Compile device tree"
make imx6ul-k6206.dtb

echo "copy device tree"
cp ./arch/arm/boot/dts/imx6ul-k6206.dtb ./

echo "copy zImage"
cp ./arch/arm/boot/zImage ./


