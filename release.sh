#!/bin/bash
#
JOBS=2
let JOBS=${JOBS}*2
JOBS="-j${JOBS}"
BOOT_PATH="arch/arm/boot"
ZIMAGE="zImage"

INPUT_STR=${1}

#export CROSS_COMPILE="/home/chulspro/sbs/tools/usr/bin/arm-linux-gnueabi-"
#export LD_LIBRARY_PATH="/home/chulspro/sbs/tools/usr/lib:/home/chulspro/sbs/tools/usr/i486-linux-gnu/arm-linux-gnueabi/lib"

make ARCH=arm ${INPUT_STR}_defconfig
if [ "$?" != "0" ]; then
	echo "Failed to make defconfig"
	exit 1
fi

make $JOBS zImage ARCH=arm
if [ "$?" != "0" ]; then
        echo "Failed to make zImage"
        exit 1
fi

#make ARCH=arm dtbs
#if [ "$?" != "0" ]; then
#        echo "Failed to make dtbs"
#        exit 1
#fi

#cat $BOOT_PATH/$ZIMAGE $BOOT_PATH/dts/exynos4212-tizenw.dtb > zImage

#tar cvf $BOOT_PATH/tizenw-linux-v3-10.tar $ZIMAGE
tar cvf System-${1}.tar -C $BOOT_PATH $ZIMAGE

