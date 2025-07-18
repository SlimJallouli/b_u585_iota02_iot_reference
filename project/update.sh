#!/bin/bash

# Usage: ./cleanup.sh stm32h5xx

# if [ -z "$1" ]; then
#     echo "Usage: $0 <stm32 series, e.g., stm32h5xx>"
#     exit 1
# fi

STM32_SERIES="$1"
HOME=~
mbedTLS_VERSON="3.1.1"
mbedTLS_source="$HOME/STM32Cube/Repository/Packs/ARM/mbedTLS/$mbedTLS_VERSON/library/"
mbedTLS_destination="./Middlewares/Third_Party/ARM_Security/"
LwIP_destination="./Middlewares/Third_Party/lwIP_Network_lwIP/"


echo "Home : $HOME"

if [ -n "$STM32_SERIES" ]; then
    echo "Using STM32 series: $STM32_SERIES"

    echo "Deleting STM32 series-specific files for $STM32_SERIES..."

    FILE_PATH="./Core/Inc/${STM32_SERIES}_hal_conf.h"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Inc/${STM32_SERIES}_it.h"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Src/${STM32_SERIES}_it.c"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Src/${STM32_SERIES}_hal_msp.c"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Src/${STM32_SERIES}_hal_timebase_tim.c"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Src/system_${STM32_SERIES}.c"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"

    FILE_PATH="./Core/Startup/startup_${STM32_SERIES}iikxq.s"
    echo "Deleting $FILE_PATH"
    rm -f "$FILE_PATH"
else
    echo "No STM32 series supplied—skipping series-specific cleanup."
fi

# Create the destination directory if it doesn't exist
mkdir -p "$mbedTLS_destination"

# Copy the contents from mbedTLS_source to mbedTLS_destination
cp -r "$mbedTLS_source" "$mbedTLS_destination"
echo "Content copied from $mbedTLS_source to $mbedTLS_destination"

FILE_PATH=$mbedTLS_destination"include/mbedtls/mbedtls_config.h"
echo "Deleting $FILE_PATH"
rm -f $FILE_PATH

FILE_PATH=$LwIP_destination"ports/freertos/include"
echo "Deleting $FILE_PATH"
rm -r -f $FILE_PATH



echo "Cleanup for $STM32_SERIES complete."
