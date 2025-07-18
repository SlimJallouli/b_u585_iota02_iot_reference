#!/bin/bash

# Define replacements as key-value pairs
declare -A REPLACEMENTS=(
    ["stm32h573i_dk_iot_reference"]="b_u585_iota02_iot_reference"
    ["STM32H573IIKxQ"]="STM32U585AIIxQ"
    ["STM32H573IIKXQ_FLASH.ld"]="STM32U585AIIXQ_FLASH.ld"
    ["STM32H5xx_HAL_Driver"]="STM32U5xx_HAL_Driver"
)

FILES_TO_PATCH=("./.cproject" "./.project")

echo "Updating ${FILES_TO_PATCH[*]}..."

for FILE in "${FILES_TO_PATCH[@]}"; do
    if [ -f "$FILE" ]; then
        for OLD in "${!REPLACEMENTS[@]}"; do
            NEW="${REPLACEMENTS[$OLD]}"
            if grep -q "$OLD" "$FILE"; then
                echo "Modified file: $FILE → Replacing '$OLD' → '$NEW'"
                sed -i "s/$OLD/$NEW/g" "$FILE"
            fi
        done
    else
        echo "Skipped $FILE (not found)"
    fi
done

# Delete .s files in Core/Startup
echo "Deleting existing .s files from ./Core/Startup..."
find "./Core/Startup" -type f -name "*.s" -exec rm -v {} +

# Copy new startup file
STARTUP_FILE="./startup_stm32u575zitxq.s"
DEST_DIR="./Core/Startup"

if [ -f "$STARTUP_FILE" ]; then
    echo "Copying $STARTUP_FILE to $DEST_DIR..."
    cp "$STARTUP_FILE" "$DEST_DIR"
    echo "Removing original $STARTUP_FILE..."
    rm -v "$STARTUP_FILE"
else
    echo "Startup file $STARTUP_FILE not found—skipping copy and removal."
fi

echo "Targeted replacements and startup cleanup complete."
