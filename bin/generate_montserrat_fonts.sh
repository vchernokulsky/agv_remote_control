#!/usr/bin/env bash

ROOT_PATH=$(realpath "$(dirname ${0})/../")

function generate() {
  size=$1

  npx lv_font_conv \
    --size ${size} \
    --bpp 4 \
    --lcd \
    --format lvgl \
    --output $ROOT_PATH/main/gui/screens/assets/Montserrat${size}Font.c \
    --force-fast-kern-format \
    --font $ROOT_PATH/resources/Montserrat-Medium.ttf --range 0x20-0x7F `# Basic Latin` \
    --font $ROOT_PATH/resources/Montserrat-Medium.ttf --range 0xB0,0x2022 `# Additional symbols as in LVGL fonts` \
    --font $ROOT_PATH/resources/Montserrat-Medium.ttf --range 0x0410-0x044F,0x0401,0x0451 `# Cyrillic` \
    --font $ROOT_PATH/resources/Montserrat-Medium.ttf --range 0x2116 `# Additional Cyrillic symbols` \

  sed -i "" "s|${ROOT_PATH}/||g" $ROOT_PATH/main/gui/screens/assets/Montserrat${size}Font.c
}

generate 12
generate 14
generate 16