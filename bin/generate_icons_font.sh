#!/usr/bin/env bash

ROOT_PATH=$(realpath "$(dirname ${0})/../")

npx lv_font_conv \
  --size 16 \
  --bpp 4 \
  --lcd \
  --format lvgl \
  --output $ROOT_PATH/main/gui/screens/assets/IconsFont.c \
  --force-fast-kern-format \
  --font $ROOT_PATH/resources/icons/icons.ttf --range 0x0030-0x0037 `# wi-fi` \
  --font $ROOT_PATH/resources/icons/icons.ttf --range 0x0041-0x0046 `# battery` \

sed -i "" "s|${ROOT_PATH}/||g" $ROOT_PATH/main/gui/screens/assets/IconsFont.c
