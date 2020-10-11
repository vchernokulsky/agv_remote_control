#!/usr/bin/env bash

ROOT_PATH=$(realpath "$(dirname ${0})/../")

npx lv_font_conv \
  --size 14 \
  --bpp 4 \
  --lcd \
  --format lvgl \
  --output $ROOT_PATH/main/gui/screens/assets/MaterialDesignIconsFont.c \
  --force-fast-kern-format \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF008E,0xF12A1,0xF12A2,0xF12A3 `# battery` \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF125E `# no battery` \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF089F,0xF12A4,0xF12A5,0xF12A6 `# battery charging` \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF092F,0xF091F,0xF0922,0xF0925,0xF0928 `# wifi signal` \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF092E `# no wifi` \
  --font $ROOT_PATH/resources/materialdesignicons-webfont.ttf --range 0xF092B `# wifi alert` \

sed -i "" "s|${ROOT_PATH}/||g" $ROOT_PATH/main/gui/screens/assets/MaterialDesignIconsFont.c
