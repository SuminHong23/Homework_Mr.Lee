#include <cstring>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
    #include "esp_system.h"
    #include "esp_log.h"
}

// MatrixPanel_I2S_DMA 헤더 포함
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

#include "esp_log.h"

static const char *TAG = "APP_MAIN";

MatrixPanel_I2S_DMA *dma_display = nullptr;

uint16_t myBLACK, myWHITE, myRED, myGREEN, myBLUE;

uint16_t colorWheel(uint8_t pos) {
    if(pos < 85) {
        return dma_display->color565(pos * 3, 255 - pos * 3, 0);
    } else if(pos < 170) {
        pos -= 85;
        return dma_display->color565(255 - pos * 3, 0, pos * 3);
    } else {
        pos -= 170;
        return dma_display->color565(0, pos * 3, 255 - pos * 3);
    }
}

void drawText(int colorWheelOffset) {
    dma_display->setTextSize(1);
    dma_display->setTextWrap(false);
    dma_display->setCursor(5, 0);

    uint8_t w = 0;
    const char *str = "ESP32 DMA";

    for (w=0; w < strlen(str); w++) {
        dma_display->setTextColor(colorWheel((w * 32) + colorWheelOffset));
        dma_display->print(str[w]);
    }

    dma_display->println();
    dma_display->print(" ");
    for (w = 9; w < 18; w++) {
        dma_display->setTextColor(colorWheel((w * 32) + colorWheelOffset));
        dma_display->print("*");
    }

    dma_display->println();

    dma_display->setTextColor(dma_display->color444(15, 15, 15));
    dma_display->println("LED MATRIX!");

    dma_display->setTextColor(dma_display->color444(0, 8, 15));
    dma_display->print('3');
    dma_display->setTextColor(dma_display->color444(15, 4, 0));
    dma_display->print('2');
    dma_display->setTextColor(dma_display->color444(15, 15, 0));
    dma_display->print('x');
    dma_display->setTextColor(dma_display->color444(8, 15, 0));
    dma_display->print('6');
    dma_display->setTextColor(dma_display->color444(8, 0, 15));
    dma_display->print('4');

    dma_display->setCursor(34, 24);
    dma_display->setTextColor(dma_display->color444(0, 15, 15));
    dma_display->print("*");
    dma_display->setTextColor(dma_display->color444(15, 0, 0));
    dma_display->print('R');
    dma_display->setTextColor(dma_display->color444(0, 15, 0));
    dma_display->print('G');
    dma_display->setTextColor(dma_display->color444(0, 0, 15));
    dma_display->print("B");
    dma_display->setTextColor(dma_display->color444(15, 0, 8));
    dma_display->println("*");
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000)); // 부팅 후 대기
    ESP_LOGI(TAG, "app_main 실행 시작!");

    HUB75_I2S_CFG mxconfig(
        PANEL_RES_X,
        PANEL_RES_Y,
        PANEL_CHAIN
    );

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);

    if (!dma_display->begin()) {
        ESP_LOGE(TAG, "MatrixPanel_I2S_DMA 초기화 실패!");
        return;
    }

    dma_display->setBrightness8(100);
    dma_display->fillScreen(dma_display->color565(255, 255, 255)); // 흰색으로 전체 채움
}
