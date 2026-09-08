#pragma once
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) do { if (0) { (void)(tag); printf(fmt, ##__VA_ARGS__); } } while (0)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI
