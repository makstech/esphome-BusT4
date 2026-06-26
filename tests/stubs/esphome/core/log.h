#pragma once
// Host-test stub for esphome/core/log.h — logging is a no-op under test.
#define ESP_LOGI(tag, ...) ((void) 0)
#define ESP_LOGW(tag, ...) ((void) 0)
#define ESP_LOGD(tag, ...) ((void) 0)
#define ESP_LOGV(tag, ...) ((void) 0)
