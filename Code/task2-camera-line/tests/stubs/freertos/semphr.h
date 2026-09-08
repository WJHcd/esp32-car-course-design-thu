#pragma once
#include <stdlib.h>
typedef int *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return calloc(1, sizeof(int)); }
static inline int xSemaphoreTake(SemaphoreHandle_t s, unsigned timeout)
{ (void)timeout; if (*s) return 0; *s = 1; return 1; }
static inline void xSemaphoreGive(SemaphoreHandle_t s) { *s = 0; }
