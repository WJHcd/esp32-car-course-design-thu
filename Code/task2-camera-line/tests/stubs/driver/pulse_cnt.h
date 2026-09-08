#pragma once
#include <stdbool.h>
#include <stdlib.h>
typedef struct { int count; } *pcnt_unit_handle_t;
typedef void *pcnt_channel_handle_t;
typedef struct { int low_limit, high_limit; struct { bool accum_count; } flags; } pcnt_unit_config_t;
typedef struct { unsigned max_glitch_ns; } pcnt_glitch_filter_config_t;
typedef struct { int edge_gpio_num, level_gpio_num; } pcnt_chan_config_t;
#define PCNT_CHANNEL_EDGE_ACTION_INCREASE 1
#define PCNT_CHANNEL_EDGE_ACTION_HOLD 0
static inline int pcnt_new_unit(const pcnt_unit_config_t *c, pcnt_unit_handle_t *u)
{ (void)c; *u = calloc(1, sizeof(**u)); return 0; }
static inline int pcnt_unit_set_glitch_filter(pcnt_unit_handle_t u, const pcnt_glitch_filter_config_t *f)
{ (void)u; (void)f; return 0; }
static inline int pcnt_new_channel(pcnt_unit_handle_t u, const pcnt_chan_config_t *c, pcnt_channel_handle_t *h)
{ (void)c; *h = u; return 0; }
static inline int pcnt_channel_set_edge_action(pcnt_channel_handle_t h, int a, int b)
{ (void)h; (void)a; (void)b; return 0; }
static inline int pcnt_unit_enable(pcnt_unit_handle_t u) { (void)u; return 0; }
static inline int pcnt_unit_start(pcnt_unit_handle_t u) { (void)u; return 0; }
static inline int pcnt_unit_clear_count(pcnt_unit_handle_t u) { u->count = 0; return 0; }
static inline int pcnt_unit_get_count(pcnt_unit_handle_t u, int *c) { *c = u->count; return 0; }
