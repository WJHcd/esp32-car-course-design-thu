#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "motion.h"
#include "display.h"
#include "ram_mode.h"
#include "camera_task.h"
#include "usb_audio.h"
#include "ultrasonic.h"

/* ============================================================
 * 攻城槌状态机实现
 *
 * SEARCH：脉冲自旋（转 250ms → 停 180ms 循环）+ GIF(搜索) + 音效(搜索)
 * ATTACK：对准 "1" 码缓慢前进 + GIF(撞击) + 音效(撞击)
 *         超声波判撞：<7cm 或 前进中距离 5 次不再减小 → 停 → 后退 650ms
 *         → 再前进（循环撞击）；目标丢失 >2.5s → 回 SEARCH
 *
 * 所有动作在 ram_task（core0）里跑；动画按帧延时非阻塞推进；
 * 语音回调上下文只做 enter/exit 标志切换。
 * ============================================================ */

#define TAG "ram"

/* 查找阶段脉冲自旋参数：停顿窗口加长（320ms），车身静止时
 * 运动模糊消失，摄像头在这段时间内解码最容易识别二维码 */
#define SEARCH_TURN_PCT     55      /* 旋转分量权重（motion 混控，上限 30% 轮速） */
#define SEARCH_ON_MS        220
#define SEARCH_OFF_MS       320

/* 撞击阶段参数 */
#define ATTACK_FWD_PCT      38      /* 前进权重（约 11% 实际轮速，缓慢逼近） */
#define ATTACK_STEER_MAX    26      /* 对准修正的转向分量上限 */
#define ATTACK_STEER_K      90      /* 转向比例系数：err=|x_ratio-0.5| */
#define ATTACK_DEADBAND     0.05f   /* 对准死区（画面中心 ±5% 不修方向） */

/* 撞击判定 */
#define HIT_DIST_CM         7       /* 距离小于此值 = 已接触 */
#define HIT_STALL_SAMPLES   5       /* 前进中连续 N 次距离不减小 = 顶住 */
#define HIT_STALL_TOL_CM    1
#define BACKOFF_MS          650     /* 撞后后退时长 */
#define BACKOFF_PCT         55

#define QR_FRESH_MS         600     /* 攻击中认为码仍可见的新鲜度 */
#define QR_LOST_BACK_MS     2500    /* 目标丢失多久退回查找 */

typedef enum {
    RAM_PHASE_SEARCH = 0,
    RAM_PHASE_ATTACK,
} ram_phase_t;

static const char *s_phase_names[] = { "目标查找", "撞击" };

static TaskHandle_t s_task;
static volatile bool s_active;

/* ---------------- 动画推进（非阻塞） ---------------- */
static void anim_tick(int asset, int64_t *last_us, int *frame)
{
    const int n = display_gif_count();
    if (n <= 0) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (now - *last_us >= 30 * 1000) {   /* GIF 帧延时压到 30ms 提帧率 */
        *last_us = now;
        (void)display_gif_draw(*frame);
        *frame = (*frame + 1) % n;
    }
}

/* ---------------- 主任务 ---------------- */
static void ram_task(void *arg)
{
    (void)arg;
    ram_phase_t phase = RAM_PHASE_SEARCH;
    int cur_gif = -1;
    audio_snd_t cur_snd = -1;
    int frame = 0;
    int64_t gif_last = 0;
    int64_t phase_entered = 0;
    int64_t hit_time = 0;

    /* 撞击检测状态 */
    uint16_t min_dist = 0xFFFF;
    int stall = 0;

    while (1) {
        if (!s_active) {
            /* 待命：停车断电，动画停 */
            motion_stop();
            motion_enable(false);
            audio_out_stop();
            cur_gif = -1;
            cur_snd = -1;
            phase = RAM_PHASE_SEARCH;
            frame = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        const int64_t now = esp_timer_get_time();

        /* ---- 阶段进入处理：切动画/音效 ---- */
        const int want_gif = (phase == RAM_PHASE_ATTACK) ? 1 : 0;
        const audio_snd_t want_snd = (phase == RAM_PHASE_ATTACK) ? AUDIO_SND_ATTACK
                                                                 : AUDIO_SND_SEARCH;
        if (cur_gif != want_gif) {
            display_gif_use(want_gif);
            frame = 0;
            gif_last = 0;
            cur_gif = want_gif;
        }
        if (cur_snd != want_snd) {
            audio_out_play(want_snd);
            cur_snd = want_snd;
        }

        /* ---- 动画推进 ---- */
        anim_tick(cur_gif, &gif_last, &frame);

        /* ---- 查询二维码 ---- */
        qr_result_t qr;
        int64_t age = -1;
        const bool have_qr = camera_qr_get(&qr, &age);
        const bool target_visible = have_qr && age >= 0 && age <= QR_FRESH_MS &&
                                    strcmp(qr.text, "1") == 0;

        if (phase == RAM_PHASE_SEARCH) {
            /* 脉冲自旋找目标 */
            const int64_t el = now - phase_entered;
            const int64_t period = (SEARCH_ON_MS + SEARCH_OFF_MS) * 1000;
            const bool on = (el % period) < SEARCH_ON_MS * 1000;
            motion_drive(0, on ? SEARCH_TURN_PCT : 0, 0);

            if (target_visible) {
                ESP_LOGI(TAG, "发现撞击目标（1 码）x=%.2f → 进入撞击",
                         qr.x_ratio);
                phase = RAM_PHASE_ATTACK;
                phase_entered = now;
                min_dist = 0xFFFF;
                stall = 0;
                motion_stop();
                vTaskDelay(pdMS_TO_TICKS(120));   /* 切阶段留出动画/音效切换 */
            }
            vTaskDelay(pdMS_TO_TICKS(25));
            continue;
        }

        /* ---- ATTACK 撞击阶段 ---- */

        /* 1) 撞击检测（150ms 一次测距） */
        static int64_t last_us_meas = 0;
        if (now - last_us_meas >= 150 * 1000) {
            last_us_meas = now;
            const uint16_t d = ultrasonic_measure_cm();
            if (d != ULTRASONIC_INVALID) {
                if (d < HIT_DIST_CM) {
                    /* 已接触：立即触发撞击流程 */
                    hit_time = now;
                    ESP_LOGI(TAG, "💥 撞击！距离 %ucm（累计第 %d 次）", d, 0);
                    motion_stop();
                    vTaskDelay(pdMS_TO_TICKS(150));
                    motion_drive(-BACKOFF_PCT, 0, 0);   /* 后退拉开 */
                    vTaskDelay(pdMS_TO_TICKS(BACKOFF_MS));
                    motion_stop();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    min_dist = 0xFFFF;
                    stall = 0;
                    continue;
                }
                if (d < min_dist) {
                    min_dist = d;
                    stall = 0;
                } else if (d <= min_dist + HIT_STALL_TOL_CM && d < 25) {
                    /* 距离很近且不再减小 = 顶住箱子 */
                    if (++stall >= HIT_STALL_SAMPLES) {
                        hit_time = now;
                        ESP_LOGI(TAG, "💥 撞击（顶住判定）距离 %ucm 不再减小", d);
                        motion_stop();
                        vTaskDelay(pdMS_TO_TICKS(150));
                        motion_drive(-BACKOFF_PCT, 0, 0);
                        vTaskDelay(pdMS_TO_TICKS(BACKOFF_MS));
                        motion_stop();
                        vTaskDelay(pdMS_TO_TICKS(200));
                        min_dist = 0xFFFF;
                        stall = 0;
                        continue;
                    }
                } else {
                    stall = 0;
                }
            }
        }

        /* 2) 目标丢失判定 */
        if (!target_visible && age > QR_LOST_BACK_MS) {
            ESP_LOGW(TAG, "目标丢失 >%dms，退回目标查找", (int)age);
            phase = RAM_PHASE_SEARCH;
            phase_entered = now;
            motion_stop();
            continue;
        }

        /* 3) 对准 + 缓慢前进 */
        if (target_visible) {
            const float err = qr.x_ratio - 0.5f;
            int turn = 0;
            if (err > ATTACK_DEADBAND || err < -ATTACK_DEADBAND) {
                /* err>0：目标在画面右侧 → 需要右转；motion 正转向=逆时针 */
                turn = (int)(-err * ATTACK_STEER_K);
                if (turn > ATTACK_STEER_MAX) turn = ATTACK_STEER_MAX;
                if (turn < -ATTACK_STEER_MAX) turn = -ATTACK_STEER_MAX;
            }
            motion_drive(ATTACK_FWD_PCT, turn, 0);
        } else {
            /* 短暂丢失：原地小幅修正等待重新识别 */
            motion_drive(0, 20, 0);
            vTaskDelay(pdMS_TO_TICKS(120));
            motion_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void ram_mode_init(void)
{
    if (s_task) {
        return;
    }
    /* 优先级与 img_proc 同级：FreeRTOS 时间片轮转，GIF 推屏不被
     * JPEG 解码（260ms 大块计算）长时间饿死，动画明显更流畅 */
    if (xTaskCreatePinnedToCore(ram_task, "ram", 6144, NULL, 5, &s_task, 0) != pdPASS) {
        ESP_LOGE(TAG, "ram 任务创建失败");
        return;
    }
    ESP_LOGI(TAG, "攻城槌模块初始化完成（语音说“攻城槌”进入）");
}

esp_err_t ram_mode_enter(void)
{
    display_init();        /* 幂等：首次进入初始化 SPI+ILI9163 */
    motion_stop();
    motion_enable(true);
    ultrasonic_init();
    s_active = true;
    ESP_LOGI(TAG, "★ 进入攻城槌模式：脉冲自旋寻找 1 码（撞击目标）");
    return ESP_OK;
}

void ram_mode_exit(void)
{
    s_active = false;
    /* 停车断电在 ram_task 待命分支完成，这里先立即刹停一遍 */
    motion_stop();
    motion_enable(false);
    audio_out_stop();
    ESP_LOGI(TAG, "已退出攻城槌模式（返回功能选择）");
}

bool ram_mode_is_active(void)
{
    return s_active;
}
