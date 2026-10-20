#include <sys/time.h>
#include <limits.h>
#include <inttypes.h>

#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mining.h"
#include "miner_job.h"
#include "string.h"
#include "esp_timer.h"

#include "asic.h"
#include "asic_common.h"
#include "difficulty_controller.h"
#include "system.h"
#include "esp_heap_caps.h"
#include "utils.h"

static const char *TAG = "create_jobs_task";

static void generate_work_from_miner_job(GlobalState *GLOBAL_STATE, const miner_job_t *job, uint64_t extranonce_2, uint32_t current_version)
{
    if (!job) return;

    asic_job_t next_job;
    uint32_t effective_version = job->version;
    if (!GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling && !miner_job_is_rollable(job)) {
        effective_version = current_version;
    }
    if (!mining_build_asic_job(job, extranonce_2, effective_version, &next_job)) {
        ESP_LOGE(TAG, "Failed to build ASIC job");
        return;
    }
    if (!GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
        return;
    }
    ASIC_send_job(GLOBAL_STATE, &next_job);
}

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint32_t current_version_mask = 0;
    miner_job_t *current_work = NULL;
    bool current_work_sent = false;
    uint64_t extranonce_2 = 0;
    uint32_t current_version = 0;
    int timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

    ESP_LOGI(TAG, "ASIC Job Interval: %d ms", timeout_ms);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        uint64_t start_time = esp_timer_get_time();
        uint32_t slot_notify = 0;
        
        TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &slot_notify, wait_ticks);
        
        if (timeout_ms > 0) {
            timeout_ms -= (esp_timer_get_time() - start_time) / 1000;
        }

        if (notified == pdTRUE) {
            miner_job_t *new_work = miner_job_get_slot((size_t)slot_notify);
            
            if (new_work) {
                if (!new_work->clean_jobs && current_work != NULL) {
                    // Non-clean job: Job pointer güncelleniyor (yeni job_id kullanımı için),
                    // ancak extranonce_2 ve version state'i KESİNLİKLE sıfırlanmıyor!
                    ESP_LOGI(TAG, "Non-clean job (false): job_id güncellendi, rolling state korundu.");
                    current_work = new_work;
                    GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
                    current_work_sent = false;
                } else {
                    // Clean job (true): Tamamen yeni blok, her şeyi sıfırla
                    ESP_LOGI(TAG, "Clean Job (true) ontvangen! Activatie nieuwe job (slot %lu) %s", (unsigned long)slot_notify, new_work->job_id);
                    current_work = new_work;
                    GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
                    current_work_sent = false;
                    current_version = new_work->version;
                    extranonce_2 = 0; // Sadece burada sıfırlanır
                }

                if (new_work->version_mask != current_version_mask && GLOBAL_STATE->ASIC_initalized) {
                    ESP_LOGI(TAG, "Set chip version rolls %i", (int)(new_work->version_mask >> 13));
                    ASIC_set_version_mask(GLOBAL_STATE, new_work->version_mask);
                    current_version_mask = new_work->version_mask;
                }

                if (GLOBAL_STATE->ASIC_initalized) {
                    difficulty_controller_update(GLOBAL_STATE);
                }
            }
        }

        if (current_work == NULL) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Çalışmayı ASIC'e gönder
        generate_work_from_miner_job(GLOBAL_STATE, current_work, extranonce_2, current_version);
        if (!current_work_sent) {
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, current_work);
            current_work_sent = true;
        }

        // Sayaçları artırmaya devam et
        if (miner_job_is_rollable(current_work)) {
            extranonce_2++;
        } else if (!GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling) {
            uint32_t mask = (current_work->version_mask != 0) ? current_work->version_mask : BIP320_VERSION_ROLLING_MASK;
            uint8_t midstates = GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;
            for (int i = 0; i < midstates; i++) {
                current_version = increment_bitmask(current_version, mask);
            }
        }
        
        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}
