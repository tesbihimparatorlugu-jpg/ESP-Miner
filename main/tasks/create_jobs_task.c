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

/* -------------------------------------------------------------------------
 * Bouw een ASIC job.
 * extranonce_2 = 0 (zero-padded door mining_build_asic_job).
 * current_version = de actieve version die naar de ASIC gaat.
 * ---------------------------------------------------------------------- */
static void generate_work_from_miner_job(GlobalState *GLOBAL_STATE,
                                         const miner_job_t *job,
                                         uint64_t extranonce_2,
                                         uint32_t current_version)
{
    if (!job) return;

    asic_job_t next_job;

    // BELANGRIJK: gebruik altijd current_version, tenzij de pool
    // version-rolling volledig verboden heeft (mask == 0).
    // Zo krijgt de ASIC elke cyclus écht nieuwe work.
    uint32_t effective_version = current_version;
    if (job->version_mask == 0) {
        effective_version = job->version;
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

/* -------------------------------------------------------------------------
 * Hoofd-taak:
 *   - extranonce_2 BLIJFT ALTIJD 0 (zero-padded)
 *   - version rolling loopt DOOR tot een true job (clean_jobs == true)
 *   - ALLEEN bij een true job resetten we version + extranonce_2
 * ---------------------------------------------------------------------- */
void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint32_t current_version_mask = 0;
    miner_job_t *current_work   = NULL;
    bool     current_work_sent  = false;
    uint64_t extranonce_2       = 0;   // blijft permanent 0
    uint32_t current_version    = 0;
    int timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

    ESP_LOGI(TAG, "ASIC Job Interval: %d ms", timeout_ms);
    ESP_LOGI(TAG, "ASIC Ready!");

    while (1) {
        uint64_t start_time = esp_timer_get_time();
        uint32_t slot_notify = 0;
        TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &slot_notify, wait_ticks);
        timeout_ms -= (esp_timer_get_time() - start_time) / 1000;

        if (notified == pdTRUE) {
            miner_job_t *new_work = miner_job_get_slot((size_t)slot_notify);

            bool is_true_job = (current_work == NULL) || new_work->clean_jobs;

            ESP_LOGI(TAG,
                     "New Work Activated (slot %lu) %s (type %d) clean=%d true=%d",
                     (unsigned long)slot_notify,
                     new_work->job_id,
                     new_work->type,
                     new_work->clean_jobs,
                     is_true_job);

            current_work = new_work;
            GLOBAL_STATE->active_job_slot_idx =
                (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
            current_work_sent = false;

            if (new_work->version_mask != current_version_mask &&
                GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Set chip version rolls %i",
                         (int)(new_work->version_mask >> 13));
                ASIC_set_version_mask(GLOBAL_STATE, new_work->version_mask);
                current_version_mask = new_work->version_mask;
            }

            if (is_true_job) {
                ESP_LOGI(TAG,
                         "True job (new block data): reset version roll "
                         "(extranonce_2 blijft 0)");
                extranonce_2    = 0;                    // blijft 0
                current_version = new_work->version;    // reset

                if (GLOBAL_STATE->ASIC_initalized) {
                    difficulty_controller_update(GLOBAL_STATE);
                }
            } else {
                ESP_LOGI(TAG,
                         "Continuation job: keep rolling "
                         "(extranonce_2=0, version=0x%08" PRIx32 ")",
                         current_version);
                // current_version NIET resetten — blijft doorrollen
            }

            if (!new_work->clean_jobs && !is_true_job) {
                continue;
            }
        } else {
            if (current_work == NULL) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
        }

        // Stuur de job met de HUIDIGE current_version naar de ASIC.
        // (Dus niet job->version voor hardware-rolling ASICs!)
        generate_work_from_miner_job(GLOBAL_STATE,
                                     current_work,
                                     extranonce_2,      // altijd 0
                                     current_version);

        if (!current_work_sent) {
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, current_work);
        }
        current_work_sent = true;

        // -----------------------------------------------------------------
        // HOST-SIDE VERSION ROLLING
        // Omdat we extranonce_2 op 0 houden, is dit onze ENIGE bron van
        // verse work voor de ASIC. Elke cyclus schuift current_version
        // één stap op binnen de pool-mask.
        // -----------------------------------------------------------------
        uint32_t mask = (current_work->version_mask != 0)
                        ? current_work->version_mask
                        : BIP320_VERSION_ROLLING_MASK;

        if (mask != 0) {
            // Eén stap per cyclus. De ASIC krijgt hierdoor elke 500 ms
            // een andere effectieve version en vindt dus andere nonces.
            current_version = increment_bitmask(current_version, mask);
        }

        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}
