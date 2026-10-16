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

// ============================================================
// Extranonce2 helpers
// ============================================================

// 4-byte swap
static inline uint32_t extranonce_2_swap32(uint32_t x)
{
    return ((x & 0x000000FFu) << 24) |
           ((x & 0x0000FF00u) <<  8) |
           ((x & 0x00FF0000u) >>  8) |
           ((x & 0xFF000000u) >> 24);
}

// 8-byte swap
static inline uint64_t extranonce_2_swap64(uint64_t v)
{
    return ((v & 0x00000000000000FFULL) << 56) |
           ((v & 0x000000000000FF00ULL) << 40) |
           ((v & 0x0000000000FF0000ULL) << 24) |
           ((v & 0x00000000FF000000ULL) <<  8) |
           ((v & 0x000000FF00000000ULL) >>  8) |
           ((v & 0x0000FF0000000000ULL) >> 24) |
           ((v & 0x00FF000000000000ULL) >> 40) |
           ((v & 0xFF00000000000000ULL) >> 56);
}

// Slimme encode op basis van pool-lengte:
//  - 8-byte pools (solofury, public-pool V1/V2): 8-byte swap
//  - 4-byte pools (btcpowlab):                   4-byte swap
static inline uint64_t extranonce_2_encode(uint64_t v, int len)
{
    if (len >= 8) return extranonce_2_swap64(v);
    if (len == 4) return (uint64_t)extranonce_2_swap32((uint32_t)v);
    return v;
}

// ============================================================
// Work generation
// ============================================================

static void generate_work_from_miner_job(GlobalState *GLOBAL_STATE,
                                         const miner_job_t *job,
                                         uint64_t extranonce_2,
                                         uint32_t current_version)
{
    if (!job) return;

    asic_job_t next_job;
    uint32_t effective_version = job->version;

    // Voor HW-rollende ASICs gebruiken we de doorgerolde software-versie,
    // zodat de version-teller NIET reset naar 0 bij elke nieuwe job.
    if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling) {
        uint32_t mask = job->version_mask != 0
                        ? job->version_mask
                        : BIP320_VERSION_ROLLING_MASK;
        effective_version = (job->version & ~mask) | (current_version & mask);
    } else {
        if (!miner_job_is_rollable(job)) {
            effective_version = current_version;
        }
    }

    // Extranonce2 slim encoderen op basis van pool-lengte.
    //
    // !!! PAS DIT AAN als jouw project een andere veldnaam gebruikt !!!
    // Zoek in je project naar waar "extranonce_2_len" wordt gezet
    // (bijv. in stratum_v1.c of stratum_api.c). Het veld zit meestal
    // in GLOBAL_STATE en heet iets als:
    //   stratum_extranonce_2_len  /  extranonce_2_len  /  pool_extranonce_2_len
    int en2_len = GLOBAL_STATE->stratum_extranonce_2_len;
    if (en2_len <= 0) en2_len = 8;  // fallback voor de zekerheid

    uint64_t encoded_en2 = extranonce_2_encode(extranonce_2, en2_len);

    if (!mining_build_asic_job(job, encoded_en2, effective_version, &next_job)) {
        ESP_LOGE(TAG, "Failed to build ASIC job");
        return;
    }
    if (!GLOBAL_STATE->ASIC_initalized) {
        ESP_LOGW(TAG, "ASIC not initialized, skipping job send");
        return;
    }
    ASIC_send_job(GLOBAL_STATE, &next_job);
}

// ============================================================
// Hoofdtaak
// ============================================================

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

    // Hoeveel version-stappen per job-window doorschuiven.
    // De ASIC rolt in één window (paar honderd ms) ongeveer 256-512
    // versies door. Door de basis met deze stap op te hogen, dekt
    // elke volgende job een NIEUWE slice van de version-ruimte.
    //
    //   0x100 (256)  → 256 windows om alle 65536 versies te dekken
    //   0x200 (512)  → 128 windows
    //   0x400 (1024) →  64 windows
    //
    // Bij 500ms interval en 0x200: alle versies gedekt in ~64 seconden.
    const uint32_t VERSION_STEP = 0x00000200;

    while (1) {
        uint64_t start_time = esp_timer_get_time();
        uint32_t slot_notify = 0;
        TickType_t wait_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &slot_notify, wait_ticks);
        timeout_ms -= (esp_timer_get_time() - start_time) / 1000;

        if (notified == pdTRUE) {
            miner_job_t *new_work = miner_job_get_slot((size_t)slot_notify);
            ESP_LOGI(TAG, "New Work (slot %lu) %s (type %d, clean=%d)",
                     (unsigned long)slot_notify, new_work->job_id,
                     new_work->type, new_work->clean_jobs);

            // Non-clean job: negeren, huidige werk blijven rollen.
            if (!new_work->clean_jobs && current_work != NULL) {
                ESP_LOGI(TAG, "Non-clean job, ignoring and continuing current work");
                continue;
            }

            // Clean job: echt wisselen en alles resetten.
            current_work = new_work;
            GLOBAL_STATE->active_job_slot_idx = (uint8_t)(slot_notify % MINER_JOB_POOL_SIZE);
            current_work_sent = false;
            current_version = new_work->version;
            extranonce_2 = 0;

            if (new_work->version_mask != current_version_mask && GLOBAL_STATE->ASIC_initalized) {
                ESP_LOGI(TAG, "Set chip version rolls %i", (int)(new_work->version_mask >> 13));
                ASIC_set_version_mask(GLOBAL_STATE, new_work->version_mask);
                current_version_mask = new_work->version_mask;
            }

            if (GLOBAL_STATE->ASIC_initalized) {
                difficulty_controller_update(GLOBAL_STATE);
            }
        } else {
            if (current_work == NULL) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
            if (!miner_job_is_rollable(current_work) && current_work_sent &&
                GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling) {
                timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
                continue;
            }
        }

        generate_work_from_miner_job(GLOBAL_STATE, current_work, extranonce_2, current_version);

        if (!current_work_sent) {
            SYSTEM_decode_and_apply_coinbase(GLOBAL_STATE, current_work);
        }
        current_work_sent = true;

        // ------------------------------------------------------------
        // Version + extranonce2 voortgang
        // ------------------------------------------------------------
        if (GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling &&
            miner_job_is_rollable(current_work))
        {
            // HW-rollende ASIC (bijv. BM1370):
            //  - basis-versie opschuiven met VERSION_STEP
            //  - en nieuwe extranonce2 per window → nieuwe merkle root
            uint32_t mask = current_work->version_mask != 0
                            ? current_work->version_mask
                            : BIP320_VERSION_ROLLING_MASK;

            // Stap VERSION_STEP keer door in de version-ruimte
            for (uint32_t i = 0; i < VERSION_STEP; i++) {
                current_version = increment_bitmask(current_version, mask);
            }

            extranonce_2++;

        } else if (miner_job_is_rollable(current_work)) {
            // Niet-HW-rollend maar wel rollable job: alleen extranonce2
            extranonce_2++;

        } else if (!GLOBAL_STATE->DEVICE_CONFIG.family.asic.hardware_version_rolling) {
            // Software version rolling (bijv. BM1397)
            uint32_t mask = (current_work->version_mask != 0)
                            ? current_work->version_mask
                            : BIP320_VERSION_ROLLING_MASK;
            uint8_t midstates = GLOBAL_STATE->DEVICE_CONFIG.family.asic.software_midstates;
            for (int i = 0; i < midstates; i++) {
                current_version = increment_bitmask(current_version, mask);
            }
        }

        timeout_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);
    }
}
