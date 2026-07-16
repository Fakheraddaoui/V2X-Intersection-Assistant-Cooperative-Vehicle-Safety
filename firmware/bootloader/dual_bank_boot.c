/*
 * STM32H7 dual-bank OTA boot logic with 3-strike rollback.
 * Bank swap is done via the FLASH_OPTCR SWAP_BANK option bit.
 */
#include <stdint.h>
#include <stdbool.h>

#define MAX_BOOT_ATTEMPTS 3u

typedef struct {
    uint32_t magic;           /* 0xB007C0DE when valid */
    uint32_t fw_version;
    uint32_t boot_attempts;   /* incremented pre-boot, cleared on confirm */
    uint32_t pending_swap;
} boot_meta_t;

extern boot_meta_t *bootmeta_load(void);
extern void bootmeta_store(const boot_meta_t *m);
extern bool image_signature_valid(uint8_t bank);
extern void flash_swap_banks(void);
extern void jump_to_application(void);

void bootloader_main(void)
{
    boot_meta_t *m = bootmeta_load();

    if (m->pending_swap) {
        if (image_signature_valid(1)) {
            flash_swap_banks();
            m->pending_swap = 0;
            m->boot_attempts = 0;
        } else {
            m->pending_swap = 0; /* refuse unsigned image */
        }
        bootmeta_store(m);
    }

    /* 3-strike rollback: app must call ota_confirm() to clear the counter */
    if (++m->boot_attempts > MAX_BOOT_ATTEMPTS) {
        flash_swap_banks();      /* revert to previous bank */
        m->boot_attempts = 0;
    }
    bootmeta_store(m);

    jump_to_application();
}

/* Called by application after passing self-test */
void ota_confirm(void)
{
    boot_meta_t *m = bootmeta_load();
    m->boot_attempts = 0;
    bootmeta_store(m);
}
