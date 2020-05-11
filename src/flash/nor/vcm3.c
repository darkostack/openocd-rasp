/***************************************************************************
 *   Copyright (C) 2018 Vertexcom Technologies. Inc                        *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/types.h>

#define VCM3_VERSION_ID 0x4004803C

#define ERASE_SECTOR_DEPENDS_ON_CODESIZE 0

/* vcm3 flash csr registers */
enum vcm3_fcsr_registers {
    FCSR_BASE = 0x40020000,

#define FCSR_REG(offset) (FCSR_BASE + offset)

    FCSR_MEM_CACCFG          = FCSR_REG(0x00C),   // cache configuration register
    FCSR_MEM_CACINVS         = FCSR_REG(0x020),   // cache invalid start address
    FCSR_MEM_CACINVE         = FCSR_REG(0x024),   // cache invalid end address
    FCSR_MEM_CACINV          = FCSR_REG(0x028),   // cache invalid active register
    FCSR_MEM_CACHIT          = FCSR_REG(0x030),   // cache hit rate counting register
    FCSR_MEM_CACHITL         = FCSR_REG(0x034),   // cache hit rate lowest value register
    FCSR_FLASH_CTRL          = FCSR_REG(0x040),   // embedded flash control register
    FCSR_EMBFLASH_PASS       = FCSR_REG(0x044),   // embedded flash password register
    FCSR_EMBFLASH_PGADDR     = FCSR_REG(0x048),   // embedded flash program address register
    FCSR_EMBFLASH_PGDATA     = FCSR_REG(0x04C),   // embedded flash program word data register
    FCSR_EMBFLASH_SERASE     = FCSR_REG(0x050),   // embedded flash sector erase control register
    FCSR_EMBFLASH_CERASE     = FCSR_REG(0x058),   // embedded flash chip erase control register
    FCSR_EMBFLASH_CSSADDR    = FCSR_REG(0x060),   // embedded flash checksum start address
    FCSR_EMBFLASH_CSEADDR    = FCSR_REG(0x064),   // embedded flash checksum end address
    FCSR_EMBFLASH_CSVALUE    = FCSR_REG(0x068),   // embedded flash checksum value register
    FCSR_EMBFLASH_CSCVALUE   = FCSR_REG(0x06c),   // embedded flash checksum compare value register
    FCSR_EMBFLASH_INTEN      = FCSR_REG(0x070),   // embedded flash checksum interrupt enable register
    FCSR_EMBFLASH_INT        = FCSR_REG(0x074),   // embedded flash checksum interrupt status register
    FCSR_EMBFLASH_RPROT      = FCSR_REG(0x078),   // embedded flash read protect status register
    FCSR_EMBFLASH_WPROT      = FCSR_REG(0x07C),   // embedded flash write protect control register
    FCSR_EMBFLASH_NVRPASS    = FCSR_REG(0x084),   // embedded flash NVR sector password register
    FCSR_EMBFLASH_STS        = FCSR_REG(0x088),   // embedded flash programming status register
    FCSR_EMBFLASH_CONF       = FCSR_REG(0x09C),   // embedded flash configuration read/write register
    FCSR_EMBFLASH_DSTB       = FCSR_REG(0x0A0),   // embedded flash deep standby control register
    FCSR_EMBFLASH_PTIME      = FCSR_REG(0x0B0),   // embedded flash program time control register
    FCSR_EMBFLASH_ETIME      = FCSR_REG(0x0B4),   // embedded flash erase time control register
};

/* key definitions */
#define FLASH_PASS_KEY       0x55AAAA55
#define FLASH_SERASE_KEY     0xAA5555AA
#define FLASH_CERASE_KEY     0xAA5555AA
#define FLASH_NVRPASS_KEY    0xAA5555AA
#define FLASH_DSTB_KEY       0xAA5555AA

struct vcm3_info {
    uint32_t code_page_size;
    bool probed;
    struct target *target;
};

struct vcm3_device_spec {
    uint32_t version_id;
    const char *variant;
    uint8_t sector_size_kb;
    unsigned int flash_size_kb;
};

static const struct vcm3_device_spec vcm3_known_devices_table[] = {
    {
        .version_id = 0x18041901,
        .variant = "sirius",
        .sector_size_kb = 1,
        .flash_size_kb = 512,
    },
};

static int vcm3_bank_is_probed(struct flash_bank *bank)
{
    struct vcm3_info *chip = bank->driver_priv;
    assert(chip != NULL);
    return chip->probed;
}

static int vcm3_probe(struct flash_bank *bank);

static int vcm3_get_probed_chip_if_halted(struct flash_bank *bank, struct vcm3_info **chip)
{
    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    *chip = bank->driver_priv;

    int probed = vcm3_bank_is_probed(bank);

    if (probed < 0) {
        return probed;
    } else if (!probed) {
        return vcm3_probe(bank);
    } else {
        return ERROR_OK;
    }
}

static int vcm3_flash_wait_for_status_idle(struct vcm3_info *chip, int timeout)
{
    int res = ERROR_OK;
    uint32_t status;
    uint32_t counter = timeout;

    do {
        res = target_read_u32(chip->target, FCSR_EMBFLASH_STS, &status);
        if (res != ERROR_OK) {
            LOG_ERROR("couldn't read EMBFLASH_STS register");
            return res;
        }
        if (status == 0x00000001) {
            return ERROR_OK;
        }
        alive_sleep(1);
    } while (counter--);

	LOG_WARNING("timed out waiting for EMBFLASH_STS idle");
    return ERROR_FLASH_BUSY;
}

static int vcm3_flash_unlock(struct vcm3_info *chip)
{
    int res = ERROR_OK;
    res = target_write_u32(chip->target, FCSR_EMBFLASH_PASS, FLASH_PASS_KEY);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to unlock the flash");
    }
    return res;
}

static int vcm3_flash_disable_cache(struct vcm3_info *chip)
{
    int res = ERROR_OK;
    res = target_write_u32(chip->target, FCSR_MEM_CACCFG, 0x0);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to disable flash cache");
    }
    return res;
}

static int vcm3_flash_lock(struct vcm3_info *chip)
{
    int res = ERROR_OK;
    res = target_write_u32(chip->target, FCSR_EMBFLASH_PASS, 0);
    if (res != ERROR_OK) {
        LOG_ERROR("falied to lock the flash");
    }
    return res;
}

static int vcm3_flash_generic_erase(struct vcm3_info *chip,
                                    uint32_t erase_register,
                                    uint32_t erase_addr)
{
    int res = ERROR_OK;

    res = vcm3_flash_unlock(chip);
    if (res != ERROR_OK) {
        goto exit;
    }

    if (erase_register == FCSR_EMBFLASH_SERASE) {
        // sector erase
        res = target_write_u32(chip->target, FCSR_EMBFLASH_PGADDR, erase_addr);
        if (res != ERROR_OK) {
            goto exit;
        }

        res = target_write_u32(chip->target, erase_register, FLASH_SERASE_KEY);
        if (res != ERROR_OK) {
            goto exit;
        }

    } else if (erase_register == FCSR_EMBFLASH_CERASE) {
        // chip erase
        res = target_write_u32(chip->target, FCSR_EMBFLASH_PGADDR, erase_addr);
        if (res != ERROR_OK) {
            goto exit;
        }

        res = target_write_u32(chip->target, erase_register, FLASH_CERASE_KEY);
        if (res != ERROR_OK) {
            goto exit;
        }

    } else {
        res = ERROR_FAIL;
        goto exit; 
    }

    if (erase_register == FCSR_EMBFLASH_SERASE) {
        res = vcm3_flash_wait_for_status_idle(chip, 100);
    } else if (erase_register == FCSR_EMBFLASH_CERASE) {
        res = vcm3_flash_wait_for_status_idle(chip, 1000);
    }

exit:
    vcm3_flash_lock(chip);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to erase reg: 0x%08"PRIx32" val: 0x%08"PRIx32,
                  erase_register, erase_addr);
    }
    return res;
}

static int vcm3_protect_check(struct flash_bank *bank)
{
    int res = ERROR_OK;
    uint32_t protected;

    struct vcm3_info *chip = bank->driver_priv;

    assert(chip != NULL);

    res = target_read_u32(chip->target, FCSR_EMBFLASH_RPROT, &protected);
    if (res != ERROR_OK) {
        goto exit;
    }

    /*
     * num of sectors 512: 0 - 511 : 512KB (sector size 1KB)
     *
     * bit 0  : sector [0   - 15 ] : 16KB  - region 0
     * bit 1  : sector [16  - 31 ] : 32KB  - region 1
     * bit 2  : sector [32  - 47 ] : 48KB  - region 2
     *  ...
     * bit 30 : sector [480 - 495] : 496KB - region 30
     * bit 31 : sector [496 - 511] : 512KB - region 31
     */

    for (int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].is_protected = (protected & (1 << (i / 16)));
    }

exit:
    return res;
}

static int vcm3_protect(struct flash_bank *bank, int set, int first, int last)
{
    int res = ERROR_OK;
    struct vcm3_info *chip;
    uint32_t protected;

    if ((first < 0) || (last > 31)) {
        /* supported region: 0 - 31 */
        res = ERROR_FAIL;
        goto exit;
    }

    res = vcm3_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        goto exit;
    }

    res = target_read_u32(chip->target, FCSR_EMBFLASH_RPROT, &protected);
    if (res != ERROR_OK) {
        goto exit;
    }

    for (int region = first; region <= last; region++) {
        if (!(protected & (1 << region))) {
            uint32_t write_protected = (protected | (1 << region));
            res = target_write_u32(chip->target, FCSR_EMBFLASH_WPROT, write_protected);
            if (res != ERROR_OK) {
                goto exit;
            }
        }
    }

    vcm3_protect_check(bank);

exit:
    return res;
}

static int vcm3_probe(struct flash_bank *bank)
{
    int res = ERROR_OK;
    struct vcm3_info *chip = bank->driver_priv;
    uint32_t version_id;

    res = target_read_u32(chip->target, VCM3_VERSION_ID, &version_id);
    if (res != ERROR_OK) {
        LOG_ERROR("couldn't read VERSIONID register");
        goto exit;
    }

    const struct vcm3_device_spec *spec = NULL;
    for (size_t i = 0; i < ARRAY_SIZE(vcm3_known_devices_table); i++) {
        if (version_id == vcm3_known_devices_table[i].version_id) {
            spec = &vcm3_known_devices_table[i];
            break;
        }
    }

    if (!chip->probed) {
        if (spec) {
            LOG_INFO("vcm3-%s: %ukB flash", spec->variant, spec->flash_size_kb);
        } else {
            LOG_ERROR("unknown device version id (0x%08"PRIx32")", version_id);
            res = ERROR_FLASH_BANK_NOT_PROBED;
            goto exit;
        }
    }

    bank->size = spec->flash_size_kb * 0x400;
    bank->num_sectors = (spec->flash_size_kb / spec->sector_size_kb);
    bank->sectors = calloc(bank->num_sectors, sizeof((bank->sectors)[0]));

    chip->code_page_size = spec->sector_size_kb * 0x400;

    if (!bank->sectors) {
        res = ERROR_FLASH_BANK_NOT_PROBED;
        goto exit;
    }

    for (int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].size = chip->code_page_size;
        bank->sectors[i].offset = i * chip->code_page_size;

        /* mark as unknown */
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = -1;
    }

    vcm3_protect_check(bank);

    chip->probed = true;

    vcm3_flash_disable_cache(chip);

exit:
    return res;
}

static int vcm3_auto_probe(struct flash_bank *bank)
{
    int probed = vcm3_bank_is_probed(bank);

    if (probed < 0) {
        return probed;
    } else if (probed) {
        return ERROR_OK;
    } else {
        return vcm3_probe(bank);
    }
}

#if ERASE_SECTOR_DEPENDS_ON_CODESIZE

static struct flash_sector *vcm3_find_sector_by_address(struct flash_bank *bank, uint32_t address)
{
    struct vcm3_info *chip = bank->driver_priv;

    for (int i = 0; i < bank->num_sectors; i++) {
        if ((bank->sectors[i].offset <= address) &&
            (address < (bank->sectors[i].offset + chip->code_page_size))) {
            return &bank->sectors[i];
        }
    }

    return NULL;
}

#endif

static int vcm3_erase_page(struct flash_bank *bank,
                           struct vcm3_info *chip,
                           struct flash_sector *sector)
{
    int res = ERROR_OK;

    if (sector->is_protected) {
        LOG_ERROR("cannot erase protected sector at 0x%"PRIx32, sector->offset);
        res = ERROR_FAIL;
        goto exit;
    }

    res = vcm3_flash_generic_erase(chip, FCSR_EMBFLASH_SERASE, sector->offset);

    if (res == ERROR_OK) { 
        sector->is_erased = 1;
    }

exit:
    return res;
}

static const uint8_t vcm3_flash_write_code[] = {
    /* see contrib/loaders/flash/vcm3x.S */
                                /* wait_fifo: */
    0xd0, 0xf8, 0x00, 0x80,     /* ldr      r8, [r0, #0] */
    0xb8, 0xf1, 0x00, 0x0f,     /* cmp      r8, #0 */
    0x0f, 0xd0,                 /* beq      exit */
    0x47, 0x68,                 /* ldr      r7, [r0, #4] */
    0x47, 0x45,                 /* cmp      r7, r8 */
    0xf7, 0xd0,                 /* beq      wait_fifo */
    0x16, 0x46,                 /* mov      r6, r2 */
    0x04, 0x32,                 /* adds     r2, r2, #4 */
    0xa6, 0x64,                 /* str      r6, [r4, #VC_FLASH_PGADDR_OFFSET] */
    0x57, 0xf8, 0x04, 0x6b,     /* ldr      r6, [r7], #0x04 */
    0xe6, 0x64,                 /* str      r6, [r4, #VC_FLASH_PGDATA_OFFSET] */
    0x8f, 0x42,                 /* cmp      r7, r1 */
    0x01, 0xd3,                 /* bcc      no_wrap */
    0x07, 0x46,                 /* mov      r7, r0 */
    0x08, 0x37,                 /* adds     r7, #8 */
                                /* no_wrap */
    0x47, 0x60,                 /* str      r7, [r0, #4] */
    0x04, 0x3b,                 /* subs     r3, #4 */
    0xea, 0xd1,                 /* bne      wait_fifo */
                                /* exit: */
    0x00, 0xbe,                 /* bkpt     0x0000 */
};

/* start a low level flash write for the specified region */
static int vcm3_flash_write(struct vcm3_info *chip, uint32_t offset, const uint8_t *buffer, uint32_t bytes)
{
    struct target *target = chip->target;
    uint32_t buffer_size = 16384;
    struct working_area *write_algorithm;
    struct working_area *source;
    uint32_t address = offset;
    struct reg_param reg_params[5];
    struct armv7m_algorithm armv7m_info;
    int retval = ERROR_OK;

	LOG_INFO("writing buffer to flash offset=0x%"PRIx32" bytes=0x%"PRIx32, offset, bytes);

    assert(bytes % 4 == 0);

    /* allocate working area with flash programming code */
    if (target_alloc_working_area(target, sizeof(vcm3_flash_write_code), &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, falling back to slow memory writes");

        for(; bytes > 0; bytes -= 4) {
            uint32_t value;
            memcpy(&value, buffer, sizeof(uint32_t));

            target_write_u32(chip->target, FCSR_EMBFLASH_PGADDR, offset);
            target_write_u32(chip->target, FCSR_EMBFLASH_PGDATA, value);

            retval = vcm3_flash_wait_for_status_idle(chip, 100);
            if (retval != ERROR_OK) {
                return retval;
            }

            offset += 4;
            buffer += 4;
        }

        return ERROR_OK;
    }

    retval = target_write_buffer(target, write_algorithm->address,
                                 sizeof(vcm3_flash_write_code),
                                 vcm3_flash_write_code);

    if (retval != ERROR_OK) {
        return retval;
    }

    /* memory buffer */
    while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
        buffer_size /= 2;
        if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);
			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
        }
    }

    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);     /* buffer start, status (out) */
    init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);        /* buffer end */
    init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);        /* flash target address */
    init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);        /* bytes  */
    init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);        /* flash base */

    buf_set_u32(reg_params[0].value, 0, 32, source->address);
    buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
    buf_set_u32(reg_params[2].value, 0, 32, address);
    buf_set_u32(reg_params[3].value, 0, 32, bytes);
    buf_set_u32(reg_params[4].value, 0, 32, FCSR_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, bytes/4, 4,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

    if (retval == ERROR_FLASH_OPERATION_FAILED) {
        LOG_ERROR("error executing vcm3 flash write algorithm");
        retval = ERROR_FAIL;
    }

    target_free_working_area(target, source);
    target_free_working_area(target, write_algorithm);

    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);
    destroy_reg_param(&reg_params[2]);
    destroy_reg_param(&reg_params[3]);
    destroy_reg_param(&reg_params[4]);

    return retval;
}


/* check and erase flash sectors in specified range then start a low level page
 * write. start/end must be sector aligned.
 */
static int vcm3_write_pages(struct flash_bank *bank, uint32_t start, uint32_t end, const uint8_t *buffer)
{
    int res = ERROR_FAIL;
    struct vcm3_info *chip = bank->driver_priv;

    assert(start % chip->code_page_size == 0);
    assert(end % chip->code_page_size == 0);

#if ERASE_SECTOR_DEPENDS_ON_CODESIZE

    struct flash_sector *sector;
    uint32_t offset;

    /* erase sectors depends on the size of the image */
    for (offset = start; offset < end; offset += chip->code_page_size) {
        sector = vcm3_find_sector_by_address(bank, offset);
        if (!sector) {
            LOG_ERROR("invalid sector @ 0x%08"PRIx32, offset);
            return ERROR_FLASH_SECTOR_INVALID;
        }
        if (sector->is_protected) {
            LOG_ERROR("can't erase protected sector @ 0x%08"PRIx32, offset);
            goto exit;
        }
        if (sector->is_erased != 1) { /* 1 = erased, 0 = not-erased, -1 = unknown */
            res = vcm3_erase_page(bank, chip, sector);
            if (res != ERROR_OK) {
                LOG_ERROR("failed to erase sector @ 0x%08"PRIx32, sector->offset);
                goto exit;
            }
        }
        sector->is_erased = 0;
    }

#else

    /* skip hwset at 0x7fc00 (sector 511) and hwset2 at 0x7f400 (sector 509) */

    for (int i = 0; i < bank->num_sectors; i++) {
	if ((i == 511) || (i == 509)) {
	   continue;
	}
    	res = vcm3_erase_page(bank, chip, &bank->sectors[i]);
    	if (res != ERROR_OK) {
	    LOG_ERROR("failed to erase sector @ 0x%08"PRIx32, bank->sectors[i].offset);
	    goto exit;
    	}
    	bank->sectors[i].is_erased = 0;
    }

#endif

    res = vcm3_flash_unlock(chip);
    if (res != ERROR_OK) {
        goto exit;
    }

    res = vcm3_flash_write(chip, start, buffer, (end - start));
    if (res != ERROR_OK) {
        goto exit;
    }

exit:
    vcm3_flash_lock(chip);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write to vc flash");
    }
    return res;
}

static int vcm3_erase(struct flash_bank *bank, int first, int last)
{
    int res;
    struct vcm3_info *chip;

    res = vcm3_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    /* for each sector to be erased */
    for (int s = first; s <= last && res == ERROR_OK; s++) {
        res = vcm3_erase_page(bank, chip, &bank->sectors[s]);
    }

    return res;
}

static int vcm3_code_flash_write(struct flash_bank *bank,
                                 struct vcm3_info *chip,
                                 const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    int res;

    /* need to perform reads to fill any gaps we need to preserve in the first
     * page before the start of buffer, or in the last page, after the end of
     * buffer */

    uint32_t first_page = offset / chip->code_page_size;
    uint32_t last_page = DIV_ROUND_UP(offset + count, chip->code_page_size);

    uint32_t first_page_offset = first_page * chip->code_page_size;
    uint32_t last_page_offset = last_page * chip->code_page_size;


	LOG_DEBUG("Padding write from 0x%08"PRIx32"-0x%08"PRIx32" as 0x%08"PRIx32"-0x%08"PRIx32,
		offset, offset+count, first_page_offset, last_page_offset);

	uint32_t page_cnt = last_page - first_page;
	uint8_t buffer_to_flash[page_cnt * chip->code_page_size];

	/* Fill in any space between start of first page and start of buffer */
	uint32_t pre = offset - first_page_offset;
	if (pre > 0) {
		res = target_read_memory(bank->target,
					first_page_offset,
					1,
					pre,
					buffer_to_flash);
		if (res != ERROR_OK)
			return res;
	}

	/* Fill in main contents of buffer */
	memcpy(buffer_to_flash+pre, buffer, count);

	/* Fill in any space between end of buffer and end of last page */
	uint32_t post = last_page_offset - (offset+count);
	if (post > 0) {
		/* Retrieve the full row contents from Flash */
		res = target_read_memory(bank->target,
					offset + count,
					1,
					post,
					buffer_to_flash+pre+count);
		if (res != ERROR_OK)
			return res;
	}

    return vcm3_write_pages(bank, first_page_offset, last_page_offset, buffer_to_flash);
}

static int vcm3_write(struct flash_bank *bank, const uint8_t *buffer,
                      uint32_t offset, uint32_t count)
{
    int res;
    struct vcm3_info *chip;

    res = vcm3_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    return vcm3_code_flash_write(bank, chip, buffer, offset, count);
}

FLASH_BANK_COMMAND_HANDLER(vcm3_flash_bank_command)
{
    static struct vcm3_info *chip;

    if (bank->base != 0x00000000) {
        LOG_ERROR("invalid bank address 0x%08"PRIx32, (uint32_t)bank->base);
        return ERROR_FAIL;
    }

    if (!chip) {
        /* create a new chip */
        chip = calloc(1, sizeof(*chip));
        if (!chip) {
            return ERROR_FAIL;
        }
        chip->target = bank->target;
    }

    chip->probed = false;
    bank->driver_priv = chip;
    
    return ERROR_OK;
}

COMMAND_HANDLER(vcm3_handle_mass_erase_command)
{
    struct flash_bank *bank = NULL;
    struct target *target = get_current_target(CMD_CTX);
    int res;

    res = get_flash_bank_by_addr(target, 0x00000000, true, &bank);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to get flash bank");
        return res;
    }

    assert(bank != NULL);

    LOG_INFO("get flash bank base: 0x%08"PRIx32, (uint32_t)bank->base);

    struct vcm3_info *chip = bank->driver_priv;

    // chip erase
    res = vcm3_flash_generic_erase(chip, FCSR_EMBFLASH_CERASE, 0x00000000);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to erase the chip");
        vcm3_protect_check(bank);
        return res;
    }

    for (int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].is_erased = 1;
    }

    res = vcm3_protect_check(bank);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to check chip's write protection");
        return res;
    }

    return ERROR_OK;
}

static int vcm3_info(struct flash_bank *bank, char *buf, int buf_size)
{
    int res;

    struct vcm3_info *chip;

    res = vcm3_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    static struct {
        const uint32_t address;
        uint32_t value;
    } fcsr[] = {
        { .address = FCSR_MEM_CACCFG },
        { .address = FCSR_MEM_CACINVS },
        { .address = FCSR_MEM_CACINVE },
        { .address = FCSR_MEM_CACINV },
        { .address = FCSR_MEM_CACHIT },
        { .address = FCSR_MEM_CACHITL },
        { .address = FCSR_FLASH_CTRL },
        { .address = FCSR_EMBFLASH_PASS },
        { .address = FCSR_EMBFLASH_PGADDR },
        { .address = FCSR_EMBFLASH_PGDATA },
        { .address = FCSR_EMBFLASH_SERASE },
        { .address = FCSR_EMBFLASH_CERASE },
        { .address = FCSR_EMBFLASH_CSSADDR },
        { .address = FCSR_EMBFLASH_CSEADDR },
        { .address = FCSR_EMBFLASH_CSVALUE },
        { .address = FCSR_EMBFLASH_CSCVALUE },
        { .address = FCSR_EMBFLASH_INTEN },
        { .address = FCSR_EMBFLASH_INT },
        { .address = FCSR_EMBFLASH_RPROT },
        { .address = FCSR_EMBFLASH_WPROT },
        { .address = FCSR_EMBFLASH_NVRPASS },
        { .address = FCSR_EMBFLASH_STS },
        { .address = FCSR_EMBFLASH_CONF },
        { .address = FCSR_EMBFLASH_DSTB },
        { .address = FCSR_EMBFLASH_PTIME },
        { .address = FCSR_EMBFLASH_ETIME },
    };

    for (size_t i = 0; i < ARRAY_SIZE(fcsr); i++) {
        res = target_read_u32(chip->target, fcsr[i].address, &fcsr[i].value);
        if (res != ERROR_OK) {
            LOG_ERROR("couldn't read %"PRIx32, fcsr[i].address);
            return res;
        }
    }

    snprintf(buf, buf_size,
        "\n[flash register information]\n\n"
        "MEM_CACCFG        : %"PRIx32"\n"
        "MEM_CACINVS       : %"PRIx32"\n"
        "MEM_CACINVE       : %"PRIx32"\n"
        "MEM_CACINV        : %"PRIx32"\n"
        "MEM_CACHIT        : %"PRIx32"\n"
        "MEM_CACHITL       : %"PRIx32"\n"
        "FLASH_CTRL        : %"PRIx32"\n"
        "EMBFLASH_PASS     : %"PRIx32"\n"
        "EMBFLASH_PGADDR   : %"PRIx32"\n"
        "EMBFLASH_PGDATA   : %"PRIx32"\n"
        "EMBFLASH_SERASE   : %"PRIx32"\n"
        "EMBFLASH_CERASE   : %"PRIx32"\n"
        "EMBFLASH_CSSADDR  : %"PRIx32"\n"
        "EMBFLASH_CSEADDR  : %"PRIx32"\n"
        "EMBFLASH_CSVALUE  : %"PRIx32"\n"
        "EMBFLASH_CSCVALUE : %"PRIx32"\n"
        "EMBFLASH_INTEN    : %"PRIx32"\n"
        "EMBFLASH_INT      : %"PRIx32"\n"
        "EMBFLASH_RPROT    : %"PRIx32"\n"
        "EMBFLASH_WPROT    : %"PRIx32"\n"
        "EMBFLASH_NVRPASS  : %"PRIx32"\n"
        "EMBFLASH_STS      : %"PRIx32"\n"
        "EMBFLASH_CONF     : %"PRIx32"\n"
        "EMBFLASH_DSTB     : %"PRIx32"\n"
        "EMBFLASH_PTIME    : %"PRIx32"\n"
        "EMBFLASH_ETIME    : %"PRIx32"\n",
        fcsr[0].value,
        fcsr[1].value,
        fcsr[2].value,
        fcsr[3].value,
        fcsr[4].value,
        fcsr[5].value,
        fcsr[6].value,
        fcsr[7].value,
        fcsr[8].value,
        fcsr[9].value,
        fcsr[10].value,
        fcsr[11].value,
        fcsr[12].value,
        fcsr[13].value,
        fcsr[14].value,
        fcsr[15].value,
        fcsr[16].value,
        fcsr[17].value,
        fcsr[18].value,
        fcsr[19].value,
        fcsr[20].value,
        fcsr[21].value,
        fcsr[22].value,
        fcsr[23].value,
        fcsr[24].value,
        fcsr[25].value
    );

    return ERROR_OK;
}

static const struct command_registration vcm3_exec_command_handlers[] = {
    {
        .name    = "mass_erase",
        .handler = vcm3_handle_mass_erase_command,
        .mode    = COMMAND_EXEC,
        .help    = "Erase all flash content of the chip.",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration vcm3_command_handlers[] = {
    {
        .name = "vcm3",
        .mode = COMMAND_ANY,
        .help = "vcm3 flash command group",
        .usage = "",
        .chain = vcm3_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

struct flash_driver vcm3_flash = {
    .name               = "vcm3",
    .commands           = vcm3_command_handlers,
    .flash_bank_command = vcm3_flash_bank_command,
    .info               = vcm3_info,
    .erase              = vcm3_erase,
    .protect            = vcm3_protect,
    .write              = vcm3_write,
    .read               = default_flash_read,
    .probe              = vcm3_probe,
    .auto_probe         = vcm3_auto_probe,
    .erase_check        = default_flash_blank_check,
    .protect_check      = vcm3_protect_check,
    .free_driver_priv   = default_flash_free_driver_priv,
};
