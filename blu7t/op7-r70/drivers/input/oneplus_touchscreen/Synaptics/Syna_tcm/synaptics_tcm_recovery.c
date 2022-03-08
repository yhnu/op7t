#include "synaptics_tcm_core.h"

#define SET_UP_RECOVERY_MODE true
#define ENABLE_SYSFS_INTERFACE true

#define IHEX_BUF_SIZE (1024 * 1024)
#define DATA_BUF_SIZE (512 * 1024)
#define IHEX_RECORD_SIZE 14
#define PDT_START_ADDR 0x00e9
#define UBL_FN_NUMBER 0x35
#define F35_CHUNK_SIZE 16
#define F35_CHUNK_SIZE_WORDS 8
#define F35_ERASE_ALL_WAIT_MS 5000
#define F35_ERASE_ALL_POLL_MS 100
#define F35_DATA5_OFFSET 5
#define F35_CTRL3_OFFSET 18
#define F35_RESET_COMMAND 16
#define F35_ERASE_ALL_COMMAND 3
#define F35_WRITE_CHUNK_COMMAND 2
#define F35_READ_FLASH_STATUS_COMMAND 1

struct rmi_pdt_entry {
    unsigned char query_base_addr;
    unsigned char command_base_addr;
    unsigned char control_base_addr;
    unsigned char data_base_addr;
    unsigned char intr_src_count:3;
    unsigned char reserved_1:2;
    unsigned char fn_version:2;
    unsigned char reserved_2:1;
    unsigned char fn_number;
} __packed;

struct rmi_addr {
    unsigned short query_base;
    unsigned short command_base;
    unsigned short control_base;
    unsigned short data_base;
};

struct recovery_hcd {
    unsigned char chunk_buf[F35_CHUNK_SIZE + 3];
    unsigned char out_buf[3];
    unsigned char *ihex_buf;
    unsigned char *data_buf;
    unsigned int ihex_size;
    unsigned int ihex_records;
    unsigned int data_entries;
    struct rmi_addr f35_addr;
    struct syna_tcm_hcd *tcm_hcd;
    const struct firmware *fw_entry;
};

static int recovery_device_reset(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char command = F35_RESET_COMMAND;

    retval = syna_tcm_rmi_write(recovery_hcd->tcm_hcd,
            recovery_hcd->f35_addr.control_base + F35_CTRL3_OFFSET,
            &command,
            sizeof(command));
    if (retval < 0) {
        TPD_INFO("Failed to write F$35 command\n");
        return retval;
    }

    msleep(200);

    return 0;
}

static int recovery_add_data_entry(struct recovery_hcd *recovery_hcd, unsigned char data)
{
    if (recovery_hcd->data_entries >= DATA_BUF_SIZE) {
        TPD_INFO("Reached data buffer size limit\n");
        return -EINVAL;
    }

    recovery_hcd->data_buf[recovery_hcd->data_entries++] = data;

    return 0;
}

static int recovery_add_padding(struct recovery_hcd *recovery_hcd, unsigned int *words)
{
    int retval;
    unsigned int padding;

    padding = (F35_CHUNK_SIZE_WORDS - *words % F35_CHUNK_SIZE_WORDS);
    padding %= F35_CHUNK_SIZE_WORDS;

    while (padding) {
        retval = recovery_add_data_entry(recovery_hcd, 0xff);
        if (retval < 0) {
            TPD_INFO("Failed to add data entry\n");
            return retval;
        }

        retval = recovery_add_data_entry(recovery_hcd, 0xff);
        if (retval < 0) {
            TPD_INFO("Failed to add data entry\n");
            return retval;
        }

        (*words)++;
        padding--;
    }

    return 0;
}

static int recovery_parse_ihex(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char colon;
    unsigned char *buf;
    unsigned int addr;
    unsigned int type;
    unsigned int addrl;
    unsigned int addrh;
    unsigned int data0;
    unsigned int data1;
    unsigned int count;
    unsigned int words;
    unsigned int offset;
    unsigned int record;

    words = 0;
    offset = 0;
    buf = recovery_hcd->ihex_buf;
    recovery_hcd->data_entries = 0;

    for (record = 0; record < recovery_hcd->ihex_records; record++) {
        buf[(record + 1) * IHEX_RECORD_SIZE - 1] = 0x00;
        retval = sscanf(&buf[record * IHEX_RECORD_SIZE],
                "%c%02x%02x%02x%02x%02x%02x",
                &colon,
                &count,
                &addrh,
                &addrl,
                &type,
                &data0,
                &data1);
        if (retval != 7) {
            TPD_INFO("Failed to read ihex record\n");
            return -EINVAL;
        }

        if (type == 0x00) {
            if ((words % F35_CHUNK_SIZE_WORDS) == 0) {
                addr = (addrh << 8) + addrl;
                addr += offset;
                addr >>= 4;

                retval = recovery_add_data_entry(recovery_hcd, addr);
                if (retval < 0) {
                    TPD_INFO("Failed to add data entry\n");
                    return retval;
                }

                retval = recovery_add_data_entry(recovery_hcd, addr >> 8);
                if (retval < 0) {
                    TPD_INFO("Failed to add data entry\n");
                    return retval;
                }
            }

            retval = recovery_add_data_entry(recovery_hcd, data0);
            if (retval < 0) {
                TPD_INFO("Failed to add data entry\n");
                return retval;
            }

            retval = recovery_add_data_entry(recovery_hcd, data1);
            if (retval < 0) {
                TPD_INFO("Failed to add data entry\n");
                return retval;
            }

            words++;
        } else if (type == 0x02) {
            retval = recovery_add_padding(recovery_hcd, &words);
            if (retval < 0) {
                TPD_INFO("Failed to add padding\n");
                return retval;
            }

            offset = (data0 << 8) + data1;
            offset <<= 4;
        }
    }

    retval = recovery_add_padding(recovery_hcd, &words);
    if (retval < 0) {
        TPD_INFO("Failed to add padding\n");
        return retval;
    }

    return 0;
}

static int recovery_check_status(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char status;

    retval = syna_tcm_rmi_read(recovery_hcd->tcm_hcd,
            recovery_hcd->f35_addr.data_base,
            &status,
            sizeof(status));
    if (retval < 0) {
        TPD_INFO("Failed to read status\n");
        return retval;
    }

    status = status & 0x1f;

    if (status != 0x00) {
        TPD_INFO("Recovery mode status = 0x%02x\n", status);
        return -EINVAL;
    }

    return 0;
}

static int recovery_write_flash(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char *data_ptr;
    unsigned int chunk_buf_size;
    unsigned int chunk_data_size;
    unsigned int entries_written;
    unsigned int entries_to_write;

    entries_written = 0;

    data_ptr = recovery_hcd->data_buf;

    chunk_buf_size = sizeof(recovery_hcd->chunk_buf);

    chunk_data_size = chunk_buf_size - 1;

    recovery_hcd->chunk_buf[chunk_buf_size - 1] = F35_WRITE_CHUNK_COMMAND;

    while (entries_written < recovery_hcd->data_entries) {
        entries_to_write = F35_CHUNK_SIZE + 2;

        retval = secure_memcpy(recovery_hcd->chunk_buf,
                chunk_buf_size - 1,
                data_ptr,
                recovery_hcd->data_entries - entries_written,
                entries_to_write);
        if (retval < 0) {
            TPD_INFO("Failed to copy chunk data\n");
            return retval;
        }

        retval = syna_tcm_rmi_write(recovery_hcd->tcm_hcd,
                recovery_hcd->f35_addr.control_base,
                recovery_hcd->chunk_buf,
                chunk_buf_size);
        if (retval < 0) {
            TPD_INFO("Failed to write chunk data\n");
            return retval;
        }

        data_ptr += entries_to_write;
        entries_written += entries_to_write;
    }

    retval = recovery_check_status(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to get no error recovery mode status\n");
        return retval;
    }

    return 0;
}

static int recovery_poll_erase_completion(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char status;
    unsigned char command;
    unsigned char data_base;
    unsigned int timeout;

    timeout = F35_ERASE_ALL_WAIT_MS;

    data_base = recovery_hcd->f35_addr.data_base;

    do {
        command = F35_READ_FLASH_STATUS_COMMAND;

        retval = syna_tcm_rmi_write(recovery_hcd->tcm_hcd,
                recovery_hcd->f35_addr.command_base,
                &command,
                sizeof(command));
        if (retval < 0) {
            TPD_INFO("Failed to write F$35 command\n");
            return retval;
        }

        do {
            retval = syna_tcm_rmi_read(recovery_hcd->tcm_hcd,
                    recovery_hcd->f35_addr.command_base,
                    &command,
                    sizeof(command));
            if (retval < 0) {
                TPD_INFO("Failed to read command status\n");
                return retval;
            }

            if (command == 0x00)
                break;

            if (timeout == 0)
                break;

            msleep(F35_ERASE_ALL_POLL_MS);
            timeout -= F35_ERASE_ALL_POLL_MS;
        } while (true);

        if (command != 0 && timeout == 0) {
            retval = -EINVAL;
            goto exit;
        }

        retval = syna_tcm_rmi_read(recovery_hcd->tcm_hcd,
                data_base + F35_DATA5_OFFSET,
                &status,
                sizeof(status));
        if (retval < 0) {
            TPD_INFO("Failed to read flash status\n");
            return retval;
        }

        if ((status & 0x01) == 0x00)
            break;

        if (timeout == 0) {
            retval = -EINVAL;
            goto exit;
        }

        msleep(F35_ERASE_ALL_POLL_MS);
        timeout -= F35_ERASE_ALL_POLL_MS;
    } while (true);

    retval = 0;

exit:
    if (retval < 0) {
        TPD_INFO("Failed to get erase completion\n");
    }

    return retval;
}

static int recovery_erase_flash(struct recovery_hcd *recovery_hcd)
{
    int retval;
    unsigned char command;

    command = F35_ERASE_ALL_COMMAND;

    retval = syna_tcm_rmi_write(recovery_hcd->tcm_hcd,
            recovery_hcd->f35_addr.control_base + F35_CTRL3_OFFSET,
            &command,
            sizeof(command));
    if (retval < 0) {
        TPD_INFO("Failed to write F$35 command\n");
        return retval;
    }

    if (recovery_hcd->f35_addr.command_base) {
        retval = recovery_poll_erase_completion(recovery_hcd);
        if (retval < 0) {
            TPD_INFO("Failed to wait for erase completion\n");
            return retval;
        }
    } else {
        msleep(F35_ERASE_ALL_WAIT_MS);
    }

    retval = recovery_check_status(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to get no error recovery mode status\n");
        return retval;
    }

    return 0;
}

static int recovery_in_ubl_mode(struct recovery_hcd *recovery_hcd)
{
    int retval = 0;
    struct rmi_pdt_entry p_entry;
    retval = syna_tcm_rmi_read(recovery_hcd->tcm_hcd,
            PDT_START_ADDR,
            (unsigned char *)&p_entry,
            sizeof(p_entry));
    if (retval < 0) {
        TPD_INFO("Failed to read PDT entry\n");
        return false;
    }

    if (p_entry.fn_number != UBL_FN_NUMBER) {
        TPD_INFO("Failed to find F$35\n");
        return false;
    }
    return true;
}
static int recovery_get_fw_ihex(struct recovery_hcd *recovery_hcd, char *iHex)
{
    int retval;

    retval = request_firmware(&recovery_hcd->fw_entry, iHex, &recovery_hcd->tcm_hcd->client->dev);
    if (retval < 0) {
        TPD_INFO("Failed to request %s\n", iHex);
        return retval;
    }

    TPD_INFO("ihex file size = %d\n", (unsigned int)recovery_hcd->fw_entry->size);
    secure_memcpy(recovery_hcd->ihex_buf,
                recovery_hcd->fw_entry->size,
                recovery_hcd->fw_entry->data,
                recovery_hcd->fw_entry->size,
                recovery_hcd->fw_entry->size);
    recovery_hcd->ihex_size = recovery_hcd->fw_entry->size;
    recovery_hcd->ihex_records = recovery_hcd->ihex_size / IHEX_RECORD_SIZE;
    return 0;
}

int recovery_do_recovery(struct recovery_hcd *recovery_hcd, char *iHex)
{
    int retval;
    struct rmi_pdt_entry p_entry;

    retval = recovery_get_fw_ihex(recovery_hcd, iHex);
    if (retval < 0) {
        TPD_INFO("Failed to get ihex data\n");
        return retval;
    }

    retval = recovery_parse_ihex(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to parse ihex data\n");
        return retval;
    }

    retval = syna_tcm_rmi_read(recovery_hcd->tcm_hcd,
            PDT_START_ADDR,
            (unsigned char *)&p_entry,
            sizeof(p_entry));
    if (retval < 0) {
        TPD_INFO("Failed to read PDT entry\n");
        return retval;
    }

    if (p_entry.fn_number != UBL_FN_NUMBER) {
        TPD_INFO("Failed to find F$35\n");
        return -ENODEV;
    }

    recovery_hcd->f35_addr.query_base = p_entry.query_base_addr;
    recovery_hcd->f35_addr.command_base = p_entry.command_base_addr;
    recovery_hcd->f35_addr.control_base = p_entry.control_base_addr;
    recovery_hcd->f35_addr.data_base = p_entry.data_base_addr;

    TPD_INFO("Start of recovery\n");

    retval = recovery_erase_flash(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to erase flash\n");
        return retval;
    }

    TPD_INFO("Flash erased\n");

    retval = recovery_write_flash(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to write to flash\n");
        return retval;
    }

    TPD_INFO("Flash written\n");

    retval = recovery_device_reset(recovery_hcd);
    if (retval < 0) {
        TPD_INFO("Failed to do reset\n");
        return retval;
    }

    TPD_INFO("End of recovery\n");

    return 0;
}

int try_to_recovery_ic(struct syna_tcm_hcd *tcm_hcd, char *iHex)
{
    int retval = 0;
    struct recovery_hcd *recovery_hcd;

    recovery_hcd = kzalloc(sizeof(*recovery_hcd), GFP_KERNEL);
    if (!recovery_hcd) {
        TPD_INFO("Failed to allocate memory for recovery_hcd\n");
        return -ENOMEM;
    }

    recovery_hcd->ihex_buf = kzalloc(IHEX_BUF_SIZE, GFP_KERNEL);
    if (!recovery_hcd->ihex_buf) {
        TPD_INFO("Failed to allocate memory for recovery_hcd->ihex_buf\n");
        retval = -ENOMEM;
        goto err_allocate_ihex_buf;
    }

    recovery_hcd->data_buf = kzalloc(DATA_BUF_SIZE, GFP_KERNEL);
    if (!recovery_hcd->data_buf) {
        TPD_INFO("Failed to allocate memory for recovery_hcd->data_buf\n");
        retval = -ENOMEM;
        goto err_allocate_data_buf;
    }

    recovery_hcd->out_buf[0] = CMD_REBOOT_TO_ROM_BOOTLOADER;
    recovery_hcd->out_buf[1] = 0;
    recovery_hcd->out_buf[2] = 0;
    recovery_hcd->tcm_hcd = tcm_hcd;

    if (!recovery_in_ubl_mode(recovery_hcd)) {
        TPD_INFO("not in ubl mode, goto normal fw update process\n");
        retval = 0;
        goto exit;
    }

    recovery_do_recovery(recovery_hcd, iHex);

    retval = 1;
exit:
    kfree(recovery_hcd->data_buf);
err_allocate_data_buf:
    kfree(recovery_hcd->ihex_buf);
err_allocate_ihex_buf:
    kfree(recovery_hcd);
    recovery_hcd = NULL;

    return retval;
}
