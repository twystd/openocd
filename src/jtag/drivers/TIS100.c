/***************************************************************************
 *   Copyright (C) 2022 by twyst                                           *
 *   info@twyst.co.za                                                      *
 *                                                                         *
 *   Based on: picoprobe.c                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/swd.h>

#include <helper/command.h>
#include <jtag/commands.h>

#include "libusb_helper.h"

#define VID 0x2E8A /* Raspberry Pi */
#define PID 0x0004 /* Picoprobe */

#define BULK_EP_OUT 4
#define BULK_EP_IN 5
#define TIS100_INTERFACE 2

#define TIS100_MAX_PACKET_LENGTH 512
#define LIBUSB_TIMEOUT 10000

struct TIS100 {
    struct libusb_device_handle *usb_handle;
    uint8_t *packet_buffer;
    int freq;
};

static struct swd_cmd_queue_entry {
    uint8_t cmd;
    uint32_t *dst;
    uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} * swd_cmd_queue;

static size_t swd_cmd_queue_length;
static size_t swd_cmd_queue_alloced;
static int queued_retval;

static uint32_t TIS100_mux_address;
static struct TIS100 *TIS100_handle;

static int TIS100_init(void);
static int TIS100_quit(void);
static int TIS100_usb_open(void);
static void TIS100_usb_close(void);

enum PROBE_CMDS {
    PROBE_INVALID = 0,      // Invalid
    PROBE_WRITE_BITS = 1,   // Host wants us to write bits
    PROBE_READ_BITS = 2,    // Host wants us to read bits
    PROBE_SET_FREQ = 3,     // Set TCK freq
    PROBE_RESET = 4,        //
    PROBE_TARGET_RESET = 5, // Reset target
    PROBE_TIS100_MUX = 6,   // Set TIS-100 MUX address
};

struct __attribute__((__packed__)) probe_cmd_hdr {
    uint8_t id;
    uint8_t cmd;
    uint32_t bits;
};

struct __attribute__((__packed__)) probe_pkt_hdr {
    uint32_t total_packet_length;
};

/* Separate queue to swd_cmd_queue because we sometimes insert idle cycles not described
 * there */
#define TIS100_QUEUE_SIZE 64
static struct TIS100_queue_entry {
    uint8_t id;
    uint8_t cmd; /* PROBE_CMDS */
    unsigned bits;
    unsigned offset;
    const uint8_t *buf;
} * TIS100_queue;
static size_t TIS100_queue_length;
static size_t TIS100_queue_alloced;

const char *TIS100_serial_number = NULL;

static int TIS100_init(void) {
    LOG_INFO("... TIS-100 init");

    TIS100_handle = malloc(sizeof(struct TIS100));
    if (TIS100_handle == NULL) {
        LOG_ERROR("Failed to allocate memory");
        return ERROR_FAIL;
    }

    if (TIS100_usb_open() != ERROR_OK) {
        LOG_ERROR("Can't find a TIS100 device! Please check device connections and permissions.");
        return ERROR_JTAG_INIT_FAILED;
    }

    /* Allocate packet buffers and queues */
    TIS100_handle->packet_buffer = malloc(TIS100_MAX_PACKET_LENGTH);
    if (TIS100_handle->packet_buffer == NULL) {
        LOG_ERROR("Failed to allocate memory for the packet buffer");
        return ERROR_FAIL;
    }

    TIS100_queue_alloced = TIS100_QUEUE_SIZE;
    TIS100_queue_length = 0;
    TIS100_queue = malloc(TIS100_queue_alloced * sizeof(*TIS100_queue));
    if (TIS100_queue == NULL)
        return ERROR_FAIL;

    swd_cmd_queue_alloced = 10;
    swd_cmd_queue = malloc(swd_cmd_queue_alloced * sizeof(*swd_cmd_queue));

    return swd_cmd_queue != NULL ? ERROR_OK : ERROR_FAIL;
}

static int TIS100_quit(void) {
    TIS100_usb_close();
    return ERROR_OK;
}

static inline unsigned packet_length(uint8_t *pkt) {
    return pkt - TIS100_handle->packet_buffer;
}

static int TIS100_bulk_write(struct probe_pkt_hdr *pkt_hdr, uint8_t *pkt) {
    pkt_hdr->total_packet_length = packet_length(pkt);
    assert(pkt_hdr->total_packet_length <= TIS100_MAX_PACKET_LENGTH);
    int ret = 0;
    jtag_libusb_bulk_write(TIS100_handle->usb_handle,
                           BULK_EP_OUT, (char *)TIS100_handle->packet_buffer, packet_length(pkt), LIBUSB_TIMEOUT,
                           &ret);
    if (ret < 0)
        return ERROR_JTAG_DEVICE_ERROR;

    return ERROR_OK;
}

static int TIS100_flush(void) {
    LOG_DEBUG_IO("Flush %d transactions", (int)TIS100_queue_length);
    int ret = ERROR_OK;

    struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr *)TIS100_handle->packet_buffer;

    /* Chain pending write and read commands together */
    uint8_t *pkt = TIS100_handle->packet_buffer + sizeof(struct probe_pkt_hdr);

    unsigned total_reads = 0;
    unsigned total_read_bytes = 0;

    for (unsigned i = 0; i < TIS100_queue_length; i++) {
        /* Copy header regardless of read or write */
        struct TIS100_queue_entry *q = &TIS100_queue[i];
        struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr *)pkt;
        if (q->id != i) {
            LOG_ERROR("Wrong queue id. q->id %d != %d", q->id, i);
            return ERROR_JTAG_DEVICE_ERROR;
        }
        hdr->id = q->id;
        hdr->cmd = q->cmd;
        hdr->bits = q->bits;
        pkt += sizeof(struct probe_cmd_hdr);
        unsigned length_bytes = DIV_ROUND_UP(q->bits, 8);

        if (q->cmd == PROBE_WRITE_BITS) {
            /* Copy the data to write into the packet buffer */
            if (q->buf) {
                bit_copy(pkt, 0, q->buf, q->offset, q->bits);
            } else {
                /* Make sure the packet buffer is zerod to clock zeros */
                assert(q->offset == 0);
                memset(pkt, 0, length_bytes);
            }
            pkt += length_bytes;
        } else if (q->cmd == PROBE_READ_BITS) {
            /* Nothing to do for a read as we have already copied the header
             * Will process the data later in one go */
            total_reads++;
            total_read_bytes += length_bytes;
        } else {
            /* Unexpected cmd to flush */
            return ERROR_FAIL;
        }
    }

    /* Send all read/write commands + write data */
    ret = TIS100_bulk_write(pkt_hdr, pkt);
    if (ret < 0)
        return ERROR_JTAG_DEVICE_ERROR;

    /* If no reads we can bail */
    if (total_reads == 0) {
        TIS100_queue_length = 0;
        return ret;
    }

    /* Now get any read responses */
    unsigned rx_pkt_len = sizeof(struct probe_pkt_hdr) +
                          (sizeof(struct probe_cmd_hdr) * total_reads) +
                          total_read_bytes;
    jtag_libusb_bulk_read(TIS100_handle->usb_handle,
                          BULK_EP_IN | LIBUSB_ENDPOINT_IN, (char *)TIS100_handle->packet_buffer,
                          rx_pkt_len, LIBUSB_TIMEOUT, &ret);

    if (ret < 0)
        return ERROR_JTAG_DEVICE_ERROR;

    /* Now time to process the rx data */
    LOG_DEBUG_IO("Read %d bytes from probe", ret);

    /* If we didn't get length we expected */
    if ((int)rx_pkt_len != ret)
        return ERROR_JTAG_DEVICE_ERROR;

    struct probe_pkt_hdr *response_hdr = (struct probe_pkt_hdr *)TIS100_handle->packet_buffer;
    if (rx_pkt_len != response_hdr->total_packet_length)
        return ERROR_JTAG_DEVICE_ERROR;

    pkt = TIS100_handle->packet_buffer + sizeof(struct probe_pkt_hdr);

    /* Now go through read responses */
    for (unsigned i = 0; i < total_reads; i++) {
        struct probe_cmd_hdr *read_hdr = (struct probe_cmd_hdr *)pkt;
        pkt += sizeof(struct probe_cmd_hdr);
        unsigned read_bytes = DIV_ROUND_UP(read_hdr->bits, 8);

        if (read_hdr->cmd != PROBE_READ_BITS)
            return ERROR_JTAG_DEVICE_ERROR;

        uint8_t id = read_hdr->id;
        struct TIS100_queue_entry *q = &TIS100_queue[id];
        assert(read_hdr->cmd == q->cmd);
        assert(read_hdr->id == q->id);
        assert(read_hdr->bits == q->bits);
        LOG_DEBUG_IO("Processing read of %d bits", read_hdr->bits);

        /* Copy data back to swd cmd queue */
        memcpy((void *)q->buf, pkt, read_bytes);
        pkt += read_bytes;
    }

    unsigned processed_len = (pkt - TIS100_handle->packet_buffer);
    if (processed_len != rx_pkt_len)
        return ERROR_JTAG_DEVICE_ERROR;

    TIS100_queue_length = 0;

    /* Keep gdb alive */
    keep_alive();

    return ERROR_OK;
}

static int TIS100_read_write_bits(const uint8_t *buf, unsigned offset, unsigned length, uint8_t cmd) {
    if (TIS100_queue_length == TIS100_queue_alloced) {
        LOG_ERROR("TIS100 queue full");
        return ERROR_BUF_TOO_SMALL;
    } else {
        LOG_DEBUG_IO("TIS100 queue len %d -> %d", (int)TIS100_queue_length,
                     (int)TIS100_queue_length + 1);
    }

    struct TIS100_queue_entry *q = &TIS100_queue[TIS100_queue_length];
    q->id = TIS100_queue_length++;
    q->cmd = cmd;
    q->bits = length;
    q->offset = offset;
    q->buf = buf;

    return ERROR_OK;
}

static int TIS100_write_bits(const uint8_t *buf, unsigned offset, unsigned length) {
    LOG_DEBUG_IO("Write %d bits @ offset %d", length, offset);
    return TIS100_read_write_bits(buf, offset, length, PROBE_WRITE_BITS);
}

static int TIS100_read_bits(const uint8_t *buf, unsigned offset, unsigned length) {
    LOG_DEBUG_IO("Read %d bits @ offset %d", length, offset);

    if (TIS100_queue_length == TIS100_queue_alloced)
        return ERROR_BUF_TOO_SMALL;

    return TIS100_read_write_bits(buf, offset, length, PROBE_READ_BITS);
}

static int TIS100_swd_run_queue(void) {
    LOG_DEBUG_IO("Executing %zu queued transactions", swd_cmd_queue_length);
    int retval;

    queued_retval = TIS100_flush();

    if (queued_retval != ERROR_OK) {
        LOG_DEBUG_IO("Skipping due to previous errors: %d", queued_retval);
        goto skip;
    }

    for (size_t i = 0; i < swd_cmd_queue_length; i++) {
        if (0 == ((swd_cmd_queue[i].cmd ^ swd_cmd(false, false, DP_TARGETSEL)) &
                  (SWD_CMD_APnDP | SWD_CMD_RnW | SWD_CMD_A32))) {
            /* Targetsel has no ack so force it */
            buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3, SWD_ACK_OK);
        }

        LOG_DEBUG_IO("trn_ack_data_parity_trn:");
        for (size_t y = 0; y < sizeof(swd_cmd_queue[i].trn_ack_data_parity_trn); y++)
            LOG_DEBUG_IO("BYTE %d 0x%x", (int)y, swd_cmd_queue[i].trn_ack_data_parity_trn[y]);

        int ack = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3);

        LOG_DEBUG_IO("%s %s %s reg %X = %08" PRIx32,
                     ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT"
                                            : ack == SWD_ACK_FAULT  ? "FAULT"
                                                                    : "JUNK",
                     swd_cmd_queue[i].cmd & SWD_CMD_APnDP ? "AP" : "DP",
                     swd_cmd_queue[i].cmd & SWD_CMD_RnW ? "read" : "write",
                     (swd_cmd_queue[i].cmd & SWD_CMD_A32) >> 1,
                     buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn,
                                 1 + 3 + (swd_cmd_queue[i].cmd & SWD_CMD_RnW ? 0 : 1), 32));

        if (ack != SWD_ACK_OK) {
            queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
            goto skip;

        } else if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
            uint32_t data = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3, 32);
            int parity = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 32, 1);

            if (parity != parity_u32(data)) {
                LOG_ERROR("SWD Read data parity mismatch");
                queued_retval = ERROR_FAIL;
                goto skip;
            }

            if (swd_cmd_queue[i].dst != NULL)
                *swd_cmd_queue[i].dst = data;
        }
    }

skip:
    /* Defensive cleanup - seems like a bad idea to have potentially stale pointers sticking around */
    for (size_t i = 0; i < swd_cmd_queue_length; i++)
        swd_cmd_queue[i].dst = NULL;

    swd_cmd_queue_length = 0;
    retval = queued_retval;
    queued_retval = ERROR_OK;

    return retval;
}

static void TIS100_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk) {
    if (swd_cmd_queue_length == swd_cmd_queue_alloced)
        queued_retval = TIS100_swd_run_queue();

    if (queued_retval != ERROR_OK)
        return;

    size_t i = swd_cmd_queue_length++;
    swd_cmd_queue[i].cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

    TIS100_write_bits(&swd_cmd_queue[i].cmd, 0, 8);

    if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
        /* Queue a read transaction */
        swd_cmd_queue[i].dst = dst;

        TIS100_read_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
                         0, 1 + 3 + 32 + 1 + 1);
    } else {
        /* Queue a write transaction */
        TIS100_read_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
                         0, 1 + 3 + 1);

        buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1, 32, data);
        buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(data));

        TIS100_write_bits(swd_cmd_queue[i].trn_ack_data_parity_trn,
                          1 + 3 + 1, 32 + 1);
    }

    /* Insert idle cycles after AP accesses to avoid WAIT */
    if (cmd & SWD_CMD_APnDP) {
        if (ap_delay_clk == 0)
            return;
        LOG_DEBUG("Add %d idle cycles", ap_delay_clk);
        TIS100_write_bits(NULL, 0, ap_delay_clk);
    }
}

static void TIS100_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk) {
    assert(cmd & SWD_CMD_RnW);
    TIS100_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void TIS100_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk) {
    assert(!(cmd & SWD_CMD_RnW));
    TIS100_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int_least32_t TIS100_set_frequency(int_least32_t hz) {
    int ret;
    struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr *)TIS100_handle->packet_buffer;

    /* Assert this is a standalone command with nothing else queued */
    assert(TIS100_queue_length == 0);

    /* Chain writes and read commands together */
    uint8_t *pkt = TIS100_handle->packet_buffer + sizeof(struct probe_pkt_hdr);
    struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr *)pkt;
    hdr->id = 0;
    hdr->cmd = PROBE_SET_FREQ;
    hdr->bits = hz / 1000;
    pkt += sizeof(struct probe_cmd_hdr);

    /* Send all read/write commands + write data */
    ret = TIS100_bulk_write(pkt_hdr, pkt);
    if (ret < 0)
        return ERROR_JTAG_DEVICE_ERROR;

    return hz;
}

static int_least32_t TIS100_speed(int_least32_t hz) {
    int ret = TIS100_set_frequency(hz);

    if (ret < 0) {
        LOG_ERROR("Couldn't set TIS100 speed");
    } else {
        TIS100_handle->freq = ret;
    }

    return ERROR_OK;
}

static int TIS100_khz(int khz, int *jtag_speed) {
    *jtag_speed = khz * 1000;

    return ERROR_OK;
}

static int TIS100_speed_div(int speed, int *khz) {
    *khz = speed / 1000;

    return ERROR_OK;
}

static int TIS100_swd_init(void) {
    return ERROR_OK;
}

static int TIS100_swd_switch_seq(enum swd_special_seq seq) {
    int ret = ERROR_OK;

    switch (seq) {
    case LINE_RESET:
        LOG_DEBUG_IO("SWD line reset");
        ret = TIS100_write_bits(swd_seq_line_reset, 0, swd_seq_line_reset_len);
        break;
    case JTAG_TO_SWD:
        LOG_DEBUG("JTAG-to-SWD");
        ret = TIS100_write_bits(swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
        break;
    case SWD_TO_JTAG:
        LOG_DEBUG("SWD-to-JTAG");
        ret = TIS100_write_bits(swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
        break;
    case DORMANT_TO_SWD:
        LOG_DEBUG("DORMANT-to-SWD");
        ret = TIS100_write_bits(swd_seq_dormant_to_swd, 0, swd_seq_dormant_to_swd_len);
        break;
    case SWD_TO_DORMANT:
        LOG_DEBUG("SWD-to-DORMANT");
        ret = TIS100_write_bits(swd_seq_swd_to_dormant, 0, swd_seq_swd_to_dormant_len);
        break;
    default:
        LOG_ERROR("Sequence %d not supported", seq);
        return ERROR_FAIL;
    }

    return ret;
}

static int TIS100_reset(int trst, int srst) {
    return ERROR_OK;
}

static int TIS100_set_mux_addr(uint32_t address) {
    LOG_INFO("... setting TIS-100 mux address to %d", address);

    LOG_INFO("... TIS100 queue length:    %d", (int)TIS100_queue_length);
    LOG_INFO("...              allocated: %d", (int)TIS100_queue_alloced);

    if (TIS100_queue_alloced == 0) {
        LOG_ERROR("TIS100 queue not initialised");
        return ERROR_FAIL;
    }

    if (TIS100_queue_length == TIS100_queue_alloced) {
        LOG_ERROR("TIS100 queue full");
        return ERROR_BUF_TOO_SMALL;
    }

    int ret;
    struct probe_pkt_hdr *pkt_hdr = (struct probe_pkt_hdr *)TIS100_handle->packet_buffer;

    assert(TIS100_queue_length == 0);

    /* Chain writes and read commands together */
    uint8_t *pkt = TIS100_handle->packet_buffer + sizeof(struct probe_pkt_hdr);
    struct probe_cmd_hdr *hdr = (struct probe_cmd_hdr *)pkt;
    hdr->id = 0;
    hdr->cmd = PROBE_TIS100_MUX;
    hdr->bits = 7;
    pkt += sizeof(struct probe_cmd_hdr);

    /* Send all read/write commands + write data */
    ret = TIS100_bulk_write(pkt_hdr, pkt);
    if (ret < 0) {
        return ERROR_JTAG_DEVICE_ERROR;
    }

    LOG_INFO("... set TIS-100 mux address to %d", address);

    return ERROR_OK;
}

static const struct swd_driver TIS100_swd = {
    .init = TIS100_swd_init,
    .switch_seq = TIS100_swd_switch_seq,
    .read_reg = TIS100_swd_read_reg,
    .write_reg = TIS100_swd_write_reg,
    .run = TIS100_swd_run_queue,
};

static COMMAND_HELPER(handle_serialnum_args, const char **serialNumber) {
    if (CMD_ARGC != 1) {
        LOG_ERROR("%s: need single argument with serial number", CMD_NAME);
        *serialNumber = NULL;
        return ERROR_COMMAND_SYNTAX_ERROR;
    } else {
        *serialNumber = CMD_ARGV[0];
        return ERROR_OK;
    }
}

COMMAND_HANDLER(handle_serialnum_command) {
    const char *serialNumber = NULL;
    int retval = CALL_COMMAND_HANDLER(handle_serialnum_args, &serialNumber);

    if (ERROR_OK == retval) {
        TIS100_serial_number = malloc(strlen(serialNumber) + 1);
        if (TIS100_serial_number) {
            strcpy((char *)TIS100_serial_number, (char *)serialNumber);
            command_print(CMD, "Using serial number : %s", serialNumber);
        }
    }

    return retval;
}

COMMAND_HANDLER(handle_mux_command) {
    uint32_t address;

    if (CMD_ARGC < 1 || CMD_ARGC > 2) {
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

    if (address > 16) {
        LOG_ERROR("Invalid MUX address (%d) - valid range is [0..16]", address);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }

    LOG_INFO("TIS-100 MUX address is %d", address);
    TIS100_set_mux_addr(address);

    return ERROR_OK;
}

static const struct command_registration serialnum_command_handlers[] = {
    {
        .name = "TIS100_serialnum",
        .mode = COMMAND_ANY,
        .handler = handle_serialnum_command,
        .help = "use TIS100 with this serial number",
        .usage = "'serial number'",
    },
    {
        .name = "mux",
        .mode = COMMAND_ANY,
        .handler = handle_mux_command,
        .help = "set the TIS-100 SWD multiplexer address",
        .usage = "address",
    },
    COMMAND_REGISTRATION_DONE,
};

static const char *const TIS100_transports[] = {"swd", NULL};

struct adapter_driver TIS100_adapter_driver = {
    .name = "TIS100",
    .commands = serialnum_command_handlers,
    .transports = TIS100_transports,
    .swd_ops = &TIS100_swd,
    .init = TIS100_init,
    .quit = TIS100_quit,
    .reset = TIS100_reset,
    .speed = TIS100_speed,
    .speed_div = TIS100_speed_div,
    .khz = TIS100_khz,
};

static int TIS100_usb_open(void) {
    const uint16_t vids[] = {VID, 0};
    const uint16_t pids[] = {PID, 0};

    if (jtag_libusb_open(vids, pids, TIS100_serial_number,
                         &TIS100_handle->usb_handle, NULL) != ERROR_OK) {
        LOG_ERROR("Failed to open or find the device");
        return ERROR_FAIL;
    }

    if (libusb_claim_interface(TIS100_handle->usb_handle, TIS100_INTERFACE) != ERROR_OK) {
        LOG_ERROR("Failed to claim TIS100 interface");
        return ERROR_FAIL;
    }

    return ERROR_OK;
}

static void TIS100_usb_close(void) {
    jtag_libusb_close(TIS100_handle->usb_handle);
}
