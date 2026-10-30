#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <string.h>

#include <math.h>
#include <stdio.h>
#include "shared_function_h.h"

#if defined(TEST_SIMPLE_RX_ANCHOR)

extern void test_run_info(unsigned char *data);

#define APP_NAME "SIMPLE RX Anchor v0.5"

static dwt_config_t config = {
    5,
    DWT_PLEN_256,
    DWT_PAC32,
    9,
    9,
    1,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (257 + 8 - 32),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define ALL_MSG_COMMON_LEN 10

#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

static uint32_t status_reg = 0;

#define POLL_RX_TIMEOUT_UUS 1400
#define POLL_RX_TO_RESP_TX_DLY_UUS 900

#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

#define FINAL_RX_TIMEOUT_UUS 220

#define PRE_TIMEOUT 5

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static double tof;
static double distance;

extern dwt_txconfig_t txconfig_options;

static dwt_rxdiag_t rx_diag;

static const float rx_power_correction_db[6] = { 0, 0, 0, 0, 0, 0 }; // Can be customized

#define LOG_CONSTANT_C0    63.2
#define LOG_CONSTANT_D0_E0 51.175
#define ALPHA_PRF_64       120.7

void pvUartDistance(uint8_t* uart_buffer, uint16_t* offset, float dist);
void pvUartReceiveSignalPowerLevel(uint8_t* uart_buffer, uint16_t* offset, float rsl);
void pvUartFPPowerLevel(uint8_t* uart_buffer, uint16_t* offset, float fsl);
void pvUartSequnecNumber(uint8_t* uart_buffer, uint16_t* offset, uint8_t seq_nb);
void pvUartAntennaNumber(uint8_t* uart_buffer, uint16_t* offset, uint8_t ant_nb);
void logic_gpio_p0_ex02a(int pin_logic, int pin);
void init_gpio_p0_ex02a(uint32_t pin_num);

#define V1 11
#define V2 13
#define V3 15

int simple_rx_anchor(void)
{
    init_gpio_p0_ex02a(V1);
    init_gpio_p0_ex02a(V2);
    init_gpio_p0_ex02a(V3);

    uint8_t D;
    float ip_f1, ip_f2, ip_f3, ip_n = 0;
    float ip_alpha, ip_cp, ip_rsl, ip_fsl = 0;
    float log_constant = 0;

    uint32_t dev_id;
    uint8_t seq_nb = 0;
    uint8_t uart_buffer[1000] = {0};
    uint16_t offset = 0;

    ip_alpha = -(ALPHA_PRF_64 + 1);
    uint8_t ant_nb = 0;

    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_fastrate();

    reset_DWIC();
    Sleep(2);
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    dev_id = dwt_readdevid();

    while (!dwt_checkidlerc()) { }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_configciadiag(1);
    if ((dev_id == (uint32_t)DWT_DW3000_DEV_ID) || (dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID))
    {
        log_constant = LOG_CONSTANT_C0;
    }
    else
    {
        log_constant = LOG_CONSTANT_D0_E0;
    }

    uart0_init();
    while (1)
    {
        dwt_setpreambledetecttimeout(0);

        dwt_setrxtimeout(POLL_RX_TIMEOUT_UUS);

        logic_gpio_p0_ex02a((ant_nb & 0x01)? 1:0, V3);
        logic_gpio_p0_ex02a((ant_nb & 0x02)? 1:0, V2);
        logic_gpio_p0_ex02a((ant_nb & 0x04)? 1:0, V1);

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t frame_len;

            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            frame_len = dwt_getframelength();
            if (frame_len >= sizeof(rx_poll_msg) && frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            else
            {
                goto next_antenna;
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t resp_tx_time;
                int ret;

                poll_rx_ts = get_rx_timestamp_u64();

                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                dwt_setpreambledetecttimeout(PRE_TIMEOUT);

                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
                ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                if (ret == DWT_ERROR)
                {
                    goto next_antenna;
                }

                waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

                frame_seq_nb++;

                if (status_reg & DWT_INT_RXFCG_BIT_MASK)
                {
                    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

                    frame_len = dwt_getframelength();
                    if (frame_len >= sizeof(rx_final_msg) && frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }
                    else
                    {
                        goto next_antenna;
                    }

                    seq_nb = rx_buffer[ALL_MSG_SN_IDX];
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64_t tof_dtu;

                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                        resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                        final_rx_ts_32 = (uint32_t)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;

                        dwt_readdiagnostics(&rx_diag);

                        D = dwt_get_dgcdecision() * 6;

                        ip_n = rx_diag.ipatovAccumCount;
                        ip_f1 = rx_diag.ipatovF1 / 4;
                        ip_f2 = rx_diag.ipatovF2 / 4;
                        ip_f3 = rx_diag.ipatovF3 / 4;
                        ip_cp = rx_diag.ipatovPower;

                        ip_n *= ip_n;
                        ip_f1 *= ip_f1;
                        ip_f2 *= ip_f2;
                        ip_f3 *= ip_f3;
                        ip_fsl = 10 * log10(((ip_f1 + ip_f2 + ip_f3) / ip_n)) + ip_alpha + D - rx_power_correction_db[ant_nb];
                        ip_rsl = 10 * log10((float)ip_cp / ip_n) + ip_alpha + log_constant + D - rx_power_correction_db[ant_nb];

                        pvUartSequnecNumber(uart_buffer, &offset, seq_nb);
                        pvUartAntennaNumber(uart_buffer, &offset, ant_nb);
                        pvUartDistance(uart_buffer, &offset, distance);
                        pvUartFPPowerLevel(uart_buffer, &offset, ip_fsl);
                        pvUartReceiveSignalPowerLevel(uart_buffer, &offset, ip_rsl);
                        offset += sprintf((char *)&uart_buffer[offset], "\n");
                        uart0_strsend((uint8_t *)uart_buffer);

                        memset(uart_buffer, 0, sizeof(uart_buffer));
                        offset = 0;
                    }
                }
            }
        }
next_antenna:
        dwt_forcetrxoff();
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | DWT_INT_TXFRS_BIT_MASK);
        ant_nb = (ant_nb + 1) % 6;
    }
}

void pvUartDistance(uint8_t* uart_buffer, uint16_t* offset, float dist){
    *offset += sprintf((char*)(uart_buffer + *offset ), "D:%f|", dist);
}

void pvUartReceiveSignalPowerLevel(uint8_t* uart_buffer, uint16_t* offset, float rsl){
    *offset += sprintf((char*)(uart_buffer + *offset ), "RSL:%f|", rsl);
}

void pvUartFPPowerLevel(uint8_t* uart_buffer, uint16_t* offset, float fsl){
    *offset += sprintf((char*)(uart_buffer + *offset ), "FSL:%f|", fsl);
}

void pvUartSequnecNumber(uint8_t* uart_buffer, uint16_t* offset, uint8_t seq_nb){
    *offset += sprintf((char*)(uart_buffer + *offset ), "NB:%u|", seq_nb);
}
void pvUartAntennaNumber(uint8_t* uart_buffer, uint16_t* offset, uint8_t ant_nb){
    *offset += sprintf((char*)(uart_buffer + *offset ), "AN:%u|", ant_nb);
}

void init_gpio_p0_ex02a(uint32_t pin_num){
    NRF_GPIO_Type * reg = ((NRF_GPIO_Type*)NRF_P0_BASE);
    reg->PIN_CNF[pin_num] = ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
                              |((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                              |((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                              |((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                              |((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void logic_gpio_p0_ex02a(int pin_logic, int pin)
{
  NRF_GPIO_Type * reg = ((NRF_GPIO_Type*)NRF_P0_BASE);
  if(pin_logic == 1)
    reg->OUTSET = (1UL << pin);
  else
    reg->OUTCLR = (1UL << pin);
}

#endif
