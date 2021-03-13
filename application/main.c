#include <stdbool.h>
#include <stdint.h>
#include "custom_board.h"
#include "app_timer.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "nrfx_twim.h"

#define SHT30_RESET 29
#define SHT30_SDA 31
#define SHT30_SCL 30
#define SHT30_ADDR 0x44
#define SHT30_CMD_MEASURE { 0x2C, 0x0D }

// Input range of internal Vdd measurement = (0.6 V)/(1/6) = 3.6 V
// 3.0 volts -> 14486 ADC counts with 14-bit sampling: 4828.8 counts per volt
#define ADC12_COUNTS_PER_VOLT 4551

#define MEASURE_INTERVAL_SEC 600

APP_TIMER_DEF(m_measure_timer);

static ble_gap_adv_params_t m_adv_params;
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; 
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

#pragma pack(push)
#pragma pack(1)
static struct sensor_data_t {
    uint16_t msg_id;
    int32_t temperature;
    int32_t humidity;
    uint16_t battery_voltage;
    uint8_t battery_percentage;
} sensor_data;
#pragma pack(pop)

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0, line_num, p_file_name);
}

static void advertising_init(void)
{
    ble_advdata_t advdata;
    ble_advdata_manuf_data_t manuf_specific_data;

    memset(&m_adv_params, 0, sizeof(m_adv_params));
    memset(&advdata, 0, sizeof(advdata));

    manuf_specific_data.company_identifier = 0xFFFF;
    manuf_specific_data.data.p_data = (uint8_t *) &sensor_data;
    manuf_specific_data.data.size   = sizeof(struct sensor_data_t);

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr        = NULL;
    m_adv_params.filter_policy      = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval           = MSEC_TO_UNITS(25, UNIT_0_625_MS);
    m_adv_params.max_adv_evts       = 3;
    m_adv_params.duration           = 0;
    
    ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);    
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, 4);
    sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
}

static void update_bettery_voltage()
{  
    // https://devzone.nordicsemi.com/f/nordic-q-a/14486/measuring-the-battery-voltage-with-nrf52832
    uint32_t timeout = 1000;
    volatile int16_t buffer[8];

    nrf_saadc_channel_config_t myConfig =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_20US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_ENABLED,
        .pin_p      = NRF_SAADC_INPUT_VDD,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    nrf_saadc_resolution_set((nrf_saadc_resolution_t) 3);   // 3 is 14-bit
    nrf_saadc_oversample_set((nrf_saadc_oversample_t) 2);   // 2 is 4x, about 150uSecs total
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_enable();

    NRF_SAADC->CH[1].CONFIG =
              ((myConfig.resistor_p << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((myConfig.resistor_n << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((myConfig.gain       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((myConfig.reference  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((myConfig.acq_time   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((myConfig.mode       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((myConfig.burst      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[1].PSELN = myConfig.pin_n;
    NRF_SAADC->CH[1].PSELP = myConfig.pin_p;

    nrf_saadc_enable();
    NRF_SAADC->RESULT.PTR = (uint32_t)buffer;
    NRF_SAADC->RESULT.MAXCNT = 1;
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

    while (0 == nrf_saadc_event_check(NRF_SAADC_EVENT_END) && timeout > 0)
    {
        timeout--;
    }
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_disable();

    if (timeout != 0)
    {
        sensor_data.battery_voltage =  ((buffer[0] * 1000L)+(ADC12_COUNTS_PER_VOLT/2)) / ADC12_COUNTS_PER_VOLT;
        sensor_data.battery_percentage = battery_level_in_percent(sensor_data.battery_voltage);  	
    }
    else
    {
        sensor_data.battery_voltage = 0;
        sensor_data.battery_percentage = 0;
    }
}

static void read_sensor()
{            
    nrf_gpio_pin_write(SHT30_RESET, 1);
    nrf_delay_us(1000); // power up time

    const nrfx_twim_t twi = NRFX_TWIM_INSTANCE(0);
    const nrfx_twim_config_t twi_config = {
       .scl                = SHT30_SCL,
       .sda                = SHT30_SDA,
       .frequency          = NRF_TWIM_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit     = false
    };

    nrfx_twim_init(&twi, &twi_config, NULL, NULL);
    nrfx_twim_enable(&twi);

    uint8_t cmd[] = SHT30_CMD_MEASURE;
    nrfx_twim_tx(&twi, SHT30_ADDR, cmd, sizeof(cmd), false);

    uint8_t data[6] = {};
    nrfx_twim_rx(&twi, SHT30_ADDR, data, sizeof(data));
    
    // https://github.com/Sensirion/embedded-sht/blob/master/sht3x/sht3x.c#L89
    sensor_data.temperature = ((21875 * (int32_t)(data[0] << 8 | data[1])) >> 13) - 45000;
    sensor_data.humidity = ((12500 * (int32_t)(data[3] << 8 | data[4])) >> 13);

    nrfx_twim_disable(&twi);
    nrfx_twim_uninit(&twi);
    
    nrf_gpio_pin_write(SHT30_RESET, 0);
}

static void timer_handler(void* p_context)
{    
    sensor_data.msg_id++;
    read_sensor();
    update_bettery_voltage();

    advertising_init();
    sd_ble_gap_adv_start(m_adv_handle, 1); 
}

int main(void)
{
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE); 
    app_timer_init();
    nrf_pwr_mgmt_init();

    uint32_t ram_start = 0;
    nrf_sdh_enable_request();
    nrf_sdh_ble_default_cfg_set(1, &ram_start);
    nrf_sdh_ble_enable(&ram_start);

    sensor_data.msg_id = 0;
    sensor_data.temperature = 0;
    sensor_data.humidity = 0;
    sensor_data.battery_voltage = 0;
    sensor_data.battery_percentage = 0;

    nrf_gpio_cfg_output(SHT30_RESET);
    nrf_gpio_pin_write(SHT30_RESET, 0);
    
    app_timer_create(&m_measure_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    app_timer_start(m_measure_timer, APP_TIMER_TICKS(MEASURE_INTERVAL_SEC * 1000), NULL);

    timer_handler(NULL); // run once immediately after boot
    
    while(true)
    {
        nrf_pwr_mgmt_run();
    }
}