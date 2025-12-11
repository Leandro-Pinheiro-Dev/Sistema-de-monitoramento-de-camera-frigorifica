/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "sdkconfig.h"
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "semaphore.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "esp_check.h"
#include <string.h>
#include <nvs_flash.h>
#include "nvs.h"
#include "esp_system.h"
#include <esp_rmaker_core.h>
#include "esp_rmaker_standard_types.h" 
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <app_network.h>
#include <app_insights.h>
#include "app_wifi.h" 
#include "esp_heap_caps.h" 
#include <esp_rmaker_standard_devices.h>
#include "esp_sntp.h"
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

#define REPORTING_PERIOD    10000 /* milisegundos */
#define EXAMPLE_ONEWIRE_BUS_GPIO    4
#define EXAMPLE_ONEWIRE_MAX_DS18B20 1

#define EXAMPLE_PCNT_HIGH_LIMIT  1000
#define EXAMPLE_PCNT_LOW_LIMIT  -1000

#define EXAMPLE_EC11_GPIO_A 12//0
#define EXAMPLE_EC11_GPIO_B 13//2

#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           36//3
#define EXAMPLE_PIN_NUM_SCL           35//4
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              CONFIG_EXAMPLE_SSD1306_HEIGHT
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *sp_sensor_device;
esp_rmaker_device_t *device_status;

SemaphoreHandle_t xSemaphore;

static const char *TAG = "TEMPERATURA";
int8_t valorLimite = 0;
int8_t valorTemp = 0;
int8_t soma = 0;
float current_temperature = 0.0;
float last_temperature = 0.0;
static bool limite_alterado = false;
static TickType_t ultimo_limite_tick = 0;

// Handle for the task to be controlled
TaskHandle_t display_OLED_TaskHandle;
TaskHandle_t rotary_encoder_TaskHandle;
TaskHandle_t ds18b20_TaskHandle;
TaskHandle_t esp_rainmaker_TaskHandle;
TaskHandle_t heap_monitor_TaskHandle;
TaskHandle_t limite_auto_save_TaskHandle;

typedef struct { // Estrutura para enviar dados para o display
    bool atualizaTelaTemperatura;
    bool atualizaTelaLimite;
} atualizaDisplay_t;

static QueueHandle_t filaDisplay;// Fila global para comunicação com a task do display

void sincronizar_hora(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_LOGI("TIME", "Sincronizando hora via NTP...");
    while (timeinfo.tm_year < (2020 - 1900)) {  // Espera até o ano ser >= 2020
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    ESP_LOGI("TIME", "Hora sincronizada: %s", asctime(&timeinfo));
}

void enviar_status_para_app(const char *mensagem)
{
    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_name(device_status, "status"),
        esp_rmaker_str(mensagem)
    );
}

static esp_err_t write_cb_limite(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param, const esp_rmaker_param_val_t val, void *priv_data)
{
    if (val.type == RMAKER_VAL_TYPE_INTEGER) {
        int8_t novo_valor = (int8_t) val.val.i;
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            valorLimite = novo_valor;
            soma = novo_valor;
            xSemaphoreGive(xSemaphore);
        }
        limite_alterado = true;
        ultimo_limite_tick = xTaskGetTickCount();

        ESP_LOGI("RainMaker", "Limite alterado via app: %d", valorLimite);

        // Atualiza o valor mostrado no app (garante sincronismo)
        esp_rmaker_param_update_and_report(param, esp_rmaker_int(valorLimite));

        // Atualiza display imediatamente
        atualizaDisplay_t atualizaTemp = {
            .atualizaTelaTemperatura = false,
            .atualizaTelaLimite = true
        };
        xQueueSend(filaDisplay, &atualizaTemp, 0);

        return ESP_OK;
    }
    return ESP_FAIL;
}

static void salvar_limite_nvs(int8_t sp)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        //nvs_erase_key(nvs, "limite"); // limpa a chave anterior
        nvs_set_i8(nvs, "limite", sp);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Limite %d salvo na NVS.", sp);
    } else {
        ESP_LOGE(TAG, "Erro ao abrir NVS: %s", esp_err_to_name(err));
    }
}

static void ler_limite_nvs(void)
{
    nvs_handle_t nvs;
    int8_t sp_lido;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_i8(nvs, "limite", &sp_lido);
        if (err == ESP_OK) {
            valorLimite = sp_lido;
            ESP_LOGI(TAG, "Limite lido da NVS: %d", valorLimite);
        } else {
            ESP_LOGW(TAG, "Nenhum limite salvo, usando valor padrão: %d", valorLimite);
        }
        nvs_close(nvs);
    }
    else{
        ESP_LOGE(TAG, "Limite não lido da NVS");
    }
}

void display_OLED(void *pvParameter)
{
    atualizaDisplay_t atualizaTemp;

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
    ESP_LOGI(TAG, "Display LVGL Scroll Text");

    lv_obj_t *labelBoot = lv_label_create(lv_scr_act());
    lv_label_set_text(labelBoot, "Inicializando...");
    lv_obj_align(labelBoot, LV_ALIGN_CENTER, 0, 0);
    lv_timer_handler(); // Atualiza a tela imediatamente

    vTaskDelay(pdMS_TO_TICKS(2000));
    lv_obj_clean(lv_scr_act());

    //lv_init();  // Initialize LVGL
    // Screen driver initialization is skipped here for simplicity.
    // In practice, you should register the display driver and set up buffers.
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_t *temp = lv_label_create(lv_scr_act());
    lv_obj_t *limiteVal = lv_label_create(lv_scr_act());
    char sprintfTemperatura[20];
    char sprintfLimite[20];

    while (1) {
        if (xQueueReceive(filaDisplay, &atualizaTemp, portMAX_DELAY) == pdTRUE) {
            // Lock the mutex due to the LVGL APIs are not thread-safe      
            if (lvgl_port_lock(0)) 
            {
                // Protege leitura de current_temperature e valorLimite com mutex (se existir)
                float temp_copy;
                int8_t limite_copy;
                if (xSemaphore != NULL && xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
                    temp_copy = current_temperature;
                    limite_copy = valorLimite;
                    xSemaphoreGive(xSemaphore);
                } else {
                    // fallback sem mutex
                    temp_copy = current_temperature;
                    limite_copy = valorLimite;
                }
                lv_obj_set_pos(label,10,0);
                lv_label_set_text(label, "Temperatura:");

                lv_obj_set_pos(temp,45,20);
                sprintf(sprintfTemperatura, "%.1f°C", temp_copy);
                lv_label_set_text(temp, sprintfTemperatura);
                
                lv_obj_set_pos(limiteVal,20,45);
                sprintf(sprintfLimite, "Limite: %d°C", limite_copy);
                lv_label_set_text(limiteVal, sprintfLimite);

                lv_timer_handler(); // atualiza LVGL
                lvgl_port_unlock();
                ESP_LOGI("DISPLAY", "ATUALIZANDO DISPLAY (temp: %.1f, Limite: %d)", temp_copy, limite_copy);
            } else {
                ESP_LOGW("DISPLAY", "Não conseguiu lock LVGL");
            }
        }
    }
}

void rotary_encoder(void *pvParameter)
{
//    bool atualizaTelaLimite = false;
    atualizaDisplay_t atualizaTemp = {
            .atualizaTelaTemperatura = false,
            .atualizaTelaLimite = false
        };
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    //ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    //ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    //ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    //ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    //ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    //ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Report counter value
    int pulse_count = 0;
    int pulso_atual = 0;
    int pulso_anterior = 0;
    if( xSemaphore != NULL )
    {
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
            soma = valorLimite;
            xSemaphoreGive(xSemaphore);
        }
    }
    while (1)
    {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        pulso_atual = pulse_count;
        //ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        if(pulso_atual > pulso_anterior) //incrementa
        {
            soma++;
            if (soma > 99) soma = 99;
            //ESP_LOGI(TAG, "Pulse count: %d", soma);
            limite_alterado = true;
            ultimo_limite_tick = xTaskGetTickCount();;
            atualizaTemp.atualizaTelaLimite = true;
        }
        else if(pulso_atual < pulso_anterior)//decrementa
        {
            soma--;
            if (soma < -99) soma = -99;
            //ESP_LOGI(TAG, "Pulse count: %d", soma);
            limite_alterado = true;
            ultimo_limite_tick = xTaskGetTickCount();;
            atualizaTemp.atualizaTelaLimite = true;
        }
        if( xSemaphore != NULL )
        {
            if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
                /* The semaphore was created successfully and can be used. */
                valorLimite = soma; //Colocar mutex!
                xSemaphoreGive(xSemaphore);
                // Marca alteração e registra o tempo
                //ultimo_limite_tick = xTaskGetTickCount();
            }
        }
        
        if(atualizaTemp.atualizaTelaLimite == true)
        {
            if (xQueueSend(filaDisplay, &atualizaTemp, 0) != pdPASS) {
                ESP_LOGW("ENCODER", "Fila cheia");
            } else {
                ESP_LOGI("ENCODER", "Solicitou atualização");
            }
            atualizaTemp.atualizaTelaLimite = false;
        }
        vTaskDelay(250 / portTICK_PERIOD_MS);  
        pulso_anterior = pulso_atual;         
    }
}

void ds18b20(void *pvParameter)
{
    atualizaDisplay_t atualizaTemp = {
            .atualizaTelaTemperatura = false,
            .atualizaTelaLimite = false
        };

    // install new 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };

    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d", EXAMPLE_ONEWIRE_BUS_GPIO);

    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
                ds18b20_device_num++;
                if (ds18b20_device_num >= EXAMPLE_ONEWIRE_MAX_DS18B20) {
                    ESP_LOGI(TAG, "Max DS18B20 number reached, stop searching...");
                    break;
                }
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    // set resolution for all DS18B20s
    for (int i = 0; i < ds18b20_device_num; i++) {
        // set resolution
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20s[i], DS18B20_RESOLUTION_12B));
    }

    // get temperature from sensors one by one
    float temperature;
    int current_temperature_int=0;
    int last_temperature_int=0;
    while (1) {
        last_temperature_int = current_temperature_int;
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20s[0]));
        ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[0], &temperature));
//        ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", 0, temperature);
        current_temperature_int = temperature*10;
        current_temperature = (float)(current_temperature_int)/10; //Para limitar em 1 casa decimal
    
        //if (current_temperature > valorLimite) {
        //    ws2812_led_set_rgb(255, 0, 0); // Vermelho
        //} else {
        //    ws2812_led_set_rgb(0, 255, 0); // Verde
       // }

        if(last_temperature_int != current_temperature_int)
        {
            atualizaTemp.atualizaTelaTemperatura = true;
            xQueueSend(filaDisplay, &atualizaTemp, (TickType_t)0);
            ESP_LOGI("DS18B20", "Solicitou atualização de temperatura no display");
            atualizaTemp.atualizaTelaTemperatura = false;
        }
    }
}

void esp_rainmaker(void *pvParameter)
{
    esp_err_t err;
        // Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
    app_network_init();
    
    //sincronizar_hora(); // Utiliza SNTP para sincronizar data e hora

    // Initialize the ESP RainMaker Agent.
    // Note that this should be called after app_network_init() but before app_network_start()
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Sensor DS18B20");
        if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    // Create a device and add the relevant parameters to it
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Sensor DS18B20", NULL, current_temperature);//   temp_sensor_device = esp_rmaker_temp_sensor_device_create("Sensor DS18B20", NULL, app_get_current_temperature());
    esp_rmaker_node_add_device(node, temp_sensor_device);

    sp_sensor_device = esp_rmaker_device_create("Limite", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    esp_rmaker_param_t *param_limite = esp_rmaker_param_create(
        "Limite",
        ESP_RMAKER_PARAM_TEMPERATURE,
        esp_rmaker_int(valorLimite),
        PROP_FLAG_READ | PROP_FLAG_WRITE
    );
    //esp_rmaker_param_add_bounds(param_limite, esp_rmaker_int(-99), esp_rmaker_int(99), esp_rmaker_int(1));
    esp_rmaker_device_add_param(sp_sensor_device, param_limite);
    esp_rmaker_device_assign_primary_param(sp_sensor_device, param_limite);
    esp_rmaker_device_add_cb(sp_sensor_device, write_cb_limite, NULL);
    esp_rmaker_node_add_device(node, sp_sensor_device);

    // Criação do dispositivo de status no RainMaker
    device_status = esp_rmaker_device_create("Status", ESP_RMAKER_DEVICE_SWITCH, NULL);
    esp_rmaker_param_t *param_status = esp_rmaker_param_create(
    "status",
    "esp.param.alarm",
    esp_rmaker_str("Temperatura elevada"),
    PROP_FLAG_READ | PROP_FLAG_WRITE
    );
    esp_rmaker_device_add_param(device_status, param_status);
    esp_rmaker_node_add_device(node, device_status);

    // Enable OTA
    esp_rmaker_ota_enable_default();

    // Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y
    app_insights_enable();

    // Start the ESP RainMaker Agent
    esp_rmaker_start();
    
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
    
    while (1) {
        float temp_copy;
        int8_t limite_copy;
        if (xSemaphore != NULL) {
            if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
                temp_copy = current_temperature;
                limite_copy = valorLimite;
                xSemaphoreGive(xSemaphore);
            } else {
                temp_copy = current_temperature; // fallback
                limite_copy = valorLimite;
            }
        } else {
            temp_copy = current_temperature;
            limite_copy = valorLimite;
        }
        // Só atualiza temperatura periodicamente

        esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE), esp_rmaker_float(temp_copy));

        // Não reenviar limite se ele está sendo alterado
        if (!limite_alterado) {
            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(sp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE), esp_rmaker_int(limite_copy));
            //ESP_LOGE(TAG, "SETPOINT-1: %d", (int8_t)sp_sensor_device);
        }

        if(temp_copy>=limite_copy)
        {
            enviar_status_para_app("Temperatura elevada");
            ESP_LOGE(TAG, "Temperatura elevada");
        }
        else
        {
            enviar_status_para_app("Temperatura normal");
            ESP_LOGE(TAG, "Temperatura normal");
        }
        
        ESP_LOGI(TAG, "Atualizado no RainMaker: %.2f °C", temp_copy);
        vTaskDelay(pdMS_TO_TICKS(REPORTING_PERIOD)); // reporta a cada 10s
    }
}

void heap_monitor_task(void *arg)
{
    while (1) {
        size_t h = esp_get_free_heap_size();
        ESP_LOGI("HEAP_MON", "Free heap: %u bytes", (unsigned)h);
        printf("--- Uso de Memória ---\n");
        printf("Heap Total Livre: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT)); //Heap total livre (todas as regiões de heap)  
        printf("Heap DRAM Livre: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_8BIT)); //Heap DRAM livre (acessível por byte)
        printf("Heap IRAM Livre: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_32BIT)); // Heap IRAM livre (memória executável)
        printf("Heap DMA Livre: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_DMA)); // Heap DMA-capable livre
        #if CONFIG_SPIRAM_SUPPORT // Heap PSRAM livre (se disponível e habilitado)
            printf("Heap PSRAM Livre: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        #endif
        printf("----------------------\n");
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

void limite_auto_save(void *pvParam)
{
    while (1)
    {
//vTaskDelay(pdMS_TO_TICKS(10000));
        if (limite_alterado && ((xTaskGetTickCount() - ultimo_limite_tick) >= 1000)) //maior ou igual a 10 segundos, entrar
        {
            ESP_LOGI(TAG, "SALVANDO SETPOINT NA MEMÓRIA O VALOR DE ----------->>.: %d", valorLimite);
            salvar_limite_nvs(valorLimite);
            limite_alterado = false;
            // Reporta o novo valor confirmado ao RainMaker
            esp_rmaker_param_update_and_report(esp_rmaker_device_get_param_by_type(sp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE), esp_rmaker_int(valorLimite));
            //ESP_LOGE(TAG, "SETPOINT-2: %d", (int8_t)sp_sensor_device);
        }
        //        ESP_LOGE(TAG, "limite_alterado------------------------------->>.: %d", limite_alterado);
        //        ESP_LOGE(TAG, "xTaskGetTickCount()----------------------------->>.: %lu", xTaskGetTickCount());
        //        ESP_LOGE(TAG, "ultimo_limite_tick---------------------------->>.: %lu", ultimo_limite_tick);
        //        ESP_LOGE(TAG, "xTaskGetTickCount() - ultimo_limite_tick------>>.: %lu", (xTaskGetTickCount() - ultimo_limite_tick));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ler_limite_nvs();

    lv_init();  // Initialize LVGL

    filaDisplay = xQueueCreate(10, sizeof(atualizaDisplay_t));// Cria a fila
    if (filaDisplay == NULL) {
        ESP_LOGE("MAIN", "Erro ao criar a fila do display!");
        return;
    }

    xSemaphore = xSemaphoreCreateMutex(); //Cria  Mutex
    if (xSemaphore == NULL) {
        ESP_LOGE("MAIN", "Erro ao criar mutex global!");
        return;
    }

    BaseType_t ret;

    ret = xTaskCreatePinnedToCore(display_OLED, "display_OLED", 4096, NULL, 4, &display_OLED_TaskHandle, 0);
    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar display_OLED task");

    ret = xTaskCreatePinnedToCore(ds18b20, "ds18b20", 4096, NULL, 5, &ds18b20_TaskHandle, 0);
    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar ds18b20 task");

    ret = xTaskCreatePinnedToCore(rotary_encoder, "rotary_encoder", 4096, NULL, 5, &rotary_encoder_TaskHandle, 0);
    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar rotary_encoder task");

    ret = xTaskCreatePinnedToCore(esp_rainmaker, "esp_rainmaker", 4096, NULL, 6, &esp_rainmaker_TaskHandle, 1);
    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar esp_rainmaker task");

    ret = xTaskCreatePinnedToCore(limite_auto_save, "limite_auto_save", 3072, NULL, 4, &limite_auto_save_TaskHandle, 0);
    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar limite_auto_save task");

//    ret = xTaskCreatePinnedToCore(heap_monitor_task, "heap_monitor", 2048, NULL, 7, &heap_monitor_TaskHandle, 1);
//    if (ret != pdPASS) ESP_LOGE("MAIN", "Falha ao criar heap_monitor task");
}

