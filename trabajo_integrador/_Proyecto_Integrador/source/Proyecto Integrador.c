#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_swm.h"
#include "fsl_power.h"
#include "fsl_adc.h"
#include "fsl_sctimer.h"
#include "fsl_i2c.h"
#include "FreeRTOS.h"
#include "task.h"

// Dirección del BH1750
#define BH1750_ADDR 0x5c

// Frecuencia del PWM
#define PWM_FREQ 1000

// Lux máximo para el 100%
#define LUX_MAX 2000

// Setpoint mínimo y máximo (en porcentaje)
#define SETPOINT_MIN 25
#define SETPOINT_MAX 75

// Pines de botones
#define S1_PIN 16
#define S2_PIN 25

// Canal de ADC para el potenciómetro
#define ADC_POT_CH 0

// Variables globales
volatile uint8_t setpoint = 50;  // Inicializa el setpoint en 50%
volatile uint16_t adc_result = 0;  // Resultado del ADC
volatile uint32_t elapsedTime = 0;  // Tiempo transcurrido en ms

// Prototipos de funciones
void InitBH1750(void);
float ReadLux(void);

//Inicializa el I2C y el sensor BH1750
void InitBH1750(void) {
    CLOCK_Select(kI2C1_Clk_From_MainClk);
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26);
    CLOCK_DisableClock(kCLOCK_Swm);

    i2c_master_config_t config;
    I2C_MasterGetDefaultConfig(&config);
    config.baudRate_Bps = 400000;
    I2C_MasterInit(I2C1, &config, SystemCoreClock);

    if (I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
        uint8_t cmd = 0x01;  // Comando de encendido
        I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
        I2C_MasterStop(I2C1);
    }
    if (I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
        uint8_t cmd = 0x10;  // Modo de medición continua
        I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
        I2C_MasterStop(I2C1);
    }
}
//Lee el valor de luz del sensor BH1750
float ReadLux(void) {
    uint8_t res[2] = {0};
    if (I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Read) == kStatus_Success) {
        I2C_MasterReadBlocking(I2C1, res, 2, kI2C_TransferDefaultFlag);
        I2C_MasterStop(I2C1);
    }
    return ((res[0] << 8) + res[1]) / 1.2;
}
//Tarea de FreeRTOS para leer el sensor de lux, el potenciómetro y actualizar el PWM
void LuxSensorTask(void *pvParameters) {
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT4, kSWM_PortPin_P0_29);
    CLOCK_DisableClock(kCLOCK_Swm);

    uint32_t sctimer_clock = CLOCK_GetFreq(kCLOCK_Fro);
    sctimer_config_t sctimer_config;
    SCTIMER_GetDefaultConfig(&sctimer_config);
    SCTIMER_Init(SCT0, &sctimer_config);

    sctimer_pwm_signal_param_t pwm_config = {
        .output = kSCTIMER_Out_4,
        .level = kSCTIMER_LowTrue,
        .dutyCyclePercent = 0
    };

    uint32_t event;
    SCTIMER_SetupPwm(SCT0, &pwm_config, kSCTIMER_CenterAlignedPwm, PWM_FREQ, sctimer_clock, &event);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    while (1) {
        float lux = ReadLux();
        uint8_t dutyCycle = (lux * 100) / (LUX_MAX * setpoint / 100);
        if (dutyCycle > 100) dutyCycle = 100;

        // Leer valor del ADC del potenciómetro
        adc_result_info_t adc_info;
        ADC_DoSoftwareTriggerConvSeqA(ADC0);
        while (!ADC_GetChannelConversionResult(ADC0, ADC_POT_CH, &adc_info));
        adc_result = adc_info.result;
        uint32_t potValue = (adc_result * 100) / 4095;

        SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_4, (dutyCycle * potValue) / 100, event);

        vTaskDelay(pdMS_TO_TICKS(1000));
        elapsedTime += 1000;
    }
}
//Tarea para manejar los botones de ajuste del setpoint
void ButtonTask(void *pvParameters) {
    gpio_pin_config_t button_config = {kGPIO_DigitalInput, 0};
    GPIO_PinInit(GPIO, 0, S1_PIN, &button_config);
    GPIO_PinInit(GPIO, 0, S2_PIN, &button_config);
    while (1) {
        if (GPIO_PinRead(GPIO, 0, S1_PIN) == 0) {
            if (setpoint < SETPOINT_MAX) {
                setpoint++;
            }
            vTaskDelay(pdMS_TO_TICKS(300));  // Aumentar el tiempo de debounce
        }

        if (GPIO_PinRead(GPIO, 0, S2_PIN) == 0) {
            if (setpoint > SETPOINT_MIN) {
                setpoint--;
            }
            vTaskDelay(pdMS_TO_TICKS(300));  // Aumentar el tiempo de debounce
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Pequeña espera para reducir la carga del CPU
    }
}
//Tarea para imprimir datos en consola
void PrintDataTask(void *pvParameters) {
    while (1) {
        PRINTF("Tiempo transcurrido: %d ms | Lux: %d%% | Setpoint: %d%% | Brillo LED: %d%%\r\n",
               elapsedTime, (uint16_t)(ReadLux() * 100 / LUX_MAX), setpoint, (adc_result * 100) / 4095);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//Programa principal
int main(void) {
    BOARD_BootClockFRO24M();
    BOARD_InitDebugConsole();
    InitBH1750();

    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN0, true);
    CLOCK_DisableClock(kCLOCK_Swm);

    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1);
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);
    uint32_t frequency = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
    ADC_DoSelfCalibration(ADC0, frequency);

    adc_config_t adc_config;
    ADC_GetDefaultConfig(&adc_config);
    ADC_Init(ADC0, &adc_config);

    adc_conv_seq_config_t adc_sequence = {
        .channelMask = 1 << ADC_POT_CH,
        .triggerMask = 0,
        .triggerPolarity = kADC_TriggerPolarityPositiveEdge,
        .enableSyncBypass = false,
        .interruptMode = kADC_InterruptForEachConversion
    };
    ADC_SetConvSeqAConfig(ADC0, &adc_sequence);
    ADC_EnableConvSeqA(ADC0, true);

    xTaskCreate(LuxSensorTask, "Lux Sensor Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ButtonTask, "Button Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(PrintDataTask, "Print Data Task", configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 3, NULL);

    vTaskStartScheduler();
}
