#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_swm.h"
#include "fsl_power.h"
#include "fsl_adc.h"
#include "fsl_sctimer.h"

// Canal de ADC para el potenciometro
#define ADC_POT_CH      0
#define LED_PWM_OUT     kSCTIMER_Out_0 // Canal de salida PWM

// Variables globales para el resultado del ADC
volatile uint16_t adc_result = 0;

/*
 * @brief   Application entry point.
 */
int main(void) {
    // Inicialización de clock
    BOARD_BootClockFRO24M();
    BOARD_InitDebugConsole();

    // Activo clock de matriz de conmutacion
    CLOCK_EnableClock(kCLOCK_Swm);
    // Configuro la funcion de ADC en el canal del potenciometro
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN0, true);
    SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT0, 1); // Configura el pin de salida para PWM (LED)

    // Desactivo clock de matriz de conmutacion
    CLOCK_DisableClock(kCLOCK_Swm);

    // Elijo clock desde el FRO con divisor de 1 (30MHz)
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1);

    // Prendo el ADC
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);
    // Obtengo frecuencia deseada y calibro ADC
    uint32_t frequency = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
    ADC_DoSelfCalibration(ADC0, frequency);
    // Configuración por defecto del ADC
    adc_config_t adc_config;
    ADC_GetDefaultConfig(&adc_config);
    ADC_Init(ADC0, &adc_config);

    // Configuración para las conversiones
    adc_conv_seq_config_t adc_sequence = {
        .channelMask = 1 << ADC_POT_CH,                         // Canal del potenciometro
        .triggerMask = 0,                                       // Sin trigger por hardware
        .triggerPolarity = kADC_TriggerPolarityPositiveEdge,    // Flanco ascendente
        .enableSyncBypass = false,                              // Sin bypass de trigger
        .interruptMode = kADC_InterruptForEachConversion        // Interrupciones por conversion
    };
    ADC_SetConvSeqAConfig(ADC0, &adc_sequence);
    ADC_EnableConvSeqA(ADC0, true);

    // Configuración del SCTimer para PWM
    sctimer_config_t sctimerConfig;
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    SCTIMER_Init(SCT0, &sctimerConfig);

    // Configuración de PWM con frecuencia de 1kHz
    uint32_t pwmFreq = 1000; // 1 kHz
    uint32_t event;
    SCTIMER_SetupPwm(SCT0, LED_PWM_OUT, pwmFreq, 0, false, &event); // 0% duty cycle inicial

    // Configuración de SysTick para interrupciones cada 1 ms
    SysTick_Config(SystemCoreClock / 1000);

    while (1) {
        // Resultado de conversion
        adc_result_info_t adc_info;
        // Inicio conversion
        ADC_DoSoftwareTriggerConvSeqA(ADC0);
        // Espero a terminar la conversion
        while (!ADC_GetChannelConversionResult(ADC0, ADC_POT_CH, &adc_info));
        // Guardo el resultado en la variable global
        adc_result = adc_info.result;
    }
    return 0;
}

// Manejador de interrupción de SysTick
void SysTick_Handler(void) {
    // Ajuste del ciclo de trabajo de PWM en función de la lectura del potenciómetro
    uint32_t dutyCycle = (adc_result * 100) / 4095; // Normaliza a 0-100%
    SCTIMER_UpdatePwmDutycycle(SCT0, LED_PWM_OUT, dutyCycle, 0); // Actualiza el ciclo de trabajo
}
