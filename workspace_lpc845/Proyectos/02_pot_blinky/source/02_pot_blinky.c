#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_swm.h"
#include "fsl_power.h"
#include "fsl_adc.h"
#include "fsl_sctimer.h"

// Canal de ADC para el potenciómetro
#define ADC_POT_CH       0
#define PWM_FREQ         1000

volatile uint16_t adc_result = 0;

/*
 * @brief   Application entry point.
 */
int main(void) {
    BOARD_BootClockFRO24M();
    BOARD_InitDebugConsole();

    // Configuración del pin de ADC para el potenciómetro
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN0, true);
    CLOCK_DisableClock(kCLOCK_Swm);

    // Configuración de reloj para el ADC
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1);

    POWER_DisablePD(kPDRUNCFG_PD_ADC0);
    uint32_t frequency = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
    ADC_DoSelfCalibration(ADC0, frequency);

    // Inicialización del ADC
    adc_config_t adc_config;
    ADC_GetDefaultConfig(&adc_config);
    ADC_Init(ADC0, &adc_config);

    // Configuración de secuencia A para el ADC
    adc_conv_seq_config_t adc_sequence = {
        .channelMask = 1 << ADC_POT_CH,
        .triggerMask = 0,
        .triggerPolarity = kADC_TriggerPolarityPositiveEdge,
        .enableSyncBypass = false,
        .interruptMode = kADC_InterruptForEachConversion
    };
    ADC_SetConvSeqAConfig(ADC0, &adc_sequence);
    ADC_EnableConvSeqA(ADC0, true);

    // Configuración de PWM en el LED
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT4, kSWM_PortPin_P0_29);
    CLOCK_DisableClock(kCLOCK_Swm);

    uint32_t sctimer_clock = CLOCK_GetFreq(kCLOCK_Fro);
    sctimer_config_t sctimer_config;
    SCTIMER_GetDefaultConfig(&sctimer_config);
    SCTIMER_Init(SCT0, &sctimer_config);

    sctimer_pwm_signal_param_t pwm_config = {
        .output = kSCTIMER_Out_4,
        .level = kSCTIMER_HighTrue,
        .dutyCyclePercent = 0
    };
    uint32_t event;
    SCTIMER_SetupPwm(SCT0, &pwm_config, kSCTIMER_CenterAlignedPwm, PWM_FREQ, sctimer_clock, &event);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

    while (1) {
        adc_result_info_t adc_info;
        ADC_DoSoftwareTriggerConvSeqA(ADC0);

        while (!ADC_GetChannelConversionResult(ADC0, ADC_POT_CH, &adc_info));
        adc_result = adc_info.result;

        uint32_t duty_cycle = (adc_result * 100) / 4095;
        SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_4, duty_cycle, event);

        PRINTF("El resultado del canal %ld en el PIO1_1 dio %d, Duty Cycle: %d %%\n", adc_info.channelNumber, adc_info.result, duty_cycle);

        for (volatile int i = 0; i < 10000; i++);
    }
}
