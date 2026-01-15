#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_swm.h"
#include "fsl_power.h"
#include "fsl_adc.h"

// Canal de ADC para el potenciómetro
#define ADC_POT_CH     8      // Pin PIO0_18 es ADC Channel 8
#define LED_BLUE       2      // Pin del LED azul

// Función para generar un retardo en milisegundos
void delay_ms(uint32_t ms)
{
    uint32_t count = ms * (CLOCK_GetFreq(kCLOCK_CoreSysClk) / 1000U);
    while (count--)
      {
          __NOP(); // No operation
      }
}

int main(void) {
    // Estructura de configuración para salida
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 1 };

    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, LED_BLUE, &config);

    // Inicialización de clock
    BOARD_BootClockFRO30M();
    BOARD_InitDebugConsole();

    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN8, true);
    CLOCK_DisableClock(kCLOCK_Swm);
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1);

    // Prendo el ADC
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);

    // Obtengo frecuencia deseada y calibro el ADC
	uint32_t frequency = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
	ADC_DoSelfCalibration(ADC0, frequency);


	adc_config_t adc_config;
	ADC_GetDefaultConfig(&adc_config);
	ADC_Init(ADC0, &adc_config);

	adc_conv_seq_config_t adc_sequence = {
		.channelMask = 1 << ADC_POT_CH,                         // Elijo el canal 8 para el potenciómetro
		.triggerMask = 0,                                       // No hay trigger por hardware
		.triggerPolarity = kADC_TriggerPolarityPositiveEdge,     // Flanco ascendente
		.enableSyncBypass = false,                              // Sin bypass de trigger
		.interruptMode = kADC_InterruptForEachConversion        // Interrupciones para cada conversión
	};
	ADC_SetConvSeqAConfig(ADC0, &adc_sequence);
	ADC_EnableConvSeqA(ADC0, true);

    while(1) {
    	adc_result_info_t adc_info;

        // Inicio la conversión
    	ADC_DoSoftwareTriggerConvSeqA(ADC0);
    	while(!ADC_GetChannelConversionResult(ADC0, ADC_POT_CH, &adc_info));
        uint32_t delay_time = 2 + (adc_info.result * 98) / 4095;
        GPIO_PinWrite(GPIO, 1, LED_BLUE, !GPIO_PinRead(GPIO, 1, LED_BLUE));
        delay_ms(delay_time);
    }

    return 0;
}
