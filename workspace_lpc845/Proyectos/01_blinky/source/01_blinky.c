#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

// Etiqueta para el LED azul
#define LED_BLUE	2

/*
 * @brief   Application entry point.
*/
int main(void) {
	// Inicializacion
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    // Estructura de configuracion para salida
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 1 };
    // Habilito el clock del GPIO 1
    GPIO_PortInit(GPIO, 1);
    // Configuro el pin 0 del GPIO 1 como salida
    GPIO_PinInit(GPIO, 1, LED_BLUE, &config);

    while(1) {
    	// Cambio el estado anterior del LED azul
    	GPIO_PinWrite(GPIO, 1, LED_BLUE, !GPIO_PinRead(GPIO, 1, LED_BLUE));
    	// Demora
    	for(uint32_t i = 0; i < 1000000; i++);
    }
    return 0;
}
