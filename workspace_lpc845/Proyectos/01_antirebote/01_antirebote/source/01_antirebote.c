#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

// Definir el pin del LED rojo y el botón USER
#define LED_RED 2     // El LED Rojo está en el pin 2
#define USER    4     // El botón USER está en el pin 4

// Función de retardo básico
void delay(void) {
    for (volatile uint32_t i = 0; i < 1000000; i++) {
        __asm("NOP");  // No operación para generar el retardo
    }
}

int main(void) {
    // Inicialización del hardware
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    // Estructura de configuración para salida (LED) e entrada (USER)
    gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0 };  // LED apagado al inicio
    gpio_pin_config_t button_config = { kGPIO_DigitalInput, 0 }; // Botón USER como entrada

    // Habilitar el clock del GPIO
    GPIO_PortInit(GPIO, 1);

    // Configurar el LED rojo como salida
    GPIO_PinInit(GPIO, 1, LED_RED, &led_config);

    // Configurar el botón USER como entrada
    GPIO_PinInit(GPIO, 1, USER, &button_config);

    // Bucle principal
    while (1) {
        // Leer el estado del botón USER
        if (GPIO_PinRead(GPIO, 1, USER) == 0) {  // Si el botón está presionado (nivel bajo)
            GPIO_PinWrite(GPIO, 1, LED_RED, 1);  // Encender el LED rojo
        } else {
            GPIO_PinWrite(GPIO, 1, LED_RED, 0);  // Apagar el LED rojo
        }

        delay();  // Retardo para antirebote
    }

    return 0;
}
