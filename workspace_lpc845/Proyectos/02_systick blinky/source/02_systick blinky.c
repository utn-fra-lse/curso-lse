#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

// Definir pines para los LEDs
#define LED_BLUE 2  // LED azul en el pin 2
#define LED_D1   29  // LED D1 en el pin 3

// Variables globales para el control del parpadeo
volatile uint32_t msTicks = 0;         // Contador de milisegundos
volatile uint32_t ledBlueTicks = 0;     // Contador para el LED azul
volatile uint32_t ledD1Ticks = 0;       // Contador para el LED D1

// Función que se ejecuta en la interrupción de SysTick
void SysTick_Handler(void) {
    msTicks++;  // Incrementa el contador global de tiempo

    // Control del LED azul (cada 500 ms)
    if (msTicks - ledBlueTicks >= 500) {
        GPIO_PortToggle(GPIO, 1U, 1U << LED_BLUE);  // Cambiar estado del LED azul
        ledBlueTicks = msTicks;  // Reiniciar el contador para el LED azul
    }

    // Control del LED D1 (cada 1500 ms)
    if (msTicks - ledD1Ticks >= 1500) {
        GPIO_PortToggle(GPIO, 1U, 1U << LED_D1);  // Cambiar estado del LED D1
        ledD1Ticks = msTicks;  // Reiniciar el contador para el LED D1
    }
}

int main(void) {
    // Inicialización del hardware
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    // Estructura de configuración para los LEDs
    gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0 };  // LEDs apagados al inicio

    // Habilitar el clock del GPIO
    GPIO_PortInit(GPIO, 1);

    // Configurar los pines de los LEDs como salida
    GPIO_PinInit(GPIO, 1, LED_BLUE, &led_config);
    GPIO_PinInit(GPIO, 0, LED_D1, &led_config);

    // Inicialización del temporizador SysTick para generar una interrupción cada 1 ms
    SysTick_Config(SystemCoreClock / 1000);

    // Bucle principal (el parpadeo se maneja en la interrupción de SysTick)
    while (1) {
        // El bucle principal puede estar vacío ya que todo se maneja en la interrupción
    }

    return 0;
}
