#include "board.h"
#include "fsl_swm.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"

// Direccion del BH1750
#define BH1750_ADDR	0x5c

/**
 * @brief Programa principal
 */
int main(void) {
	// Arranco de 30 MHz
	BOARD_BootClockFRO30M();

	// Inicializo el clock del I2C1
	CLOCK_Select(kI2C1_Clk_From_MainClk);
    // Asigno las funciones de I2C1 a los pines 26 y 27
	CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26);
    CLOCK_DisableClock(kCLOCK_Swm);

    // Configuracion de master para el I2C con 400 KHz de clock
    i2c_master_config_t config;
    I2C_MasterGetDefaultConfig(&config);
    config.baudRate_Bps = 400000;
    // Usa el clock del sistema de base para generar el de la comunicacion
    I2C_MasterInit(I2C1, &config, SystemCoreClock);

	if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
		// Comando de power on
		uint8_t cmd = 0x01;
		I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
		I2C_MasterStop(I2C1);
	}
	if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
		// Comando de medicion continua a 1 lux de resolucion
		uint8_t cmd = 0x10;
		I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
		I2C_MasterStop(I2C1);
	}

	while(1) {
		// Lectura del sensor
		if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Read) == kStatus_Success) {
			// Resultado
			uint8_t res[2] = {0};
			I2C_MasterReadBlocking(I2C1, res, 2, kI2C_TransferDefaultFlag);
			I2C_MasterStop(I2C1);
			// Devuelvo el resultado
			float lux = ((res[0] << 8) + res[1]) / 1.2;
			PRINTF("LUX : %d \r\n",(uint16_t) lux);
		}
    }
    return 0;
}
