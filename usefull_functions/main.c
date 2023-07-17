/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"

#define TEMP130_CAL_ADDR ((int32_t) *((uint16_t *) ((uint32_t) 0x1FF8007E)))
#define TEMP30_CAL_ADDR ((int32_t) *((uint16_t *) ((uint32_t) 0x1FF8007A)))

#define BUF_SIZE 1000
#define x 1		// '1/x' to czas w sekundach pomiedzy kolejnymi obliczeniami temperatury (poniewaz próbkujemy 1000 razy na sekunde, a bufsize = 1000)

volatile int32_t tempBuf[BUF_SIZE];
int32_t g_buf = 0;			// iterator bufora
int32_t g_count = -(BUF_SIZE - (BUF_SIZE / x));	//zabezpiecza pierwszy wydruk (pierwsze '1/x' sekundy)
int32_t g_update = 0;		// flaga informujaca o kolejnym przepelnieniu bufora nowymi pomiarami
int32_t g_temp_new = 0;		// wyswietlic mozna tylko temperatury od 0 do 99
int32_t g_temp_old = 1000; 	//'1000' jest tylko dlatego zeby nie bylo bugu z niewyswietleniem pierwszego pomiaru
int32_t g_sum = 0;			// tutaj przechowywana jest suma pomiarow z bufora
uint8_t liczba_jednosci = 0;
uint8_t liczba_dziesiatek = 0;

#include "przydatne_funkcje.h"

int main(void) {

	//clock & i2c & adc config
	cfg_clk();
	I2C_clock_enable();
	TIM6_clock_enable();
	ADC_clock_enable();
	GPIOB_clock_enable();
	I2C_pin_config();

	//czesc i2c
	I2C_reset();
	I2C_peripherial_disable();	// wszelkie ustawienia timingr przy wylaczonym peryferiale

	I2C_timingr_config();	// timingr

	I2C_peripherial_enable();

	I2C_set_slave_address(0x42);	// (0x43 to expander lewych segmentów, 0x42 - prawych)
	I2C_7bit_transfer_config(0x3);	// wysylamy 3 bajty, bo adres rejestru + dwa bajty segmentów

	//czesc startu wysylania po i2c
	I2C_start();
	I2C_send_byte(0x14);	// adres rejestru do ustawiania bajtów segmentów
	I2C_send_byte(0xAE);	// pierwszy bajt do ustawienia (znak stopnia) 'AE'
	I2C_send_byte(0xD);		// drugi (znak C) 'D'
	I2C_check_transfer_complete();	// czekanie na ustawienie sie bitu transfer complete (pêtla while)
	I2C_stop();

	//adc config
	ADC1_config();
	ADC1_start();
	TIM6_config();
	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	TIM6_counter_enable();

	while (1) {
		if ((0 != g_update)) {
			for (int32_t i = 0; i < BUF_SIZE; i++)
				g_sum += tempBuf[i];

			//obliczenie temperatury i czyszczenie flag i iteratora
			//g_temp_new = (((33 * g_sum / (BUF_SIZE * 30)) - TEMP30_CAL_ADDR) * (int32_t) (130 - 30) / (TEMP130_CAL_ADDR - TEMP30_CAL_ADDR)) + 30; //dokladnosc do liczby jednosci
			g_temp_new = (10*((33 * g_sum / (BUF_SIZE * 30)) - TEMP30_CAL_ADDR) * (int32_t) (130 - 30) / (TEMP130_CAL_ADDR - TEMP30_CAL_ADDR)) + 300 - 22; //dokladnosc do czesci dziesietnej (w debugerze)), odjeto 2.2 stopnia
			g_count = 0;
			g_sum = 0;
			g_update = 0;
			//wyswietlenie temperatury na segmentach (powtorzenie calej procedury i2c, tylko jesli zmienila sie temperatura)
			if (g_temp_new != g_temp_old) {
				g_temp_old = g_temp_new;
				I2C_reset();
				I2C_peripherial_disable();
				I2C_timingr_config();
				I2C_peripherial_enable();
				I2C_set_slave_address(0x43);	// (0x43 to expander lewych segmentów, 0x42 - prawych)
				I2C_7bit_transfer_config(0x3);

				liczba_dziesiatek = (g_temp_new % 1000) / 100;
				liczba_jednosci = (g_temp_new % 100) / 10;

				I2C_start();
				I2C_send_byte(0x14);	// adres rejestru do ustawiania lewych segmentów
				wyswietl_pierwszy_slot(liczba_dziesiatek, liczba_jednosci);
				wyswietl_drugi_slot(liczba_jednosci);
				I2C_check_transfer_complete();
				I2C_stop();
			}
		}
	}
}
