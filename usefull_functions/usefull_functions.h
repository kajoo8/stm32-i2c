/*
 * przydatne_funkcje.h
 *
 *  Created on: 29.12.2021
 *      Author: Kajetan Jeznach
 */

#ifndef PRZYDATNE_FUNKCJE_H_
#define PRZYDATNE_FUNKCJE_H_

/******************************przerwanie_adc******************************/
void ADC1_COMP_IRQHandler(void) {
	if (0 != (ADC_ISR_EOC & ADC1->ISR)) {

		tempBuf[g_buf] = ADC1->DR;
		g_buf = (g_buf + 1) % BUF_SIZE;
		g_count++;

		if ((BUF_SIZE / x - 1) == g_count) {
			g_update = 1;
		}
	}
}

/******************************zegary******************************/
void cfg_clk(void) {
	RCC->CR |= RCC_CR_HSION; //enable hsi
	while (0 == (RCC->CR & RCC_CR_HSIRDY)) { //wait until RDY bit is set
		//empty
	}
	RCC->CFGR |= RCC_CFGR_SW_0; //set SYSCLK to HSI16
}

void I2C_clock_enable(void) {
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // wlaczenie zegara i2c
}

void TIM6_clock_enable(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
}

void ADC_clock_enable(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
}

void GPIOB_clock_enable(void) {
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
}

/******************************i2c******************************/
void I2C_pin_config(void) {
	GPIOB->MODER &= ~(GPIO_MODER_MODE8_0); // PB8 AF
	GPIOB->MODER &= ~(GPIO_MODER_MODE9_0); // PB9 AF
	GPIOB->AFR[1] |= 0x4; // AF4 for PB8 > I2C SCL
	GPIOB->AFR[1] |= (0x4 << 4); // AF4 for PB9 -> I2C SDA
	GPIOB->OTYPER |= (0x1 << 8); // open drain PB8
	GPIOB->OTYPER |= (0x1 << 9); // open drain PB9
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED8_1 | GPIO_OSPEEDER_OSPEED8_0);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED8_1; // set PB8 high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED9_1 | GPIO_OSPEEDER_OSPEED9_0);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED9_1; // set PB9 high speed
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD8_0);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_0; // pull-up for PA8
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD9_1 | GPIO_PUPDR_PUPD9_0);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD9_0; // pull-up for PA9;
}

void I2C_reset(void) {
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; // Reset I2C1, czyli 1
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST); // teraz 0
}

void I2C_peripherial_disable(void) {
	I2C1->CR1 = (0 << I2C_CR1_PE);
	while (0 != (I2C1->CR1 & I2C_CR1_PE))
		; // upewnienie sie ze peryferial jest wylaczony
}

void I2C_peripherial_enable(void) {
	I2C1->CR1 |= I2C_CR1_PE;	// wlaczenie peryferialu
}

void I2C_timingr_config(void) {
	I2C1->TIMINGR |= (0x19 << 0);
	I2C1->TIMINGR |= (0x19 << 8);
	I2C1->TIMINGR |= (0x39 << 16);
	I2C1->TIMINGR |= (0x39 << 20);
	I2C1->TIMINGR |= (0x9 << 28);
}

void I2C_set_slave_address(uint8_t slave_address) {
	I2C1->CR2 |= (slave_address << 1);	// przesuniecie o jeden, a tam bedzie zero czyli write
}

void I2C_7bit_transfer_config(uint8_t nbytes) {
	I2C1->CR2 &= ~(I2C_CR2_NBYTES);	// czyszczenie nbytes
	I2C1->CR2 |= (nbytes << 16);	// ustawienie liczby bajtów do wys³ania
	I2C1->CR2 &= ~(I2C_CR2_RELOAD);	// wylaczenie reloadu i autoendu, aby flaga TC dzialala po przeslaniu nbyteów
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND);
	I2C1->CR2 |= (0 << 10);			// tryb write
	I2C1->CR2 |= (0 << 11);			// tryb 7 bitowy
}

void I2C_start(void) {
	I2C1->CR2 |= (I2C_CR2_START);
}

void I2C_stop(void) {
	I2C1->CR2 |= (I2C_CR2_STOP);
	while (0 == (I2C1->ISR & I2C_ISR_STOPF))
		;
}

void I2C_send_byte(uint8_t byte) {
	while (0 == (I2C1->ISR & I2C_ISR_TXE))
		; // czekanie na ustawienie sie bitu txe
	I2C1->TXDR = byte;
}

void I2C_check_transfer_complete() {
	while (0 == (I2C1->ISR & I2C_ISR_TC))
		;
}

/******************************adc i timery******************************/
void ADC1_start() {
	ADC1->CR |= ADC_CR_ADSTART;
}

void TIM6_config() {
	TIM6->CR2 = TIM_CR2_MMS_1;
	TIM6->PSC = 125 - 1;
	TIM6->ARR = 128 - 1;	// 16MHz / 125 / 128 = 1000Hz
}

void TIM6_counter_enable() {
	TIM6->CR1 |= TIM_CR1_CEN;
}

void ADC1_config() {
	ADC1->IER = ADC_IER_EOCIE; // end of conversion interrupt enable
	ADC1->CR = ADC_CR_ADVREGEN; // enable voltage regulator
	ADC1->CFGR1 = ADC_CFGR1_EXTEN_0; // hardware trigger, rising edge, tim6, 12 bitów, wyr. do prawej
	ADC1->CFGR2 = ADC_CFGR2_CKMODE_1 | ADC_CFGR2_CKMODE_0; // zegar PCLK podzielony przez 1
	ADC1->SMPR = ADC_SMPR_SMPR_0 | ADC_SMPR_SMPR_1 | ADC_SMPR_SMPR_2; // najwyzszy czas probkowania 160,5 cyklu zegarowego
	ADC1->CHSELR = ADC_CHANNEL_18; // na tym kanale jest czujnik temp
	ADC->CCR = ADC_CCR_TSEN; // wlacz czujnik
	ADC1->CR |= ADC_CR_ADCAL; // kalibracja
	while (0 != (ADC_CR_ADCAL & ADC1->CR)) {
		//empty
	}
	ADC1->ISR = ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while (0 == (ADC_ISR_ADRDY & ADC1->ISR)) {
		//empty
	}
}

/******************************wyswietlanie liczb******************************/
void wyswietl_drugi_slot(uint8_t liczba_jednosci) {
	switch (liczba_jednosci) {
	case 0:
		I2C_send_byte(0x3D);
		break;
	case 1:
		I2C_send_byte(0x30);
		break;
	case 2:
		I2C_send_byte(0x1E);
		break;
	case 3:
		I2C_send_byte(0x3E);
		break;
	case 4:
		I2C_send_byte(0x33);
		break;
	case 5:
		I2C_send_byte(0x2F);
		break;
	case 6:
		I2C_send_byte(0x2F);
		break;
	case 7:
		I2C_send_byte(0x34);
		break;
	case 8:
		I2C_send_byte(0x3F);
		break;
	case 9:
		I2C_send_byte(0x3F);
		break;
	default:
		I2C_send_byte(0xF); // przy bledzie wyswietla siê EE
		break;
	}
}

void wyswietl_pierwszy_slot(uint8_t liczba_dziesiatek, uint8_t liczba_jednosci) {
	if (0 == liczba_jednosci || 2 == liczba_jednosci || 6 == liczba_jednosci || 8 == liczba_jednosci) {
		switch (liczba_dziesiatek) {
		case 0:
			I2C_send_byte(0xFB); // wszystkie maja jedynke na poczatku (zapis binarny) z racji tego ze potrzebna jest ona do poprawnego wyswietlenia liczby jednosci
			break;
		case 1:
			I2C_send_byte(0xE0);
			break;
		case 2:
			I2C_send_byte(0xBD);
			break;
		case 3:
			I2C_send_byte(0xFC);
			break;
		case 4:
			I2C_send_byte(0x73);
			break;
		case 5:
			I2C_send_byte(0xDE);
			break;
		case 6:
			I2C_send_byte(0xDF);
			break;
		case 7:
			I2C_send_byte(0xE8);
			break;
		case 8:
			I2C_send_byte(0xFF);
			break;
		case 9:
			I2C_send_byte(0xFE);
			break;
		default:
			I2C_send_byte(0x4F); // przy bledzie wyswietla siê EE
			break;
		}
	} else {
		switch (liczba_dziesiatek) {
				case 0:
					I2C_send_byte(0x7B); // wszystkie maja zero na poczatku, tym razem
					break;
				case 1:
					I2C_send_byte(0x60);
					break;
				case 2:
					I2C_send_byte(0x3D);
					break;
				case 3:
					I2C_send_byte(0x7C);
					break;
				case 4:
					I2C_send_byte(0x33);
					break;
				case 5:
					I2C_send_byte(0x5E);
					break;
				case 6:
					I2C_send_byte(0x5F);
					break;
				case 7:
					I2C_send_byte(0x68);
					break;
				case 8:
					I2C_send_byte(0x7F);
					break;
				case 9:
					I2C_send_byte(0x7E);
					break;
				default:
					I2C_send_byte(0x4F); // przy bledzie wyswietla siê EE
					break;
				}
	}
}

#endif /* PRZYDATNE_FUNKCJE_H_ */
