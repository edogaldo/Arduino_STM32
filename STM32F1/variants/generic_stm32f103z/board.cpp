/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file   wirish/boards/maple/board.cpp
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Maple board file.
 */

#include <board/board.h>         // For this board's header file

/* Roger Clark. Added next to includes for changes to Serial */
#include <libmaple/usart.h>
#include <HardwareSerial.h>

#include <wirish_types.h> // For stm32_pin_info and its contents
                                 // (these go into PIN_MAP).

#include "boards_private.h"      // For PMAP_ROW(), which makes
                                 // PIN_MAP easier to read.

// boardInit(): nothing special to do for Maple.
//
// When defining your own board.cpp, you can put extra code in this
// function if you have anything you want done on reset, before main()
// or setup() are called.
//
// If there's nothing special you need done, feel free to leave this
// function out, as we do here.
/*
void boardInit(void) {
}
*/

// Pin map: this lets the basic I/O functions (digitalWrite(),
// analogRead(), pwmWrite()) translate from pin numbers to STM32
// peripherals.
//
// PMAP_ROW() lets us specify a row (really a struct stm32_pin_info)
// in the pin map. Its arguments are:
//
// - GPIO device for the pin (&gpioa, etc.)
// - GPIO bit for the pin (0 through 15)
// - Timer device, or NULL if none
// - Timer channel (1 to 4, for PWM), or 0 if none
// - ADC device, or NULL if none
// - ADC channel, or ADCx if none

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {
/*
    gpio_dev *gpio_device;      GPIO device 
    timer_dev *timer_device;    Pin's timer device, if any.
    const adc_dev *adc_device;  ADC device, if any. 
    uint8 gpio_bit;             Pin's GPIO port bit. 
    uint8 timer_channel;        Timer channel, or 0 if none. 
    uint8 adc_channel;          Pin ADC channel, or ADCx if none. 
*/

    {&gpioa, &timer2, &adc1,  0, 1,    0}, /* PA0   ADC123_IN0|TIM2_CH1 */
    {&gpioa, &timer2, &adc1,  1, 2,    1}, /* PA1   ADC123_IN1|TIM2_CH2 */
	{&gpioa, &timer2, &adc1,  2, 3,    2}, /* PA2   ADC123_IN2|TIM2_CH3 */
    {&gpioa, &timer2, &adc1,  3, 4,    3}, /* PA3   ADC123_IN3|TIM2_CH4 */
	{&gpioa,    NULL, &adc1,  4, 0,    4}, /* PA4   ADC12_IN4|DAC_OUT1 */
    {&gpioa,    NULL, &adc1,  5, 0,    5}, /* PA5   ADC12_IN5|DAC_OUT2 */
    {&gpioa, &timer3, &adc1,  6, 1,    6}, /* PA6   ADC12_IN6|TIM3_CH1 */
    {&gpioa, &timer3, &adc1,  7, 2,    7}, /* PA7   ADC12_IN7|TIM3_CH2 */
    {&gpioa, &timer1,  NULL,  8, 1, ADCx}, /* PA8   TIM1_CH1 */
    {&gpioa, &timer1,  NULL,  9, 2, ADCx}, /* PA9   TIM1_CH2 */
    {&gpioa, &timer1,  NULL, 10, 3, ADCx}, /* PA10  TIM1_CH3 */
    {&gpioa, &timer1,  NULL, 11, 4, ADCx}, /* PA11  USB_DM|CAN_RX|TIM1_CH4 */ 
    {&gpioa,    NULL,  NULL, 12, 0, ADCx}, /* PA12  USB_DP|CAN_TX|TIM1_ETR */
    {&gpioa,    NULL,  NULL, 13, 0, ADCx}, /* PA13 */
    {&gpioa,    NULL,  NULL, 14, 0, ADCx}, /* PA14 */
    {&gpioa,    NULL,  NULL, 15, 0, ADCx}, /* PA15  SPI3_NSS */
	
	{&gpiob, &timer3, &adc1,  0, 3,    8}, /* PB0 */ 
	{&gpiob, &timer3, &adc1,  1, 4,    9}, /* PB1 */
	{&gpiob,    NULL,  NULL,  2, 0, ADCx}, /* PB2  */	// Boot1 pin. It affects boot mode (flash, RAM, ROM)
	{&gpiob,    NULL,  NULL,  3, 0, ADCx}, /* PB3  */ //JTDO, SPI3_SCK / I2S3_CK/
    {&gpiob,    NULL,  NULL,  4, 0, ADCx}, /* PB4  */ //NJTRST, SPI3_MISO
    {&gpiob,    NULL,  NULL,  5, 0, ADCx}, /* PB5 */ //I2C1_SMBA/ SPI3_MOSI
    {&gpiob, &timer4,  NULL,  6, 1, ADCx}, /* PB6 */ //I2C1_SCL(9)
    {&gpiob, &timer4,  NULL,  7, 2, ADCx}, /* PB7 */ //I2C1_SDA(9) / FSMC_NADV
    {&gpiob, &timer4,  NULL,  8, 3, ADCx}, /* PB8 */ //SDIO_D4
	{&gpiob, &timer4,  NULL,  9, 4, ADCx}, /* PB9 */ //SDIO_D5
    {&gpiob,    NULL,  NULL, 10, 0, ADCx}, /* PB10 */ //I2C2_SCL/USART3_TX
    {&gpiob,    NULL,  NULL, 11, 0, ADCx}, /* PB11 */ //I2C2_SDA/USART3_RX
    {&gpiob,    NULL,  NULL, 12, 0, ADCx}, /* PB12 */ //SPI2_NSS/I2S2_WS/I2C2_SMBA/USART3_CK
    {&gpiob,    NULL,  NULL, 13, 0, ADCx}, /* PB13 */ //SPI2_SCK/I2S2_CK/USART3_CTS
    {&gpiob,    NULL,  NULL, 14, 0, ADCx}, /* PB14 */ //SPI2_MISO/TIM1_CH2N/USART3_RTS
    {&gpiob,    NULL,  NULL, 15, 0, ADCx}, /* PB15 */ //SPI2_MOSI/I2S2_SD

    {&gpioc,    NULL, &adc1,  0, 0,   10}, /* PC0   ADC123_IN10 */
    {&gpioc,    NULL, &adc1,  1, 0,   11}, /* PC1   ADC123_IN11 */
    {&gpioc,    NULL, &adc1,  2, 0,   12}, /* PC2   ADC123_IN12 */
    {&gpioc,    NULL, &adc1,  3, 0,   13}, /* PC3   ADC123_IN13 */
    {&gpioc,    NULL, &adc1,  4, 0,   14}, /* PC4   ADC12_IN14  */
    {&gpioc,    NULL, &adc1,  5, 0,   15}, /* PC5   ADC12_IN15  */
    {&gpioc, &timer8,  NULL,  6, 1, ADCx}, /* PC6   I2S2_MCK/SDIO_D6*/	
	{&gpioc, &timer8,  NULL,  7, 2, ADCx}, /* PC7   I2S3_MCK/SDIO_D7*/
    {&gpioc, &timer8,  NULL,  8, 3, ADCx}, /* PC8   SDIO_D0*/
    {&gpioc, &timer8,  NULL,  9, 4, ADCx}, /* PC9   SDIO_D1*/
    {&gpioc,    NULL,  NULL, 10, 0, ADCx}, /* PC10  UART4_TX/SDIO_D2 */
    {&gpioc,    NULL,  NULL, 11, 0, ADCx}, /* PC11  UART4_RX/SDIO_D3 */
    {&gpioc,    NULL,  NULL, 12, 0, ADCx}, /* PC12  UART5_TX/SDIO_CK */	
    {&gpioc,    NULL,  NULL, 13, 0, ADCx}, /* PC13  TAMPER-RTC/ Limited output*/
    {&gpioc,    NULL,  NULL, 14, 0, ADCx}, /* PC14  OSC32_IN/ Limited output*/
    {&gpioc,    NULL,  NULL, 15, 0, ADCx}, /* PC15  OSC32_OUT/ Limited output*/

	{&gpiod,    NULL, NULL,   0, 0, ADCx}, /* PD0   OSC_IN */
	{&gpiod,    NULL, NULL,   1, 0, ADCx}, /* PD1   OSC_OUT */
	{&gpiod,    NULL, NULL,   2, 0, ADCx}, /* PD2   TIM3_ETR/UART5_RX SDIO_CMD */
	{&gpiod,    NULL, NULL,   3, 0, ADCx}, /* PD3   FSMC_CLK */
	{&gpiod,    NULL, NULL,   4, 0, ADCx}, /* PD4   FSMC_NOE */
	{&gpiod,    NULL, NULL,   5, 0, ADCx}, /* PD5   FSMC_NWE */
	{&gpiod,    NULL, NULL,   6, 0, ADCx}, /* PD6   FSMC_NWAIT */
	{&gpiod,    NULL, NULL,   7, 0, ADCx}, /* PD7   FSMC_NE1/FSMC_NCE2 */
	{&gpiod,    NULL, NULL,   8, 0, ADCx}, /* PD8   FSMC_D13 */
	{&gpiod,    NULL, NULL,   9, 0, ADCx}, /* PD9   FSMC_D14 */
	{&gpiod,    NULL, NULL,  10, 0, ADCx}, /* PD10  FSMC_D15 */
	{&gpiod,    NULL, NULL,  11, 0, ADCx}, /* PD11  FSMC_A16 */
	{&gpiod,    NULL, NULL,  12, 0, ADCx}, /* PD12  FSMC_A17 */
	{&gpiod,    NULL, NULL,  13, 0, ADCx}, /* PD13  FSMC_A18 */	
	{&gpiod,    NULL, NULL,  14, 0, ADCx}, /* PD14  FSMC_D0 */
	{&gpiod,    NULL, NULL,  15, 0, ADCx}, /* PD15  FSMC_D1 */
	
	{&gpioe,    NULL, NULL,   0, 0, ADCx}, /* PE0   TIM4_ETR / FSMC_NBL0 */
	{&gpioe,    NULL, NULL,   1, 0, ADCx}, /* PE1   FSMC_NBL1 */
	{&gpioe,    NULL, NULL,   2, 0, ADCx}, /* PE2   TRACECK/ FSMC_A23 */
	{&gpioe,    NULL, NULL,   3, 0, ADCx}, /* PE3   TRACED0/FSMC_A19 */
	{&gpioe,    NULL, NULL,   4, 0, ADCx}, /* PE4   TRACED1/FSMC_A20 */
	{&gpioe,    NULL, NULL,   5, 0, ADCx}, /* PE5   TRACED2/FSMC_A21 */
	{&gpioe,    NULL, NULL,   6, 0, ADCx}, /* PE6   TRACED3/FSMC_A22 */
	{&gpioe,    NULL, NULL,   7, 0, ADCx}, /* PE7   FSMC_D4 */
	{&gpioe,    NULL, NULL,   8, 0, ADCx}, /* PE8   FSMC_D5 */
	{&gpioe,    NULL, NULL,   9, 0, ADCx}, /* PE9   FSMC_D6 */
	{&gpioe,    NULL, NULL,  10, 0, ADCx}, /* PE10  FSMC_D7 */
	{&gpioe,    NULL, NULL,  11, 0, ADCx}, /* PE11  FSMC_D8 */
	{&gpioe,    NULL, NULL,  12, 0, ADCx}, /* PE12  FSMC_D9 */
	{&gpioe,    NULL, NULL,  13, 0, ADCx}, /* PE13  FSMC_D10 */	
	{&gpioe,    NULL, NULL,  14, 0, ADCx}, /* PE14  FSMC_D11 */
	{&gpioe,    NULL, NULL,  15, 0, ADCx}, /* PE15  FSMC_D12 */	

	{&gpiof,    NULL, NULL,   0, 0, ADCx}, /* PF0   */
	{&gpiof,    NULL, NULL,   1, 0, ADCx}, /* PF1   */
	{&gpiof,    NULL, NULL,   2, 0, ADCx}, /* PF2   */
	{&gpiof,    NULL, NULL,   3, 0, ADCx}, /* PF3   */
	{&gpiof,    NULL, NULL,   4, 0, ADCx}, /* PF4   */
	{&gpiof,    NULL, NULL,   5, 0, ADCx}, /* PF5   */
	{&gpiof,    NULL, &adc3,  6, 0,    4}, /* PF6   */
	{&gpiof,    NULL, &adc3,  7, 0,    5}, /* PF7   */
	{&gpiof,    NULL, &adc3,  8, 0,    6}, /* PF8   */
	{&gpiof,    NULL, &adc3,  9, 0,    7}, /* PF9   */
	{&gpiof,    NULL, &adc3, 10, 0,    8}, /* PF10  */
	{&gpiof,    NULL, NULL,  11, 0, ADCx}, /* PF11  */
	{&gpiof,    NULL, NULL,  12, 0, ADCx}, /* PF12  */
	{&gpiof,    NULL, NULL,  13, 0, ADCx}, /* PF13  */	
	{&gpiof,    NULL, NULL,  14, 0, ADCx}, /* PF14  */
	{&gpiof,    NULL, NULL,  15, 0, ADCx}, /* PF15  */		
	
	{&gpiog,    NULL, NULL,   0, 0, ADCx}, /* PG0   */
	{&gpiog,    NULL, NULL,   1, 0, ADCx}, /* PG1   */
	{&gpiog,    NULL, NULL,   2, 0, ADCx}, /* PG2   */
	{&gpiog,    NULL, NULL,   3, 0, ADCx}, /* PG3   */
	{&gpiog,    NULL, NULL,   4, 0, ADCx}, /* PG4   */
	{&gpiog,    NULL, NULL,   5, 0, ADCx}, /* PG5   */
	{&gpiog,    NULL, NULL,   6, 0, ADCx}, /* PG6   */
	{&gpiog,    NULL, NULL,   7, 0, ADCx}, /* PG7   */
	{&gpiog,    NULL, NULL,   8, 0, ADCx}, /* PG8   */
	{&gpiog,    NULL, NULL,   9, 0, ADCx}, /* PG9   */
	{&gpiog,    NULL, NULL,  10, 0, ADCx}, /* PG10  */
	{&gpiog,    NULL, NULL,  11, 0, ADCx}, /* PG11  */
	{&gpiog,    NULL, NULL,  12, 0, ADCx}, /* PG12  */
	{&gpiog,    NULL, NULL,  13, 0, ADCx}, /* PG13  */	
	{&gpiog,    NULL, NULL,  14, 0, ADCx}, /* PG14  */
	{&gpiog,    NULL, NULL,  15, 0, ADCx}  /* PG15  */		
};

/*  Basically everything that is defined as having a timer is PWM */
extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    PA0,PA1,PA2,PA3,PA6,PA7,PA8,PA9,PA10,PB0,PB1,PB6,PB7,PB8,PB9,PC6,PC7,PC8,PC9
};

/*  Basically everything that is defined having ADC */
extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5,PF6,PF7,PF8,PF9,PF10
};

/* not sure what this is used for */
extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
    BOARD_JTMS_SWDIO_PIN,
    BOARD_JTCK_SWCLK_PIN, BOARD_JTDI_PIN, BOARD_JTDO_PIN, BOARD_NJTRST_PIN
};


DEFINE_HWSERIAL(Serial1, 1);
DEFINE_HWSERIAL(Serial2, 2);
DEFINE_HWSERIAL(Serial3, 3);
DEFINE_HWSERIAL_UART(Serial4, 4);
DEFINE_HWSERIAL_UART(Serial5, 5);

#ifdef SERIAL_USB
#include <usb_serial.h>
USBSerial Serial;
#else
HardwareSerial& Serial=Serial1;
#endif
