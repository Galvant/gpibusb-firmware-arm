/*
* GPIBUSB Adapter
* firmware.C
**
* © 2014 Steven Casagrande (scasagrande@galvant.ca).
*
* This file is a part of the GPIBUSB Adapter project.
* Licensed under the AGPL version 3.
**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/** INCLUDES ******************************************************************/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <stdio.h>
#include <errno.h>

#include "firmware.h"
#include "ring.h"
#include "gpib.h"

/** DEFINES *******************************************************************/

#define UART_BUFFER_SIZE 1024

/** GLOBALS *******************************************************************/

struct ring output_ring;
struct ring input_ring;

uint8_t output_ring_buffer[UART_BUFFER_SIZE];
uint8_t input_ring_buffer[UART_BUFFER_SIZE];

bool mode = 1;

/** PROTOTYPES ****************************************************************/

int _write(int file, char *ptr, int len);
static void clock_setup(void);
static void usart_setup(void);
static void gpio_setup(void);
int main(void);

/** FUNCTIONS *****************************************************************/

static void clock_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_USART2);
}

static void usart_setup(void)
{
    ring_init(&output_ring, output_ring_buffer, UART_BUFFER_SIZE);
    ring_init(&input_ring, input_ring_buffer, UART_BUFFER_SIZE);

    /* A2 = TX, A3 = RX */
	nvic_enable_irq(NVIC_USART2_IRQ);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO3);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(USART2);
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	/* Nucleo LED = A5 */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	
	// LED Setup
	//gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR);
	
	// Flow port pins will always be outputs
	gpio_mode_setup(FLOW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TE);
	gpio_mode_setup(FLOW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PE);
	gpio_mode_setup(FLOW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SC);
	gpio_mode_setup(FLOW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DC);
	gpio_clear(FLOW_PORT, TE);
	gpio_clear(FLOW_PORT, PE);
	if (mode) {
	    gpio_set(FLOW_PORT, SC); // TX on REN and IFC
	    gpio_clear(FLOW_PORT, DC); // TX on ATN and RX on SRQ
	} else {
	    gpio_clear(FLOW_PORT, SC);
	    gpio_set(FLOW_PORT, DC);
	}
	
	// Float all DIO lines
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO7);
	gpio_mode_setup(DIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
	
	// Set mode and pin state for all GPIB control lines
	if (mode) {
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ATN);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, EOI);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DAV);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRFD);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NDAC);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, IFC);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SRQ);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, REN);
	    gpio_set(CONTROL_PORT, ATN);
	    gpio_clear(CONTROL_PORT, NRFD);
	    gpio_clear(CONTROL_PORT, NDAC);
	    gpio_set(CONTROL_PORT, IFC);
	    gpio_clear(CONTROL_PORT, REN);
	} else {
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ATN);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, EOI);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DAV);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, NRFD);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, NDAC);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, IFC);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SRQ);
	    gpio_mode_setup(CONTROL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, REN);
	} 
	
}

int _write(int file, char *ptr, int len)
{
    int ret;

    if (file == 1) {
        ret = ring_write(&output_ring, (uint8_t *)ptr, len);

        if (ret < 0)
            ret = -ret;

        usart_enable_tx_interrupt(USART2);

        return ret;
    }

    errno = EIO;
    return -1;
}

void usart2_isr(void)
{
	// Check if we were called because of RXNE. 
	if (usart_get_interrupt_source(USART2, USART_ISR_RXNE)) {
		ring_write_ch(&input_ring, usart_recv(USART2));
		usart_enable_tx_interrupt(USART2);
	}

	// Check if we were called because of TXE. 
	if (usart_get_interrupt_source(USART2, USART_ISR_TXE)) {
	    int32_t data;
		data = ring_read_ch(&output_ring, 0);
		if (data == -1) {
            usart_disable_tx_interrupt(USART2);
            
        } else {
            usart_send_blocking(USART2, data);
        }
	}
}

int main(void)
{
    int i;
	clock_setup();
	gpio_setup();
	usart_setup();
	
	// Turn on the error LED
	gpio_set(LED_PORT, LED_ERROR);
	
	// TODO: start WDT
	// TODO: start 1ms timer
	// TODO: Load settings from EEPROM
	
	// Initialize the GPIB bus
	if (mode) {
	    gpib_controller_assign(0x00);
	}
	
	// TODO: enable timer interrupts
	gpio_clear(LED_PORT, LED_ERROR);

	while (1) {
		for (i = 0; i < 1000000; i++) {
			__asm__("NOP");
		}
		gpio_toggle(GPIOA, GPIO5);
	}

	return 0;
}

