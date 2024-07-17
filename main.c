/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include "app_error.h"
#include "app_uart.h"
#include "bsp.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_common.h"
#include "nrf_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// #define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES (15U) /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE    256   /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE    256   /**< UART RX buffer size. */
#define QUEEUE_BUF_SIZE     1024
#define UART_BUF_SIZE       512
#define BUTTON1             17
#define LED1                21
#define SCL_PIN             15
#define SDA_PIN             16
#define BTN_IDLE            0
#define BTN_PUSHED          1
#define BTN_CHECK           2

typedef struct
{
    char que_buffer[QUEEUE_BUF_SIZE];
    int  head;
    int  tail;
} cir_queeue_t;

char          uart_buffer[UART_BUF_SIZE];
volatile int  check_string_index = 0;
volatile int  button_input_flag  = 0;
volatile bool command_ready      = false;
volatile bool is_tx_available    = false;
volatile bool check_string       = false;
volatile bool check_backspace    = false;
volatile bool que_start_flag     = false;
cir_queeue_t  que;

#if 0
    volatile uint32_t *RXD_REG     = (volatile uint32_t *) 0x40002518;
    volatile uint32_t *STARTRX_REG = (volatile uint32_t *) 0x40002000;
    volatile uint32_t *STOPRX_REG  = (volatile uint32_t *) 0x40002004;
    volatile uint32_t *RXDRDY_REG  = (volatile uint32_t *) 0x40002108;

    volatile uint32_t *TXD_REG     = (volatile uint32_t *) 0x4000251c;
    volatile uint32_t *STARTTX_REG = (volatile uint32_t *) 0x40002008;
    volatile uint32_t *STOPTX_REG  = (volatile uint32_t *) 0x4000200c;
    volatile uint32_t *TXDRDY_REG  = (volatile uint32_t *) 0x4000211c;

    volatile uint32_t *ENABLE_REG      = (volatile uint32_t *) 0x40002500;
    volatile uint32_t *PSELTXD_REG     = (volatile uint32_t *) 0x4000250C;
    volatile uint32_t *PSELRXD_REG     = (volatile uint32_t *) 0x40002514;
    volatile uint32_t *BAUDRATE_REG    = (volatile uint32_t *) 0x40002524;
    volatile uint32_t *UART_CONFIG_REG = (volatile uint32_t *) 0x4000256C;

#endif

void uart_error_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#ifdef ENABLE_LOOPBACK_TEST
/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}

/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t *tx_data = (uint8_t *) ("\r\nLOOPBACK_TEST\r\n");
    uint8_t  rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS)
            ;

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}

#endif
int que_count(int head, int tail)
{
    int count = ((tail - head) + QUEEUE_BUF_SIZE) % QUEEUE_BUF_SIZE;

    return count;
}

void enqueue(cir_queeue_t *que, char *p_str)
{
    int p_str_len = strlen(p_str);

    if (que_count(que->head, que->tail) + p_str_len > QUEEUE_BUF_SIZE)
    {
        return;
    }

    while ((*p_str) != '\0')
    {
        que->tail                  = (que->tail + 1) % QUEEUE_BUF_SIZE;
        que->que_buffer[que->tail] = *p_str;
        p_str                      = p_str + 1;
    }
}

char dequeue(cir_queeue_t *que)
{
    char target;
    que->head                  = (que->head + 1) % QUEEUE_BUF_SIZE;
    target                     = que->que_buffer[que->head];
    que->que_buffer[que->head] = 0;

    return target;
}

void print_string(char *p_str)
{
    char ch;

    while ((*p_str) != '\0')
    {
        while ((volatile int) 1)
        {

            if (is_tx_available)
            {
                break;
            }
        }

        ch    = *p_str;
        p_str = p_str + 1;

        NRF_UART0->TXD = ch;

        is_tx_available = false;
    }
}

void print_char(char target)
{
    while ((volatile int) 1)
    {

        if (is_tx_available)
        {
            break;
        }
    }
    NRF_UART0->TXD = target;

    is_tx_available = false;
}

void MY_UART0_IRQHandler(void)
{
    if (NRF_UART0->EVENTS_TXDRDY == 1)
    {
        NRF_UART0->EVENTS_TXDRDY = 0;
        is_tx_available          = true;
        if (que_start_flag == true)
        {
            print_char(dequeue(&que));
        }

        if (que_count(que.head, que.tail) == 0)
        {
            que_start_flag = false;
        }
    }

    if (NRF_UART0->EVENTS_RXDRDY == 1)
    {
        NRF_UART0->EVENTS_RXDRDY = 0;

        char ch = NRF_UART0->RXD;

        if (ch == '\b' && check_string_index > 0)
        {
            check_backspace = true;
            return;
        }

        if (check_string_index < UART_BUF_SIZE)
        {
            uart_buffer[check_string_index] = ch;
            NRF_UART0->TXD                  = ch;
        }
        else
        {
            memset(uart_buffer, 0, UART_BUF_SIZE);
            check_string_index = 0;
            ch                 = NULL;
        }

        if (ch == '\n')
        {
            check_string = true;
        }
        else if (ch == '\b' && check_string_index > 0)
        {
            check_backspace = true;
        }
        else
        {
            check_string_index = check_string_index + 1;
        }
    }
}

void LED_on_off()
{

    if (NRF_GPIO->OUT & (1UL << LED1))
    {
        NRF_GPIO->OUTCLR = (1UL << LED1);
        enqueue(&que, "LED light on\n");
    }
    else
    {
        NRF_GPIO->OUTSET = (1UL << LED1);
        enqueue(&que, "LED light off\n");
    }
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[0] == 1)
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        NRF_TIMER1->TASKS_CLEAR  = 1;
        NRF_TIMER1->TASKS_START  = 1;
        button_input_flag        = BTN_PUSHED;
    }
}
void GPIOTE_init(void)
{
    NRF_GPIO->PIN_CNF[BUTTON1] = 0xc;

    NRF_GPIOTE->CONFIG[0] = 0x21101;

    NRF_GPIOTE->INTENSET = 1;

    nrf_drv_common_irq_enable(GPIOTE_IRQn, 3);
}

void LED_init(void)
{
    NRF_GPIO->PIN_CNF[LED1] = 3;
    NRF_GPIO->OUTSET        = (1UL << LED1);
}

void UART_INTERRUPT_init(void)
{
    nrf_gpio_pin_set(9);
    nrf_gpio_cfg_output(9);
    nrf_gpio_cfg_input(11, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELRXD  = 11;
    NRF_UART0->PSELTXD  = 9;
    NRF_UART0->BAUDRATE = 0x01D7E000;
    NRF_UART0->CONFIG   = 0;

    NRF_UART0->INTENSET = (NRF_UART_INT_MASK_TXDRDY | NRF_UART_INT_MASK_RXDRDY);
    nrf_drv_common_irq_enable(UART0_IRQn, 3);

    NRF_UART0->TASKS_STARTRX = 1;
    NRF_UART0->TASKS_STARTTX = 1;

    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->ENABLE        = 4;

    is_tx_available = true;
}

void TIMER_init(void)
{
    NRF_TIMER1->MODE = 0;

    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = 4;
    NRF_TIMER1->CC[0]     = 10000;

    // NRF_TIMER1->SHORTS      = 1; // Enable only 'COMPARE0_CELAR'
    NRF_TIMER1->INTENSET    = 0x10000;
    NRF_TIMER1->TASKS_CLEAR = 1;
    nrf_drv_common_irq_enable(TIMER1_IRQn, 3);
}

void TIMER1_IRQHandler(void)
{

    if (NRF_TIMER1->EVENTS_COMPARE[0] == 1)
    {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
#if 0
        static int LED_STATE = 0;

        if (LED_STATE)
        {
            NRF_GPIO->OUTCLR = (1UL << LED1);
        }
        else
        {
            NRF_GPIO->OUTSET = (1UL << LED1);
        }

        LED_STATE = !LED_STATE;
				NRF_TIMER1->TASKS_STOP  = 1;
        NRF_TIMER1->TASKS_CLEAR = 1;
				NRF_TIMER1->TASKS_START=1;
#else
        if (button_input_flag == BTN_PUSHED)
        {
            button_input_flag       = BTN_CHECK;
            NRF_TIMER1->TASKS_STOP  = 1;
            NRF_TIMER1->TASKS_CLEAR = 1;
        }
#endif
    }
}
void I2C_init(void)
{
    NRF_GPIO->PIN_CNF[SCL_PIN] = 0x60;

    NRF_GPIO->PIN_CNF[SDA_PIN] =0x60;;

    NRF_TWI0->PSELSCL = SCL_PIN;
    NRF_TWI0->PSELSDA = SDA_PIN;

     NRF_TWI0->ENABLE    = 5;
    NRF_TWI0->FREQUENCY = 0x01980000; // 100 kbps

   NRF_TWI0->ADDRESS = 0x20;
}
void I2C_read(void)
{
    char   *msg = "I2C 값: ";
    uint8_t data;

    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->EVENTS_STOPPED  = 0;
    NRF_TWI0->TASKS_STARTRX   = 1;

    while (NRF_TWI0->EVENTS_RXDREADY == 0)
        ;

    data                      = NRF_TWI0->RXD;
    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->TASKS_STOP      = 1;

    while (NRF_TWI0->EVENTS_STOPPED == 0)
        ;

    NRF_TWI0->EVENTS_STOPPED = 0;

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "%s 0x%X\n", msg, data);
    enqueue(&que, buffer);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;

    char *command_string[4]  = {"LED1 on\n", "LED1 off\n", "LED2 on\n", "LED2 off\n"};
    char *reaction_string[4] = {"LED1 active\n", "LED1 inactive\n", "LED2 active\n", "LED2 inactive\n"};

    memset(&que, 0, sizeof(que));

    TIMER_init();
    UART_INTERRUPT_init();
    LED_init();
    GPIOTE_init();

    enqueue(&que, "System start! \r\n");
    I2C_init();
    I2C_read();

    while ((volatile int) 1)
    {

        if (button_input_flag == BTN_CHECK && (NRF_GPIO->IN & (1UL << BUTTON1)) == 0)
        {
            LED_on_off();
            button_input_flag = BTN_IDLE;
        }

        if (que_count(que.head, que.tail) != 0 && que_start_flag == false)
        {
            que_start_flag = true;
            print_char(dequeue(&que));
        }

        // #if 0
        if (check_string == true)
        {

            for (int i = 0; i < 4; i++)
            {
                if (strncmp((char *) uart_buffer, command_string[i], strlen(command_string[i])) == 0)
                {
                    enqueue(&que, reaction_string[i]);
                    break;
                    // LED must be on, immediately
                }
            }
            memset(uart_buffer, 0, UART_BUF_SIZE);
            check_string_index = 0;
            check_string       = false;
        }
        else if (check_backspace == true)
        {
            print_string("\b \b");
            uart_buffer[check_string_index] = NULL;
            check_string_index              = check_string_index - 1;
            check_backspace                 = false;
        }

        // #endif
    }
#if 0
    while (1)
    {
        while (*RXDRDY_REG == 0)
            ;
        *RXDRDY_REG = 0;

        ch = *RXD_REG;

        if (ch == check_string[index])
        {
            index++;
        }
        else
        {
            index = 0;
            if (ch == check_string[index])
            {
                index++;
            }
        }

        if (index == 3)
        {

            for (int i = 0; i < 3; i++)
            {
                *TXD_REG = 'X';

                while (*TXDRDY_REG == 0)
                    ;

                *TXDRDY_REG = 0;
            }
        }

        *TXD_REG = ch;

        while (*TXDRDY_REG == 0)
            ;

        *TXDRDY_REG = 0;
    }


    bsp_board_leds_init();

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };
			
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

#ifndef ENABLE_LOOPBACK_TEST
    //printf("\r\nStart: \r\n");
		const char check_string[3]="hi\n";
    
	  volatile uint32_t *RXD_REG = (volatile uint32_t *)0x40002518;
		volatile uint32_t *STARTRX_REG = (volatile uint32_t *)0x40002000;
		volatile uint32_t *STOPRX_REG = (volatile uint32_t *)0x40002004;
		volatile uint32_t *RXDRDY_REG = (volatile uint32_t *)0x40002108;

		volatile uint32_t *TXD_REG = (volatile uint32_t *)0x4000251c;
		volatile uint32_t *STARTTX_REG = (volatile uint32_t *)0x40002008;
		volatile uint32_t *STOPTX_REG = (volatile uint32_t *)0x4000200c;
		volatile uint32_t *TXDRDY_REG = (volatile uint32_t *)0x4000211c;
		
		volatile uint32_t *ENABLE_REG = (volatile uint32_t *)0x40002500;
	
		
		uint8_t cr;
			
			
			*RXDRDY_REG = 0;
			*TXDRDY_REG = 0;
			
			*ENABLE_REG = 4;
			
			*STARTRX_REG = 1;
			*STARTTX_REG = 1;
						
			while (1)
			{
				if (*TXDRDY_REG == 1)
				{
					*TXDRDY_REG = 0;
					*TXD_REG = 'a';
				}
			}
			
		while (true)
    {
			
			while(1)
			{
				if (*RXDRDY_REG == 1)
				{
					printf("2\n");
					*TXD_REG = *RXD_REG;
					*RXDRDY_REG = 0;
					
					while (1)
					{
						printf("3\n");
							if (*TXDRDY_REG == 0)
							{
								printf("4\n");
								*TXDRDY_REG = 0;
								break;
							}
					}
				}
			}
			
			*STOPRX_REG=1;
			*STOPTX_REG = 1;
        //while (app_uart_get(&cr) != NRF_SUCCESS);
        while (app_uart_put(cr) != NRF_SUCCESS);
				
					if(check_string[index]==cr){
						index++;
					}
					else{
						index=0;
						printf("\nfail\n");

					}
			
				if(index==strlen(check_string)){
					printf("good morning\n");
					index=0;
				}

        if (cr=='Q')
        {
            printf(" \r\nExit!\r\n");

            while (true)
            {
                // Do nothing.
            }
        }
    }
#else

    // This part of the example is just for testing the loopback .
    while (true)
    {
        uart_loopback_test();
    }
#endif
#endif
}

/** @} */
