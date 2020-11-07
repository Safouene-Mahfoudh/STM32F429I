##The serial transmission is of the asynchronous type. The DISCOVERY board has several RS232 outputs: synchronous or asynchronous type.
* USART1: Pins PA9 and PA10
* USART2: Pins PA2: TX and PA3: RX
* USART3: Pins PB10 and PB11
* USART6: Pins PC6 and PC7
* UART4: Pins PA0 and PA1
* UART5: PC12 and PD2 pins

##Don't forget to configure USARTx with STM32CubeMX:
**Pinout & Configuration -> Connectivity -> USARTx -> Mode: asynchronous** then at the bottom set the baud rate, 9600,8bits,no parity and 1 stop bit. 

##In the main program, the following functions needed to be added in order to be able to use easily the **printf** display function to send data easily to the terminal.

**#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huartx, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}***

###It will be necessary to add all the instructions already quoted before int main (void) 