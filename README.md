# Bluetooth_DC_Motors_STM32
Control of 3 DC motors via bluetooth and display of 3 voltages on smartphone.

This code configures various peripherals of a microcontroller (STM32F4xx) such as ADC (Analog to Digital Converter),
USART (Universal Synchronous Asynchronous Receiver Transmitter), and GPIO (General Purpose Input/Output) on the microcontroller.

It sets up the system clock source to be the HSE (High Speed External) oscillator and initializes the ADC to scan 3 channels, CH2, CH0, and CH1.
The ADC conversion result interrupt is enabled, and the NVIC (Nested Vectored Interrupt Controller) is enabled for the ADC interrupt.

The USART3 peripheral is also configured for communication and the GPIOB peripheral is set up to alternate function for USART3 transmission and reception.
The SendChar and SendString functions are used to transmit characters and strings to the USART3 peripheral.


