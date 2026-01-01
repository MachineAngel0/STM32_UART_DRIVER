#ifndef USART2_H
#define USART2_H


void USART2_Baremetal_Init(void)
{
    /*
    TX Procedure:
    1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
    2. Program the M bit in USART_CR1 to define the word length.
    3. Program the number of stop bits in USART_CR2.
    4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
    place. Configure the DMA register as explained in multibuffer communication.
    5. Select the desired baud rate using the USART_BRR register.
    6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
    7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
    for each data to be transmitted in case of single buffer.
    8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
    that the transmission of the last frame is complete. This is required for instance when
    the USART is disabled or enters the Halt mode to avoid corrupting the last
    transmission

    Clearing the TXE bit is always performed by a write to the data register.
    The TXE bit is set by hardware and it indicates:
    • The data has been moved from TDR to the shift register and the data transmission has
    started.
    • The TDR register is empty.
    • The next data can be written in the USART_DR register without overwriting the
    previous data.
    This flag generates an interrupt if the TXEIE bit is set.



    RX Procedure:
    1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
    2. Program the M bit in USART_CR1 to define the word length.
    3. Program the number of stop bits in USART_CR2.
    4. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take
    place. Configure the DMA register as explained in multibuffer communication. STEP 3
    5. Select the desired baud rate using the baud rate register USART_BRR
    6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a
    start bit.
    When a character is received
    • The RXNE bit is set. It indicates that the content of the shift register is transferred to the
    RDR. In other words, data has been received and can be read (as well as its
    associated error flags).
    • An interrupt is generated if the RXNEIE bit is set.
    • The error flags can be set if a frame error, noise or an overrun error has been detected
    during reception.
    • In multibuffer, RXNE is set after every byte received and is cleared by the DMA read to
    the Data Register.
    • In single buffer mode, clearing the RXNE bit is performed by a software read to the
    USART_DR register. The RXNE flag can also be cleared by writing a zero to it. The
    RXNE bit must be cleared before the end of the reception of the next character to avoid
    an overrun error.
 */


    //enable the clock line that this USART is on
    //enable the port, then the usart
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


    /* TX PORT CONFIG*/

    //set the GPIO, pin2 to the alt function mode
    //port mode register
    // 00: Input (reset state)
    // 01: General purpose output mode
    // 10: Alternate function mode (this is the one we want)
    // 11: Analog mode
    //clear register then set the AF MODE
    GPIOA->MODER &= ~GPIO_MODER_MODER2;
    GPIOA->MODER |= GPIO_MODER_MODER2_1;

    //clear the register to set it to push pull
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;

    //00: Low speed
    // 01: Medium speed
    // 10: High speed
    // 11: Very high speed
    //clear the register to set it to low speed
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR2;

    //pullup/down
    // 00: No pull-up, pull-down
    // 01: Pull-up
    // 10: Pull-down
    // 11: Reserved
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;

    //turn on the input register
    // GPIOA->IDR |= GPIO_IDR_ID2;
    //turn on the output register
    // GPIOA->ODR |= GPIO_ODR_OD2;


    //USART is considered an alternate function for the gpio, so we have to set that
    //check the alternate mapping in the data sheet
    // USART2 TX PA2, AF07,
    // USART2 RX PA3, AF07,

    // https://www.learningaboutelectronics.com/Articles/Alternate-function-low-register-high-register-STM32F4xx.php
    //low register is for the gpio pins from 0-7
    //high register is for the gpio pins from 8-15 (but it won't be written like that in the hardware abstraction!!!!!)

    //0 is the low register, 1 is the high register
    // we are setting PA2 and PA3
    // AF7 = 0111

    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0; //1
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1; //1
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2; //1
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_3; //0


    /* RX PORT CONFIG*/
    //basically a copy paste of TX for GPIO port 3
    GPIOA->MODER &= ~GPIO_MODER_MODER3;
    GPIOA->MODER |= GPIO_MODER_MODER3_1;

    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;

    GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR3;

    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;

    // USART2 RX PA3, AF07,
    // AF7 = 0111
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0; //1
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1; //1
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2; //1
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_3; //0



    //word length (data length really)
    //0 -> 1 start bit and 8 data bits, n stop bits
    //1 -> 1 start bit and 9 data bits, n stop bits
    // USART2->CR1 |= USART_CR1_M;
    // USART2->CR1 &= USART_CR1_M;
    USART2->CR1 &= ~USART_CR1_M;

    //set the number of stop bits
    // These bits are used for programming the stop bits.
    // 00: 1 Stop bit
    // 01: 0.5 Stop bit
    // 10: 2 Stop bits
    USART2->CR2 &= ~USART_CR2_STOP; // set bits to 00

    //baud rate
    //FORMULA = clock line hz / baud rate
    uint32_t baud_rate = 115200;
    uint32_t clock_line_hz = (84000000 / 2); // 42mHZ
    USART2->BRR = clock_line_hz / baud_rate;

    //transmitter enabled
    USART2->CR1 |= USART_CR1_TE;
    //receiver enabled
    USART2->CR1 |= USART_CR1_RE;

    //enable usart
    USART2->CR1 |= USART_CR1_UE;
}

void USART2_Baremetal_Write(uint8_t* usart_buffer, uint8_t* usart_buffer_length)
{
    //after done sending data wait until TC=1
    //we wait in general and make sure there is nothing in the register
    // NOTE: i can exclude this but it still works, not going to because of the data sheet
    while (!(USART2->SR & USART_SR_TC))
    {
    }


    //write into the data register, clears the TXE bit, TXE is set by hardware
    for (uint8_t i = 0; i < *usart_buffer_length; i++)
    {
        // wait for data to be transferred into the shift register
        // 0 damage is not transferred, 1 data has been transferred
        while (!(USART2->SR & USART_SR_TXE))
        {
        }

        USART2->DR = usart_buffer[i];
    }
}

uint8_t USART2_Baremetal_Read(void)
{
    //Wait for a data to be received on the USART_SR_RXNE bit
    while (!(USART2->SR & USART_SR_RXNE))
    {
    }

    //read the dr register to get the data, this also causes a clear on the dr register
    return USART2->DR;
}

#endif //USART2_H
