### test_project

STM32/FreeRTOS SPI <-> UART bridge test project

Pinout:

    SPI1_SCK  PA1   A1
    SPI1_MISO PA6   A5
    SPI1_MOSI PA7   A6

    USART1_TX PB6   D5
    USART1_RX PB7   D6

The above pinout is for the STM Nucleo-32 L432KC board (the one I found on my shelves)

It is supposed :
  - we have external GPIO from SPI device signaling about some SPI data ready for read
  - we send data in both directions in 16 bytes packets
  - did not bother about correct DMA channels & external GPIO intialization
  - did not make sure eveything is compiled smoothly and no debug on real hardware done