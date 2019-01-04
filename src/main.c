#include "main.h"

#include <stdint.h>
#include "stm32f4xx.h"

#define DESELECT()      GPIOC->ODR |= (1<<2)
#define SELECT()        GPIOC->ODR &= ~(1<<2)

void spi5_w8(uint8_t);   // write 8 bit data to SPI5
void spi5_w16(uint16_t); // write 16 bit data to SPI5
void spi5_cmd(uint8_t);  // write a command
void __attribute__((optimize("O0"))) delay_cicles(uint32_t);
void ili9341_init(void);        // init the Ilitek 9341 spi display

int main(void) {

    RCC->AHB1ENR |= (1<<3);     // Power on GPIOD
    RCC->AHB1ENR |= (1<<5);     // Power on GPIOF
    RCC->AHB1ENR |= (1<<6);     // Power on GPIOG

    // Enable PIN 13 PORT G (led)
    GPIOG->MODER |= (1<<26);
    GPIOG->MODER &= ~(1<<27);
    // Enable PIN 14 PORT G (led)
    GPIOG->MODER |= (1<<28);


    // SPI1 configuration:
    // PF7: CLOCK
    // PF8: MISO
    // PF9: MOSI

    // Alternate function mode for PF7, PF8, PF9
    GPIOF->MODER |= (0x02<<14); // PF7
    GPIOF->MODER |= (0x02<<16); // PF8
    GPIOF->MODER |= (0x02<<18); // PF9

    // Alternate functions for PF7, PF8, PF9
    // SPI5 is AF5 (i.e. 0101)
    GPIOF->AFR[0] |= (0x05<<28);    // PF7
    GPIOF->AFR[1] |= (0x05<<0);     // PF8
    GPIOF->AFR[1] |= (0x05<<4);     // PF9


    // GPIO PINS for chip driving
    // PC2: CSX (chip select)   0=selected, 1=not selected
    // PD13: WRX_DCX     0=command, 1=data
    RCC->AHB1ENR |= (1<<2);     // Power on GPIOC
    RCC->AHB1ENR |= (1<<3);     // Power on GPIOD
    GPIOC->MODER |= (1<<4);     // Enable PC2
    GPIOD->MODER |= (1<<26);    // Enable PD13

    // Initialize SPI peripheral
    RCC->APB2ENR |= (1<<20);    // Power on SPI5   
    SPI5->CR1 &= ~(1<<6);       // Make sure SPI5 is not enabled
    RCC->APB2RSTR |= (1<<20);   // Reset peripheral register for SPI5 on
    RCC->APB2RSTR &= ~(1<<20);  // Reset peripheral register for SPI5 off
    SPI5->CR1 |= (1<<0) | (1<<1);   // Set clock polarity and phase to 1 and 1
    SPI5->CR1 |= (1<<2);        // Select STM32 as the master device
    SPI5->CR1 |= (1<<9);        // Software slave management
    SPI5->CR1 |= (1<<8);        // Communication permanently enabled (we enable/disable via software)
    SPI5->CR1 &= ~(1<<7);       // MSB transmitted first
    SPI5->CR1 &= ~(1<<11);      // 8-bit data frame format
    SPI5->CR1 &= ~(1<<15);      // bidirectional mode
    SPI5->CR1 |= (0x07<<3);     // Slowest baudrate possible
    SPI5->CR1 |= (1<<6);        // Enable SPI5

    GPIOG->ODR |= (1<<13);

    GPIOD->ODR &= ~(1<<2);      // PD2  -> IM0
    GPIOD->ODR |= (1<<4);       // PD4  -> IM1
    GPIOD->ODR |= (1<<5);       // PD5  -> IM2
    GPIOD->ODR &= ~(1<<7);      // PD7  -> IM3

    // Init the display
    //GPIOC->ODR &= ~(1<<2);      // chip select
    SELECT();
    ili9341_init();
    DESELECT();
    GPIOG->ODR |= (1<<14);

    // Set column and row range in (0-100, 0-100) range
    /*
    // column
    uint16_t h=239;
    uint16_t w=319;
    SELECT();
    spi5_cmd(0x2A);
    spi5_w8( 0x00 );      // starting x (high)
    spi5_w8( 0x00 );      // starting x (low)
    spi5_w8( h>>8 );      // ending column (high)
    spi5_w8( h&0xFF ); // ending column (low)
    DESELECT();
    // row
    SELECT();
    spi5_cmd(0x2B);
    spi5_w8(0x00);
    spi5_w8(0x00);
    spi5_w8( w>>8 );
    spi5_w8( w&0xFF ); // ending row (low)
    DESELECT();
    // Set "write to ram"
    SELECT();
    spi5_cmd(0x2C);
    DESELECT();
    */
    
    // TODO: provare a copiare l'esempio con interfaccia RGB e framebuffer


    // LTDC GPIO configuration
    // PC10 -> R2
    // PB0  -> R3
    // PA11 -> R4
    // PA12 -> R5
    // PB1  -> R6
    // PG6  -> R7
    // PA6  -> G2
    // PG10 -> G3
    // PB10 -> G4
    // PB11 -> G5
    // PC7  -> G6
    // PD3  -> G7
    // PD6  -> B2
    // PG11 -> B3
    // PG12 -> B4
    // PA3  -> B5
    // PB8  -> B6
    // PB9  -> B7


    // LTDC init
    //RCC->APB2ENR |= (1<<26);    // LTDC clock enable
    // Typical dot clock is about 6MHz (datasheet pag. 46)
    //LTDC->GCR |= (1<<0);        // Enable LTDC

    int tft_i=0;
    while (1) {
        //GPIOG->ODR |= (1<<13);
        //GPIOG->ODR |= (1<<14);

        // Write 320x240 pixels
        /*
        SELECT();
        for (tft_i=0; tft_i<(h+1)*(w+1); tft_i++) {
            spi5_w8(0xF8);
            spi5_w8(0x55);
            //spi5_w8(0x00);
        }
        DESELECT();
        */
        
    }
}

void spi5_w8(uint8_t data)
{
    SELECT();
    GPIOD->ODR |= (1<<13);                  // PD13 high (data mode)
    while ( !(SPI5->SR & (1<<1)) ) {};      // Wait for the TX buffer to be empty
    *(uint8_t*)&(SPI5->DR) = data;          // Send the data
    DESELECT();
}

void spi5_w16(uint16_t data)
{
    while ( !(SPI5->SR & (1<<1)) ) {};      // Wait for the TX buffer to be empty
    data = ((data & 0xff) << 8) | ((data & 0xff00) >> 8);   // Make the data be compliant with little-endianess of ARM
    *(uint16_t*)&(SPI5->DR) = data;         // Send the data
}

void spi5_cmd(uint8_t cmd)
{
    SELECT();
    GPIOD->ODR &= ~(1<<13);         // PD13 low (command mode)
    spi5_w8(cmd);                   // Write data
    while (SPI5->SR & (1<<7)) {};   // Wait for the BUSY flag to clear
    DESELECT();
}

void __attribute__((optimize("O0"))) delay_cycles(uint32_t ncycles)
{
    uint32_t i;
    for (i=0; i<ncycles; i++) {
        asm("NOP");
    }
}



void ili9341_init()
{
    /*
    //SOFTWARE RESET
    spi5_cmd(0x01);
    //HAL_Delay(1000);
    delay_cycles(2000000);
        
    //POWER CONTROL A
    spi5_cmd(0xCB);
    spi5_w8(0x39);
    spi5_w8(0x2C);
    spi5_w8(0x00);
    spi5_w8(0x34);
    spi5_w8(0x02);

    //POWER CONTROL B
    spi5_cmd(0xCF);
    spi5_w8(0x00);
    spi5_w8(0xC1);
    spi5_w8(0x30);

    //DRIVER TIMING CONTROL A
    spi5_cmd(0xE8);
    spi5_w8(0x85);
    spi5_w8(0x00);
    spi5_w8(0x78);

    //DRIVER TIMING CONTROL B
    spi5_cmd(0xEA);
    spi5_w8(0x00);
    spi5_w8(0x00);

    //POWER ON SEQUENCE CONTROL
    spi5_cmd(0xED);
    spi5_w8(0x64);
    spi5_w8(0x03);
    spi5_w8(0x12);
    spi5_w8(0x81);

    //PUMP RATIO CONTROL
    spi5_cmd(0xF7);
    spi5_w8(0x20);

    //POWER CONTROL,VRH[5:0]
    spi5_cmd(0xC0);
    spi5_w8(0x23);

    //POWER CONTROL,SAP[2:0];BT[3:0]
    spi5_cmd(0xC1);
    spi5_w8(0x10);

    //VCM CONTROL
    spi5_cmd(0xC5);
    spi5_w8(0x3E);
    spi5_w8(0x28);

    //VCM CONTROL 2
    spi5_cmd(0xC7);
    spi5_w8(0x86);

    //MEMORY ACCESS CONTROL
    spi5_cmd(0x36);
    spi5_w8(0x48);

    //PIXEL FORMAT
    spi5_cmd(0x3A);
    spi5_w8(0x55);

    //FRAME RATIO CONTROL, STANDARD RGB COLOR
    spi5_cmd(0xB1);
    spi5_w8(0x00);
    spi5_w8(0x18);

    //DISPLAY FUNCTION CONTROL
    spi5_cmd(0xB6);
    spi5_w8(0x08);
    spi5_w8(0x82);
    spi5_w8(0x27);

    //3GAMMA FUNCTION DISABLE
    spi5_cmd(0xF2);
    spi5_w8(0x00);

    //GAMMA CURVE SELECTED
    spi5_cmd(0x26);
    spi5_w8(0x01);

    //POSITIVE GAMMA CORRECTION
    spi5_cmd(0xE0);
    spi5_w8(0x0F);
    spi5_w8(0x31);
    spi5_w8(0x2B);
    spi5_w8(0x0C);
    spi5_w8(0x0E);
    spi5_w8(0x08);
    spi5_w8(0x4E);
    spi5_w8(0xF1);
    spi5_w8(0x37);
    spi5_w8(0x07);
    spi5_w8(0x10);
    spi5_w8(0x03);
    spi5_w8(0x0E);
    spi5_w8(0x09);
    spi5_w8(0x00);

    //NEGATIVE GAMMA CORRECTION
    spi5_cmd(0xE1);
    spi5_w8(0x00);
    spi5_w8(0x0E);
    spi5_w8(0x14);
    spi5_w8(0x03);
    spi5_w8(0x11);
    spi5_w8(0x07);
    spi5_w8(0x31);
    spi5_w8(0xC1);
    spi5_w8(0x48);
    spi5_w8(0x08);
    spi5_w8(0x0F);
    spi5_w8(0x0C);
    spi5_w8(0x31);
    spi5_w8(0x36);
    spi5_w8(0x0F);

    //EXIT SLEEP
    spi5_cmd(0x11);
    //HAL_Delay(120);
    delay_cycles(2000000);

    //TURN ON DISPLAY
    spi5_cmd(0x29);

    //STARTING ROTATION
    //ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
    */
   
    // We have 4-line serial interface I
    spi5_cmd(0x01); // software reset
    spi5_cmd(0xEF);
    spi5_w8(0x03);
    spi5_w8(0x80);
    spi5_w8(0x02);
    spi5_cmd(0xCF);
    spi5_w8(0x00);
    spi5_w8(0xC1);
    spi5_w8(0x30);
    spi5_cmd(0xED);
    spi5_w8(0x64);
    spi5_w8(0x03);
    spi5_w8(0x12);
    spi5_w8(0x81);
    spi5_cmd(0xE8);
    spi5_w8(0x85);
    spi5_w8(0x00);
    spi5_w8(0x78);
    spi5_cmd(0xCB);
    spi5_w8(0x39);
    spi5_w8(0x2C);
    spi5_w8(0x00);
    spi5_w8(0x34);
    spi5_w8(0x02);
    spi5_cmd(0xF7);
    spi5_w8(0x20);
    spi5_cmd(0xEA);
    spi5_w8(0x00);
    spi5_w8(0x00);
    // PWCTR1
    spi5_cmd(0xC0);
    spi5_w8(0x23);
    // PWCTR2
    spi5_cmd(0xC1);
    spi5_w8(0x10);
    // VMCTR1
    spi5_cmd(0xC5);
    spi5_w8(0x3E);
    spi5_w8(0x28);
    // VMCTR2
    spi5_cmd(0xC7);
    spi5_w8(0x86);
    // MADCTL
    spi5_cmd(0x36);
    spi5_w8(0x48);
    // VSCRSADD
    spi5_cmd(0x37);
    spi5_w8(0x00);
    // PIXFMT
    spi5_cmd(0x3A);
    spi5_w8(0x55);
    // FRMCTR1
    spi5_cmd(0xB1);
    spi5_w8(0x00);
    spi5_w8(0x18);
    // DFUNCTR
    spi5_cmd(0xB6);
    spi5_w8(0x08);
    spi5_w8(0x82);
    spi5_w8(0x27);
    spi5_cmd(0xF2);
    spi5_w8(0x00);
    // GAMMASET
    spi5_cmd(0x26);
    spi5_w8(0x01);
    // (Actual gamma settings)
    spi5_cmd(0xE0);
    spi5_w8(0x0F);
    spi5_w8(0x31);
    spi5_w8(0x2B);
    spi5_w8(0x0C);
    spi5_w8(0x0E);
    spi5_w8(0x08);
    spi5_w8(0x4E);
    spi5_w8(0xF1);
    spi5_w8(0x37);
    spi5_w8(0x07);
    spi5_w8(0x10);
    spi5_w8(0x03);
    spi5_w8(0x0E);
    spi5_w8(0x09);
    spi5_w8(0x00);
    spi5_cmd(0xE1);
    spi5_w8(0x00);
    spi5_w8(0x0E);
    spi5_w8(0x14);
    spi5_w8(0x03);
    spi5_w8(0x11);
    spi5_w8(0x07);
    spi5_w8(0x31);
    spi5_w8(0xC1);
    spi5_w8(0x48);
    spi5_w8(0x08);
    spi5_w8(0x0F);
    spi5_w8(0x0C);
    spi5_w8(0x31);
    spi5_w8(0x36);
    spi5_w8(0x0F);
    // Exit sleep mode.
    spi5_cmd(0x11);
    delay_cycles(2000000);
    // Display on.
    spi5_cmd(0x29);
    delay_cycles(2000000);
    // 'Normal' display mode.
    spi5_cmd(0x13);
    
}



