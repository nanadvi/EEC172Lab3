// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "timer.h"
#include "systick.h"
#include "spi.h"
#include "uart.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>

// Common interface includes
#include "uart_if.h"
#include "timer_if.h"
#include "pin_mux_config.h"

// Adafruit includes
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"

// draw functions includes
#include "test.h"


#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long time;
int interruptCounter;
int pulseCounter;
char prevRead, currRead;
char buffer[64];
char receiverBuffer[64];
unsigned long _time;
unsigned long _readTime;
unsigned long timeInterval[35] = {};
unsigned long timeOfInterrupt[35] = {};
unsigned int bitSequence[35] = {};
int receiverLineNumber;
int readIndex;
int buttons[12][35] = {
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,1,0}, // 0
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0}, // 1
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0}, // 2
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,0,1,0,1,1,1,1,1,1,0}, // 3
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,1,1,0}, // 4
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,0,1,0,0,0,0,0,0,1,0,1,1,1,1,1,0}, // 5
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0}, // 6
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,0,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0}, // 7
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,1,1,1,0}, // 8
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,0,1,0,1,0,0,0,0,1,0,1,0,1,1,1,1,0}, // 9
                       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // DELETE
                       {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,0} // MUTE
};
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;
static PinSetting gpioin = { .port = GPIOA0_BASE, .pin = 0x40 };

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************

static void BoardInit(void);
//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

void TimerRefIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    unsigned long ulInts;
    ulInts = TimerIntStatus(TIMERA0_BASE, true);
    TimerIntClear(TIMERA0_BASE, ulInts);
    _time++;
    _readTime++;
}


int compareTwoSequence(int sequence1[35], int sequence2[35]) {
    int i;
    for (i = 0; i < 35; i++) {
        if (sequence1[i] != sequence2[i])
            return 0;
    }
    return 1;
}

char compareBitPatterns() {
    int row;
    int col;
    for (row = 0; row < 12; row++) {
        if (compareTwoSequence(bitSequence, buttons[row])) {
            if (row == 10)
                return 'D';
            else if (row == 11)
                return 'M';
            else
                return row + '0';
        }
    }
    return 'F';
}

static void GPIOA2IntHandler(void)
{
    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (gpioin.port, false);
    MAP_GPIOIntClear(gpioin.port, ulStatus);        // clear interrupts on GPIOA2
    time = _time;
    _time = 0;
    int _interrupt = interruptCounter;
    if(interruptCounter < 35)
    {
        bitSequence[interruptCounter] = decode(time);
        timeOfInterrupt[interruptCounter] = time;
        interruptCounter++;
    }
    if(interruptCounter == 35)
    {
        // GPIOIntDisable(gpioin.port, gpioin.pin);
        // Don't tick till we read and decode the bits
        TimerDisable(TIMERA0_BASE, TIMER_A);
        // Stop taking ticks too
        // interruptCounter = 0;
        // initializeArr();
        // GPIOIntEnable(gpioin.port, gpioin.pin);
    }
}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************

static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void timerInit()
{
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 255);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerRefIntHandler);
    TimerLoadSet(TIMERA0_BASE, TIMER_A, 10000);
    TimerEnable(TIMERA0_BASE, TIMER_A);
}

void initializeArr()
{
    int i;
    for(i = 0; i < 35; i++)
    {
        timeInterval[i] = 0;
        timeOfInterrupt[i] = 0;
        bitSequence[i] = 0;
    }
}

void printBuffer()
{
    int i;
    for(i = 0; i < readIndex; i++)
    {
        Report("%c", buffer[i]);
        drawChar(6*i, 30, buffer[i], WHITE, BLACK, 0x01);
    }
    Report("\n\r");

}

int decode(unsigned long time){

    if(time > 0 && time < 10){
        return 0;
    }
    else if(time > 10 && time < 20){
        return 1;
    }
    else if(time > 1000)
    {
        interruptCounter = 0;
    }
    return 0;
}

void sendMessage(char message[64]) {
    // Disable UART Interrupt while sending characters
    MAP_UARTIntDisable(UARTA1_BASE, UART_INT_RX | UART_INT_RT);
    int i;
    for (i=0; i<64; i++) {
        // Sends the character in the buffer array to the UART register
        MAP_UARTCharPut(UARTA1_BASE, message[i]);
        // Creates a small delay to ensure that the hardware functions correctly
        MAP_UtilsDelay(80000);

    }
    // Enables UART interrupts
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX | UART_INT_RT);

}

void receiveMessage() {
    Report("H\n\r");
    int i;
    unsigned long ulStatus;
    ulStatus = MAP_UARTIntStatus(UARTA1_BASE, true);
    UARTIntClear(UARTA1_BASE, ulStatus);

    MAP_UtilsDelay(80000);

    for (i=0; i<8; i++) {
        receiverBuffer[i] = MAP_UARTCharGet(UARTA1_BASE);
        MAP_UtilsDelay(80000);
        if (receiverBuffer[i] != ' ') {
            drawChar(6*i, receiverLineNumber, receiverBuffer[i], WHITE, BLACK, 0x01);
        }
        MAP_UtilsDelay(80000);
    }
    receiverLineNumber += 10;
    memset(receiverBuffer, ' ', 64);
}

void SPI_Init() {
    // Reset SPI

    MAP_SPIReset(GSPI_BASE);

    //Enables the transmit and/or receive FIFOs.

    //Base address is GSPI_BASE, SPI_TX_FIFO || SPI_RX_FIFO are the FIFOs to be enabled

    MAP_SPIFIFOEnable(GSPI_BASE, SPI_TX_FIFO || SPI_RX_FIFO);



    // Configure SPI interface

    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),

                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,

                     (SPI_SW_CTRL_CS |

                     SPI_4PIN_MODE |

                     SPI_TURBO_OFF |

                     SPI_CS_ACTIVELOW |

                     SPI_WL_8));



    // Enable SPI for communication

    MAP_SPIEnable(GSPI_BASE);
}

void UARTInt_Init() {
    //configure Uart

    MAP_UARTConfigSetExpClk(UARTA1_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA1),

                UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |

                UART_CONFIG_PAR_NONE));

    UARTEnable(UARTA1_BASE);

    // Disable FIFO so RX interrupt triggers on any character

    MAP_UARTFIFODisable(UARTA1_BASE);

    // Set interrupt handlers

    MAP_UARTIntRegister(UARTA1_BASE,receiveMessage);

    // Clear any interrupts that may have been present

    MAP_UARTIntClear(UARTA1_BASE, UART_INT_RX);

    // Enable interrupt

    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX);

    UARTFIFOEnable(UARTA1_BASE);
}

//***************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************


int main() {
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    // UART config
    UARTInt_Init();

    // SPI config
    SPI_Init();

    // Adafruit init
    Adafruit_Init();

    fillScreen(BLACK);

    // Initializing the terminal
    InitTerm();
    // Clearing the terminal
    ClearTerm();
    // Initialize Timer
    timerInit();
    // Register the interrupt handlers
    MAP_GPIOIntRegister(gpioin.port, GPIOA2IntHandler);
    //
    // Configure rising edge interrupts on PIN 61
    //
    MAP_GPIOIntTypeSet(gpioin.port, gpioin.pin, GPIO_RISING_EDGE);
    ulStatus = MAP_GPIOIntStatus (gpioin.port, false);
    MAP_GPIOIntClear(gpioin.port, ulStatus);            // clear interrupts on GPIOA2

    // Initialize global variables
    interruptCounter = 0;
    pulseCounter = 0;
    initializeArr();
    _time = 0;
    readIndex = 0;
    receiverLineNumber = 64;
    memset(buffer, ' ', 64);
    memset(receiverBuffer, ' ', 64);
    int fPressed = 0;

    MAP_GPIOIntEnable(gpioin.port, gpioin.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\                      LAB 3                        \n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");

    // setting timer value to 0
    TimerValueSet(TIMERA0_BASE, TIMER_A, 0);

    int deleteFlag = 0;

    while (1) {
        // logic for matching the pattern and displaying character on OLED
        // 35 RISING HIGH INTERRUPTS
        if(interruptCounter == 35)
        {
            MAP_GPIOIntDisable(gpioin.port, gpioin.pin);
            
            currRead =  compareBitPatterns();

            if(currRead == 'M')
            {
                Report("\n\r");
                printBuffer();
                memset(buffer, ' ', 64);
                readIndex = 0;
                // Send message to the other board
            }
            else if (currRead == 'F' && _readTime > 2000) {
                ;
            }
            else if (currRead != 'F' && _readTime > 2000) {
                char output = ' ';
                switch(currRead) {
                case '2':
                    output = 'a';
                    break;

                case '3':
                    output = 'd';
                    break;

                case '4':
                    output = 'g';
                    break;

                case '5':
                    output = 'j';
                    break;

                case '6':
                    output = 'm';
                    break;

                case '7':
                    output = 'p';
                    break;

                case '8':
                    output = 't';
                    break;

                case '9':
                    output = 'w';
                    break;

                case '0':
                    output = ' ';
                    break;
                case 'D':
                    deleteFlag = 1;
                    break;
                }
                buffer[readIndex] = output;
                Report("%c", buffer[readIndex]);
                drawChar(6*readIndex, 0, buffer[readIndex], WHITE, BLACK, 0x01);
                if (deleteFlag == 0)
                    readIndex++;
            }
            else if (currRead == 'F' && _readTime < 2700) {
                char output = ' ';
                switch(buffer[readIndex-1]) {
                case 'a':
                    output = 'b';
                    break;

                case 'b':
                    output = 'c';
                    break;

                case 'c':
                    output = 'a';
                    break;

                case 'd':
                    output = 'e';
                    break;

                case 'e':
                    output = 'f';
                    break;

                case 'f':
                    output = 'd';
                    break;

                case 'g':
                    output = 'h';
                    break;

                case 'h':
                    output = 'i';
                    break;

                case 'i':
                    output = 'g';
                    break;

                case 'j':
                    output = 'k';
                    break;

                case 'k':
                    output = 'l';
                    break;

                case 'l':
                    output = 'j';
                    break;

                case 'm':
                    output = 'n';
                    break;

                case 'n':
                    output = 'o';
                    break;

                case 'o':
                    output = 'm';
                    break;

                case 'p':
                    output = 'q';
                    break;

                case 'q':
                    output = 'r';
                    break;

                case 'r':
                    output = 's';
                    break;

                case 's':
                    output = 'p';
                    break;

                case 't':
                    output = 'u';
                    break;

                case 'u':
                    output = 'v';
                    break;

                case 'w':
                    output = 'x';
                    break;

                case 'x':
                    output = 'y';
                    break;

                case 'y':
                    output = 'z';
                    break;

                case 'z':
                    output = 'w';
                    break;
                }
                buffer[readIndex-1] = output;
                Report("%c", buffer[readIndex-1]);
                drawChar(6*(readIndex-1), 0, buffer[readIndex-1], WHITE, BLACK, 0x01);
            }
            if((currRead == 'D' || deleteFlag == 1) && readIndex > 0)
            {
                // Pressing the delete button
                Report("D");
                Report("%d", readIndex);
                buffer[readIndex] = ' ';
                readIndex--;
                drawChar(6*readIndex, 0, ' ', WHITE, BLACK, 0x01);
                deleteFlag = 0;

            }

            // replace above line with code to print to OLED
             _readTime = 0;

            interruptCounter = 0;
            initializeArr();
            MAP_GPIOIntEnable(gpioin.port, gpioin.pin);
            TimerEnable(TIMERA0_BASE, TIMER_A);
        }
    }
}