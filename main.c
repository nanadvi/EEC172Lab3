
//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

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

#define SPI_IF_BIT_RATE  800000
#define TR_BUFF_SIZE     100
#define DC_BIAS 388;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long time;
char prevRead, currRead;
char receiverBuffer[64];
char pressedButtons[64];
char readBuffer[64];
unsigned long _time;
unsigned long _readTime;
int receiverLineNumber;
int readIndex, delFlag, charRead, sysTickFlag, goertzelsFlag, new_dig, bPressed,fPressed;
int power_all[8];
int coeff[8] = {31548, 31281, 30951, 30556, 29144, 28361, 27409, 26258}; // array to store the calculated coefficients
int f_tone[8]={697, 770, 852, 941, 1209, 1336, 1477, 1633}; // frequencies of rows & columns
char ch = ' ';
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;
static PinSetting gpioin = { .port = GPIOA0_BASE, .pin = 0x40 };
static PinSetting adc = { .port = GPIOA0_BASE, .pin = 0x40 };
static PinSetting oled = { .port = GSPI_BASE, .pin = 0x00000007 };
volatile int buffer[410]; // stores the number of samples
int MAX_BUFFER_COUNT = 410; // max number of samples from ADC
int bufferCount = 0; // counts the number of samples from the ADC

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

static void SystickIntHandler(void)
{
     sysTickFlag = 1;
     _time++;
     _readTime++;
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
    /*
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 255);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerRefIntHandler);
    TimerLoadSet(TIMERA0_BASE, TIMER_A, 10000);
    TimerEnable(TIMERA0_BASE, TIMER_A);
    */
    SysTickIntDisable();
    //Set the countdown time to about .1 ms
    SysTickPeriodSet(5000);
    //Registers the Systick interrupt handler
    SysTickIntRegister(SystickIntHandler);

    SysTickIntEnable();
    SysTickEnable();
}

void initializeArr()
{
    int i;
    for(i = 0; i < 410; i++)
    {
        buffer[i] = 0;
    }
}

void printArr()
{
    int i;
    for(i = 0; i < charRead; i++) {
        Report("%c\n\r", readBuffer[i]);
    }
    Report("\n\r");
}

void printBuffer()
{
    int i;
//    Report("%d\n\r", readIndex);
    for(i = 0; i < charRead; i++) {
        Report("%c", readBuffer[i]);
//        drawChar(6*i, 30, buffer[i], WHITE, BLACK, 0x01);
    }
    Report("\n\r");
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
        Report("%c", receiverBuffer[i]);
//        if (receiverBuffer[i] != ' ') {
//            drawChar(6*i, receiverLineNumber, receiverBuffer[i], WHITE, BLACK, 0x01);
//        }
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

void UARTInt_Init()
{
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

void GetData()
{
    unsigned char b0, b1;
    unsigned long c0, c1;
    // Enable ADC chip select
    MAP_SPICSEnable(GSPI_BASE);
//    GPIOPinWrite(adc.pin, adc.port, 0x00);
    // Put dummy data
    MAP_SPIDataPut(GSPI_BASE, 0x0);
    // Get the first byte
    MAP_SPIDataGet(GSPI_BASE, &c0);
    // Put dummy data
    MAP_SPIDataPut(GSPI_BASE, 0x00);
    // Get the second byteS
    MAP_SPIDataGet(GSPI_BASE, &c1);

    printf("Buffer Count: %d", bufferCount);

    buffer[bufferCount] = (int)(((c0 & 0x1f) << 5) + ((c1 & 0xf8) >> 3)) - DC_BIAS;

    if (bufferCount+1 >= 410) {
        bufferCount = 0;
        goertzelsFlag = 1;
    }
    else {
        goertzelsFlag = 0;
    }

    b0 = (unsigned char) c0;
    b1 = (unsigned char) c1;
    printf("%u\n", b0);
    printf("%u\n", b1);
    sysTickFlag = 0;
    MAP_SPICSDisable(GSPI_BASE);
//    GPIOPinWrite(adc.pin, adc.port, 0xff);

    // Enabling OLED CS
//    GPIOPinWrite(oled.pin, oled.port, 0x0);
}

void DisplayBanner()
{
    Message("\t\t****************************************************\n\r");
    Message("\t\t\                      LAB 3                        \n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");
}

long int goertzel(int sample[], long int coeff, int N)
//---------------------------------------------------------------//
{
//initialize variables to be used in the function
int Q, Q_prev, Q_prev2,i;
long prod1,prod2,prod3,power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i=0; i<N; i++) // loop N times and calculate Q, Q_prev, Q_prev2 at each iteration
        {
            Q = (sample[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
            Q_prev2 = Q_prev;                                    // shuffle delay elements
            Q_prev = Q;
        }

        //calculate the three products used to calculate power
        prod1=( (long) Q_prev*Q_prev);
        prod2=( (long) Q_prev2*Q_prev2);
        prod3=( (long) Q_prev *coeff)>>14;
        prod3=( prod3 * Q_prev2);

        power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

        return power;
}

char post_test(void)
//---------------------------------------------------------------//
{
    // printf("%lu\n", SysTickValueGet());
    // initialize variables to be used in the function
    int i,row,col,max_power;
    char currPressed = ' ';

    char row_col[4][4] =       // array with the order of the digits in the DTMF system
            {
             {'1', '2', '3', 'A'},
             {'4', '5', '6', 'B'},
             {'7', '8', '9', 'C'},
             {'*', '0', '#', 'D'}
            };
    // find the maximum power in the row frequencies and the row numbe
    max_power=0;            //initialize max_power=0

    for(i=0;i<4;i++)        //loop 4 times from 0>3 (the indecies of the rows)
    {
        if (power_all[i] > max_power)   //if power of the current row frequency > max_power
        {
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
        }
    }

    // find the maximum power in the column frequencies and the column number

    max_power=0;            //initialize max_power=0

    for(i=4;i<8;i++)        //loop 4 times from 4>7 (the indecies of the columns)
    {
        if (power_all[i] > max_power)   //if power of the current column frequency > max_power
        {
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
        }
    }

    if(power_all[col]<100000 && power_all[row]<100000) //if the maximum powers equal zero > this means no signal or inter-digit pause
        new_dig=1;                             //set new_dig to 1 to display the next decoded digit


    if((power_all[col]>100000 && power_all[row]>100000) && new_dig==1) // check if maximum powers of row & column exceed certain threshold AND new_dig flag is set to 1
    {

        // Report("%c\n\r", row_col[row][col-4]);
        currPressed = row_col[row][col-4];
        new_dig=0;                                              // set new_dig to 0 to avoid displaying the same digit again.
    }
    return currPressed;
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

int main()
{
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();

    UARTInt_Init();

    // SPI config
    SPI_Init();

    // Adafruit init
    Adafruit_Init();

    // Initializing the terminal
    InitTerm();
    // Clearing the terminal
    ClearTerm();

    // Initialize Timer
    timerInit();

    //set the chip select to high
    GPIOPinWrite(adc.pin, adc.port, 0xff);

    // Register the interrupt handlers
    // GpioIntInit();

    // Initialize global variables
    initializeArr();
    charRead = 0;
    _time = 0;
    sysTickFlag = 0;
    delFlag = 0;
    // Keep time between successive button press
    _readTime = 0;
    // Index for all the pressed buttons
    bPressed = 0;
    goertzelsFlag = 0;
    fPressed = 0;
    // Message array
    memset(readBuffer, ' ', 64);
    // Button press history
    memset(pressedButtons, ' ', 64);
    // readIndex = 0;
    // receiverLineNumber = 64;
    // memset(receiverBuffer, ' ', 64);

    DisplayBanner();

    fillScreen(BLACK);

    int deleteFlag = 0;
    int muteFlag = 0;
    while (1) {
//        drawChar(0, 0, 'c', WHITE, BLACK, 0x01);
        char buttonPressed = ' ';
        char currentCharRead = ' ';
        if (sysTickFlag == 1) {
            unsigned char b0, b1;
            unsigned long c0, c1;
            // Enable ADC chip select
            MAP_SPICSEnable(GSPI_BASE);
//            GPIOPinWrite(adc.pin, adc.port, 0x0);
            // Put dummy data
            MAP_SPIDataPut(GSPI_BASE, 0x0);
            // Get the first byte
            MAP_SPIDataGet(GSPI_BASE, &c0);
            // Put dummy data
            MAP_SPIDataPut(GSPI_BASE, 0x00);
            // Get the second byteS
            MAP_SPIDataGet(GSPI_BASE, &c1);
//            GPIOPinWrite(adc.pin, adc.port, 0x1);
            MAP_SPICSDisable(GSPI_BASE);

//            printf("Buffer Count: %d", bufferCount);

            buffer[bufferCount] = (int)(((c0 & 0x1f) << 5) + ((c1 & 0xf8) >> 3)) - DC_BIAS;

            if (bufferCount+1 >= 410) {
                bufferCount = 0;
                goertzelsFlag = 1;
                // MAP_SPICSDisable(GSPI_BASE);
            }
            else {
                bufferCount++;
            }
            b0 = (unsigned char) c0;
            b1 = (unsigned char) c1;

            if (goertzelsFlag == 1) {
                SysTickIntDisable();
                SysTickDisable();
                // printf("received enough number of samples");

                int i;
                for (i = 0; i < 8; i++){
                    power_all[i] = goertzel(buffer, coeff[i], MAX_BUFFER_COUNT);
                }
                buttonPressed = post_test();
                if(buttonPressed != ' ' && (_time % 410 != 0))
                {
                    pressedButtons[bPressed] = buttonPressed;
                    bPressed++;



                    if(_readTime < 2500)
                    {
                        // Multi-tap
                        // char lastRead =
                        char output = '-';
                        switch(readBuffer[charRead-1]) {
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

                        if (output != '-') {
                            readBuffer[charRead-1] = output;
                        }
                        Report("%c", readBuffer[charRead-1]);
                        // drawChar

                    }
                    else
                    {
                        char output = '-';
                        switch(buttonPressed)
                        {
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
                        case '*':
                            deleteFlag = 1;
//                            Report("d\n\r");
                            break;
                        case '#':
                            muteFlag = 1;
                            break;

                        }
                        if (output != '-') {
                            readBuffer[charRead] = output;
                        }
                        Report("%c", readBuffer[charRead]);
                        if (deleteFlag == 0)
                            charRead++;
                    }
                    if (deleteFlag == 1 && _readTime > 2500) {
                           readBuffer[charRead] = ' ';
                           charRead--;
                           deleteFlag = 0;
                    }
                    if (muteFlag == 1 && _readTime > 2500) {
                        printBuffer();
                        sendMessage(readBuffer);
                        memset(readBuffer, ' ', 64);
                        charRead = 0;
                    }

                    // drawChar
                    // if (delete)
                    _readTime = 0;

                }
                goertzelsFlag = 0;
            }
            sysTickFlag = 0;
            _time = 0;
            SysTickIntEnable();
            SysTickPeriodSet(5000);
            SysTickEnable();
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
