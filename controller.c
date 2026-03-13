#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "uart.h"
#include "utils.h"
#include "timer.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"

// Common interface includes
#include "common.h"
#include "gpio_if.h"
#include "pinmux.h"
#include "timer_if.h"
#include "uart_if.h"
#include "i2c_if.h"

// Custom includes
#include "utils/network_utils.h"

#include "pinmux.h"

//*****************************************************************************
//                 GLOBAL VARIABLES -- START
//*****************************************************************************
// ----- CSS Launchpad -----
#define APP_NAME            "AWS Text"
#define APPLICATION_VERSION "1.0.0"
#define CONSOLE             UARTA0_BASE
#define CONSOLE1            UARTA1_BASE
#define SPI_IF_BIT_RATE     100000
#define MASTER_MODE         1

#define GPIO_GROUP GPIOA0_BASE
#define GPIO_PIN   0x10

#if defined(ccs) || defined(gcc)
extern void (*const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// ----- Stopwatch -----
#define TIMERIO      TIMERA0_BASE
#define TIMERAWS     TIMERA1_BASE
#define IO_TIMEOUT   1000000 * 2
#define AWS_INTERVAL (30 * 1000000)

static volatile int stopwatchFlag = 0;
static volatile int pollShadowFlag = 0;

static volatile long sysTickValue = 0;

// ----- IR Remote -----
#define ZERO  0x60DF926D // ZERO    "01100000110111111001001001101101" COLOR
#define ONE   0x60DFC837 // ONE     "01100000110111111100100000110111"
#define TWO   0x60DF08F7 // TWO     "01100000110111110000100011110111"
#define THREE 0x60DF8877 // THREE   "01100000110111111000100001110111"
#define FOUR  0x60DFF00F // FOUR    "01100000110111111111000000001111"
#define FIVE  0x60DF30CF // FIVE    "01100000110111110011000011001111"
#define SIX   0x60DFB04F // SIX     "01100000110111111011000001001111"
#define SEVEN 0x60DFD02F // SEVEN   "01100000110111111101000000101111"
#define EIGHT 0x60DF10EF // EIGHT   "01100000110111110001000011101111"
#define NINE  0x60DF906F // NINE    "01100000110111111001000001101111"
#define MUTE  0x60DF48B7 // MUTE    "01100000110111110100100010110111" ENTER
#define LAST  0x60DFE01F // LAST    "01100000110111111110000000011111" DELETE

const unsigned long START_WIDTH = 4430 / 2;
const unsigned long ZERO_WIDTH = 550 / 2;
const unsigned long ONE_WIDTH = 1670 / 2;

const unsigned long ACCEPTABLE_JITTER = 100;
const unsigned long REPEAT_KEY_DELAY = 1500000 / 2; // 1.5s window to repeat a key

const char BUTTON2[] = "abc";
const char BUTTON3[] = "def";
const char BUTTON4[] = "ghi";
const char BUTTON5[] = "jkl";
const char BUTTON6[] = "mno";
const char BUTTON7[] = "pqrs";
const char BUTTON8[] = "tuv";
const char BUTTON9[] = "wxyz";

static volatile int signalFlag = 0;
static volatile int inputFlag = 0;

static volatile unsigned long pulseWidth[32] = { 0 };
static volatile unsigned long readBuffer = 0;
static volatile unsigned long numReadBits = 0;

// Synchronous Flags
static unsigned long sendFlag = 0;
static unsigned long deleteFlag = 0;

static unsigned long prevButton = 0;

// ----- Messaging -----
#define MAX_MSG_LEN 140

static volatile int msgRecvFlag = 0;

char message[MAX_MSG_LEN] = { 0 };
char received[MAX_MSG_LEN] = { 0 };
int messageIndex = 0;
int buttonIndex = 0;
int receiveIndex = 0;

int lastVersion = -1;
int lastTimestamp = -1;

int senderID = -1;

// ----- Display -----
#define BLACK   0x0000
#define BLUE    0x001F
#define GREEN   0x07E0
#define CYAN    0x07FF
#define RED     0xF800
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define CHAR_PIXEL_W  6
#define CHAR_PIXEL_H  8
#define CHAR_PER_LINE 20

const unsigned int NUM_COLORS = 7;
unsigned int COLORS[] = { WHITE, RED, MAGENTA, YELLOW, GREEN, BLUE, CYAN };
unsigned int appliedColors[MAX_MSG_LEN] = { 0 };
unsigned int receivedColors[MAX_MSG_LEN] = { 0 };
int colorIndex = 0;

// ----- Networking -----
#define SERVER_NAME \
    "ah5o5i77ylu7q-ats.iot.us-east-2.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT 8443
#define TOPIC           "eec172_cc3200" // CHANGE ME

#define PARTIAL_HEADER "/things/" TOPIC "/shadow HTTP/1.1\r\nHost: " SERVER_NAME "\r\nConnection: Keep-Alive\r\n"

#define GET_REQUEST "GET " PARTIAL_HEADER "\r\n"

#define POST_TEMPLATE "POST " PARTIAL_HEADER "Content-Type: application/json; charset=utf-8\r\nContent-Length: %d\r\n\r\n%s"

#define JSON_TEMPLATE               \
    "{\"state\":{\"desired\":{"     \
    "\"deviceid\":%u,"            \
    "\"message\":\"%s\","           \
    "\"colors\":\"%s\""             \
    "}}}\r\n"

#define BUILD_HOUR   ((__TIME__[0] - '0') * 10 + (__TIME__[1] - '0'))
#define BUILD_MINUTE ((__TIME__[3] - '0') * 10 + (__TIME__[4] - '0'))
#define BUILD_SECOND ((__TIME__[6] - '0') * 10 + (__TIME__[7] - '0'))
#define BUILD_YEAR                                            \
    ((__DATE__[7] - '0') * 1000 + (__DATE__[8] - '0') * 100 + \
     (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0'))
#define BUILD_MONTH                                  \
    (__DATE__[0] == 'J' && __DATE__[1] == 'a'   ? 1  \
     : __DATE__[0] == 'F'                       ? 2  \
     : __DATE__[0] == 'M' && __DATE__[2] == 'r' ? 3  \
     : __DATE__[0] == 'A' && __DATE__[1] == 'p' ? 4  \
     : __DATE__[0] == 'M' && __DATE__[2] == 'y' ? 5  \
     : __DATE__[0] == 'J' && __DATE__[2] == 'n' ? 6  \
     : __DATE__[0] == 'J' && __DATE__[2] == 'l' ? 7  \
     : __DATE__[0] == 'A' && __DATE__[1] == 'u' ? 8  \
     : __DATE__[0] == 'S'                       ? 9  \
     : __DATE__[0] == 'O'                       ? 10 \
     : __DATE__[0] == 'N'                       ? 11 \
                                                : 12)
#define BUILD_DATE \
    ((__DATE__[4] == ' ' ? 0 : (__DATE__[4] - '0') * 10) + (__DATE__[5] - '0'))

#define DATE   BUILD_DATE   /* Current Date */
#define MONTH  BUILD_MONTH  /* Month 1-12 */
#define YEAR   BUILD_YEAR   /* Current year */
#define HOUR   BUILD_HOUR   /* Time - hours */
#define MINUTE BUILD_MINUTE /* Time - minutes */
#define SECOND BUILD_SECOND /* Time - seconds */

#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

#define MAX_BUFFER_LEN 1400
#define USER_ID        1

static int socketID = -1;

//*****************************************************************************
//                 GLOBAL VARIABLES -- END
//*****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES -- START
//****************************************************************************
// ----- Accelerometer -----
int8_t GetAccelRoll()
{
    unsigned char ucDevAddr = 24;
    unsigned char reg = 3;
    int8_t aucRdDataBuf;
    RET_IF_ERR(I2C_IF_Write(ucDevAddr, &reg, 1, 0));
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf, 1));
    return aucRdDataBuf;
}

int8_t GetAccelPitch()
{
    unsigned char ucDevAddr = 24;
    unsigned char reg = 5;
    int8_t aucRdDataBuf;
    RET_IF_ERR(I2C_IF_Write(ucDevAddr, &reg, 1, 0));
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf, 1));
    return aucRdDataBuf;
}

// ----- CSS Launchpad -----
static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long) &__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();

    PinMuxConfig();
    InitTerm();
    ClearTerm();

    I2C_IF_Open(I2C_MASTER_MODE_FST);

    // Timer Init
//    Timer_IF_Init(PRCM_TIMERA0, TIMERIO, TIMER_CFG_PERIODIC, TIMER_A, 0);
//    Timer_IF_Init(PRCM_TIMERA1, TIMERAWS, TIMER_CFG_PERIODIC, TIMER_A, 0);
//    Timer_IF_IntSetup(TIMERIO, TIMER_A, StopwatchHandler);
//    Timer_IF_IntSetup(TIMERAWS, TIMER_A, UpdateHandler);
//    StartStopwatch();
//    ResetUpdateInterval();

    // SPI for OLED
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(
            GSPI_BASE,
            MAP_PRCMPeripheralClockGet(PRCM_GSPI),
            SPI_IF_BIT_RATE,
            SPI_MODE_MASTER,
            SPI_SUB_MODE_0,
            (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVEHIGH
                    | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    // OLED display
//    Adafruit_Init();
//    fillScreen(BLACK);
//    setTextColor(WHITE, BLACK);
//    setTextSize(1);
//    setCursor(0, 64);
//    char buf[CHAR_PER_LINE] = {0};
//    sprintf(buf, "As Device %d", USER_ID);
//    Outstr(buf);
//
//    UART_PRINT("Board Initialized\n\r");

    // GPIO Registers
//    MAP_GPIOIntRegister(GPIO_GROUP, EdgeHandler);
//    MAP_GPIOIntTypeSet(GPIO_GROUP, GPIO_PIN, GPIO_BOTH_EDGES);
//    MAP_GPIOIntClear(GPIO_GROUP, MAP_GPIOIntStatus(GPIO_GROUP, false));
//    MAP_GPIOIntEnable(GPIO_GROUP, GPIO_PIN);
}


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES -- END
//****************************************************************************


const int8_t AccelPitchOffset = 3;
const int8_t AccelRollOffset = 0;
const int8_t AccelPitchThreshold = 10;
const int8_t AccelRollThreshold = 10;
const int8_t AccelPitchSensitivity = 1;
const int8_t AccelRollSensitivity = 1;

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long lRetVal = -1;
    BoardInit();

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    // Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();

    udp_socket();
    controller_broadcast();
    listen_for_controller();

    // Main loop
    while (1)
    {
        if (1)
        {
            int8_t pitch = GetAccelPitch() + AccelPitchOffset;
            int8_t roll = GetAccelRoll() + AccelRollOffset;

            if (pitch >= AccelPitchThreshold || pitch <= -AccelPitchThreshold)
            {
                char buffer[UDP_MAX_SIZE];
                sprintf(buffer, "%c%d", pitch > 0 ? 'W' : 'S', pitch > 0 ? pitch : pitch * -1);
                send_udp_peer(buffer);
            }
            if (roll >= AccelRollThreshold || roll <= -AccelPitchThreshold)
            {
                char buffer[UDP_MAX_SIZE];
                sprintf(buffer, "%c%d", roll > 0 ? 'A' : 'D', roll > 0 ? roll : roll * -1);
                send_udp_peer(buffer);
            }
            MAP_UtilsDelay(1000000);
        }

        // Character locked in after delay
//        if (sysTickValue > REPEAT_KEY_DELAY) {
//            prevButton = 0;
//            StartStopwatch();
//            StopStopwatch();
//        }
//
//        if (inputFlag) {
//            inputFlag = 0;
//
//            if (DecodeInput()) {
//                DrawCharacter(messageIndex - 1);
//            }
//        }
//
//        if (deleteFlag) {
//            deleteFlag = 0;
//            DrawCharacter(messageIndex + 1);
//        }
//
//        if (msgRecvFlag) {
//            msgRecvFlag = 0;
//            DrawReceived();
//        }
//
//        if (sendFlag) {
//            pollShadowFlag = 0;
//            pollShadow();
//
//            sendMessage();
//
//            // Clear bottom half of the screen
//            fillRect(0, 72, 129, 129, BLACK);
//
//            // Reset message buffer
//            message[0] = '\0';
//            messageIndex = 0;
//            sendFlag = 0;
//        }
//
//        if (pollShadowFlag) {
//            pollShadowFlag = 0;
//            pollShadow();
//        }
    }

    // EXIT and CLEANUP
    UART_PRINT("\n\r Exiting the Application\n\r");
    sl_Stop(SL_STOP_TIMEOUT);
    while (1)
        ;
}

