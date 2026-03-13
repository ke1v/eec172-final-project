//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     -   AWS Texting
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
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
#include "glcdfont.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

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

static volatile unsigned long pulseWidth[32] = {0};
static volatile unsigned long readBuffer = 0;
static volatile unsigned long numReadBits = 0;

// Synchronous Flags
static unsigned long sendFlag = 0;
static unsigned long deleteFlag = 0;

static unsigned long prevButton = 0;

// ----- Messaging -----
#define MAX_MSG_LEN 140

static volatile int msgRecvFlag = 0;

char message[MAX_MSG_LEN] = {0};
char received[MAX_MSG_LEN] = {0};
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
unsigned int COLORS[] = {WHITE, RED, MAGENTA, YELLOW, GREEN, BLUE, CYAN};
unsigned int appliedColors[MAX_MSG_LEN] = {0};
unsigned int receivedColors[MAX_MSG_LEN] = {0};
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

#define MAX_BUFFER_LEN 1400
#define USER_ID        1

static int socketID = -1;

//*****************************************************************************
//                 GLOBAL VARIABLES -- END
//*****************************************************************************

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES -- START
//****************************************************************************
// ----- Stopwatch -----
int inRange(int value, int target, int range) {
    return value >= target - range && value <= target + range;
}

void StartStopwatch() {
    sysTickValue = 0;
    stopwatchFlag = 1;
    Timer_IF_Start(TIMERIO, TIMER_A, 1);
}

void StopStopwatch() {
    stopwatchFlag = 0;
    Timer_IF_Stop(TIMERIO, TIMER_A);
}

void StopwatchHandler(void) {
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(TIMERIO);

    if (stopwatchFlag) {
        sysTickValue++;
    }

    if (sysTickValue > IO_TIMEOUT) {
        StopStopwatch();
    }
}

void ResetUpdateInterval() {
    Timer_IF_Start(TIMERAWS, TIMER_A, AWS_INTERVAL);
}

void UpdateHandler(void) {
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(TIMERAWS);

    pollShadowFlag = 1;
}

// ----- IR Remote -----
void EdgeHandler(void) {
    MAP_GPIOIntClear(GPIO_GROUP, MAP_GPIOIntStatus(GPIO_GROUP, false));

    if (MAP_GPIOPinRead(GPIO_GROUP, GPIO_PIN)) {
        StartStopwatch();
    } else {
        StopStopwatch();

        if (sysTickValue <= 0) {
            return;
        }

        if (signalFlag) {
            if (inRange(sysTickValue, ZERO_WIDTH, ACCEPTABLE_JITTER)) {
                // Zero
                readBuffer = (readBuffer << 1);
                pulseWidth[numReadBits] = sysTickValue;
                numReadBits++;
            } else if (inRange(sysTickValue, ONE_WIDTH, ACCEPTABLE_JITTER)) {
                // One
                readBuffer = (readBuffer << 1) + 1;
                pulseWidth[numReadBits] = sysTickValue;
                numReadBits++;
            }
            if (numReadBits >= 32) {
                signalFlag = 0;
                inputFlag = 1;
                numReadBits = 0;
            }
        } else if (inRange(sysTickValue, START_WIDTH, ACCEPTABLE_JITTER)) {
            signalFlag = 1;
        }
    }
}

int DecodeInput(void) {
    if (readBuffer == ONE) {
        colorIndex++;
        colorIndex %= NUM_COLORS;
        return 1;
    }

    // Repeat button press
    if (readBuffer == prevButton) {
        buttonIndex++;
        messageIndex--;
    } else {
        buttonIndex = 0;
    }

    if (messageIndex <= 0) {
        messageIndex = 0;
    }

    prevButton = readBuffer; // for multipress
    appliedColors[messageIndex] = colorIndex;

    if (readBuffer == ZERO) {
        message[messageIndex] = ' ';
        prevButton = 0;
    } else if (readBuffer == TWO) {
        message[messageIndex] = BUTTON2[buttonIndex % 3];
    } else if (readBuffer == THREE) {
        message[messageIndex] = BUTTON3[buttonIndex % 3];
    } else if (readBuffer == FOUR) {
        message[messageIndex] = BUTTON4[buttonIndex % 3];
    } else if (readBuffer == FIVE) {
        message[messageIndex] = BUTTON5[buttonIndex % 3];
    } else if (readBuffer == SIX) {
        message[messageIndex] = BUTTON6[buttonIndex % 3];
    } else if (readBuffer == SEVEN) {
        message[messageIndex] = BUTTON7[buttonIndex % 4];
    } else if (readBuffer == EIGHT) {
        message[messageIndex] = BUTTON8[buttonIndex % 3];
    } else if (readBuffer == NINE) {
        message[messageIndex] = BUTTON9[buttonIndex % 4];
    } else if (readBuffer == LAST) {
        deleteFlag = 1;
        prevButton = 0;
        message[messageIndex] = '\0';
        if (messageIndex > 0) {
            messageIndex--;
        }
        return 1;
    } else if (readBuffer == MUTE) {
        message[messageIndex] = '\0';
        prevButton = 0;
        sendFlag = 1;
        return 1;
    } else {
        return 0; // Ignore non-mapped buttons
    }

    if (messageIndex >= MAX_MSG_LEN - 1) {
        message[messageIndex] = '\0';
    } else {
        messageIndex++;
        StartStopwatch();
    }

    return 1;
}

// ----- Messaging -----

// ----- Display -----
void DrawCursor() {
    int row = messageIndex / CHAR_PER_LINE;
    int col = messageIndex % CHAR_PER_LINE;
    int x = col * CHAR_PIXEL_W;
    int y = row * CHAR_PIXEL_H + 72;
    fillRect(x, y, CHAR_PIXEL_W, CHAR_PIXEL_H, COLORS[colorIndex]);
}

void DrawCharacter(unsigned int charIndex) {
    int row = charIndex / CHAR_PER_LINE;
    int col = charIndex % CHAR_PER_LINE;
    int x = col * CHAR_PIXEL_W;
    int y = row * CHAR_PIXEL_H + 72;
    // Clear character
    fillRect(x, y, CHAR_PIXEL_W, CHAR_PIXEL_H, BLACK);
    // Draw char
    if (charIndex <= messageIndex) {
        drawChar(x, y, message[charIndex], COLORS[appliedColors[charIndex]], BLACK,
                 1);
    }
    DrawCursor();
}

void DrawInput() {
    int i = 0;
    for (i = 0; i < MAX_MSG_LEN; i++) {
        if (message[i] == '\0') {
            return;
        }
        DrawCharacter(i);
    }
    DrawCursor();
}

void DrawReceived() {
    // Display received message on TOP half of OLED
    fillRect(0, 0, 128, 56, BLACK); // upper half
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    setCursor(0, 0);
    char buf[CHAR_PER_LINE] = {0};
    sprintf(buf, "From Device %d:", senderID);
    Outstr(buf);

    int i = 0;
    for (i = 0; i < MAX_MSG_LEN; i++) {
        if (received[i] == '\0') {
            break;
        }

        int row = i / CHAR_PER_LINE;
        int col = i % CHAR_PER_LINE;
        int x = col * CHAR_PIXEL_W;
        int y = row * CHAR_PIXEL_H + CHAR_PIXEL_H;
        // Draw char
        drawChar(x, y, received[i], COLORS[receivedColors[i]], BLACK, 1);
    }
}

// ----- Networking -----
long getJsonLong(const char* json, const char* key) {
    char* pos = strstr(json, key);
    if (!pos) {
        return -1;
    }
    pos = strchr(pos, ':');
    if (!pos) {
        return -1;
    }
    if (!pos || *pos == '\0') {
        return -1;
    }
    pos++;

    char* end = 0;
    long value = strtol(pos, &end, 10);

    if (pos == end) {
        return -1;
    }

    return value;
}

long getJsonString(const char* json, const char* key, char* buffer, long bufLen) {
    char* pos = strstr(json, key);
    if (!pos) {
        return -1;
    }

    pos = strchr(pos, ':');
    if (!pos) {
        return -1;
    }

    pos = strchr(pos, '"');
    if (!pos || *pos == '\0') {
        return -1;
    }
    pos++;

    const char* end = strchr(pos, '"');
    long len = end ? (long) (end - pos)
                   : strlen(pos);
    if (len >= bufLen) {
        strncpy(buffer, pos, bufLen - 1);
        buffer[bufLen - 1] = '\0';
        return 1;
    } else {
        strncpy(buffer, pos, len);
        buffer[len] = '\0';
    }

    return 0;
}

static void httpRequest(const char* request, unsigned long requestLen, char* responseBuffer, unsigned long responseBufLen) {
    int lRetVal = 0;
    char* buf = responseBuffer;

    if (socketID < 0) {
        UART_PRINT("Invalid socketID.\r\n");
        return;
    }

    UART_PRINT(">>>>>>>>>>");
    UART_PRINT(request);
    UART_PRINT("\n\r\n\r");

    lRetVal = sl_Send(socketID, request, requestLen, 0);
    if (lRetVal < 0) {
        UART_PRINT("Send failed. Error Number: %i\n\r", lRetVal);
        return;
    }
    UART_PRINT("Sent %d bytes.\r\n", lRetVal);
    lRetVal = sl_Recv(socketID, buf, responseBufLen, 0);
    if (lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r", lRetVal);
        return;
    } else {
        UART_PRINT("Received %d bytes.\r\n", lRetVal);
        responseBuffer[lRetVal + 1] = '\0';
        UART_PRINT("<<<<<<<<<<");
        UART_PRINT(responseBuffer);
        UART_PRINT("\n\r\n\r");
    }
}

static void pollShadow() {
    char recvBuffer[MAX_BUFFER_LEN] = {0};
    char colorBuffer[MAX_MSG_LEN] = {0};

    httpRequest(GET_REQUEST, strlen(GET_REQUEST), recvBuffer, MAX_BUFFER_LEN);

    const char* metadata = strstr(recvBuffer, "\"metadata\":{");

    if (!metadata) {
        UART_PRINT("Polled device shadow has no metadata.\r\n");
        return;
    }

    long version = getJsonLong(metadata, "\"version\"");
    long timestamp = getJsonLong(metadata, "\"timestamp\"");
    if (version <= lastVersion) {
        if (timestamp <= lastTimestamp) {
            UART_PRINT("Polled device shadow has no new data. Version: %d\r\nTimestamp: %d\r\n", version, timestamp);
            return;
        }
    }
    lastTimestamp = timestamp;
    lastVersion = version;

    const char* desired = strstr(recvBuffer, "\"desired\":{");

    if (!desired) {
        UART_PRINT("Polled device shadow has no desired field.\r\n");
        return;
    }

    long sender = getJsonLong(desired, "deviceid");
    if (sender == USER_ID) {
        return;
    }
    senderID = sender;

    getJsonString(desired, "message", received, MAX_MSG_LEN);
    getJsonString(desired, "colors", colorBuffer, MAX_MSG_LEN);

    int i = 0;
    for (i = 0; colorBuffer[i] != '\0' && i < MAX_MSG_LEN; i++) {
        switch (colorBuffer[i]) {
            case 'W':
                receivedColors[i] = 0;
                break;
            case 'R':
                receivedColors[i] = 1;
                break;
            case 'M':
                receivedColors[i] = 2;
                break;
            case 'Y':
                receivedColors[i] = 3;
                break;
            case 'G':
                receivedColors[i] = 4;
                break;
            case 'B':
                receivedColors[i] = 5;
                break;
            case 'C':
                receivedColors[i] = 6;
                break;
            default:
                receivedColors[i] = 0;
        }
    }

    msgRecvFlag = 1;
    ResetUpdateInterval();

    return;
}

static void sendMessage() {
    char colorBuffer[MAX_MSG_LEN] = {0};
    int i = 0;
    for (i = 0; i < messageIndex && message[i] != '\0'; i++) {
        switch (appliedColors[i]) {
            case 0:
                colorBuffer[i] = 'W';
                break;
            case 1:
                colorBuffer[i] = 'R';
                break;
            case 2:
                colorBuffer[i] = 'M';
                break;
            case 3:
                colorBuffer[i] = 'Y';
                break;
            case 4:
                colorBuffer[i] = 'G';
                break;
            case 5:
                colorBuffer[i] = 'B';
                break;
            case 6:
                colorBuffer[i] = 'C';
                break;
            default:
                colorBuffer[i] = 'W';
        }
    }

    char writeBuffer[MAX_BUFFER_LEN] = {0};
    char postBuffer[MAX_BUFFER_LEN] = {0};
    long jsonLen = 0;
    sprintf(writeBuffer, JSON_TEMPLATE, USER_ID, message, colorBuffer);
    jsonLen = strlen(writeBuffer);
    sprintf(postBuffer, POST_TEMPLATE, jsonLen, writeBuffer);

    httpRequest(postBuffer, strlen(postBuffer), writeBuffer, MAX_BUFFER_LEN);
    return;
}

// ----- CSS Launchpad -----
static void BoardInit(void) {
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

    // Timer Init
    Timer_IF_Init(PRCM_TIMERA0, TIMERIO, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_Init(PRCM_TIMERA1, TIMERAWS, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERIO, TIMER_A, StopwatchHandler);
    Timer_IF_IntSetup(TIMERAWS, TIMER_A, UpdateHandler);
    StartStopwatch();
    ResetUpdateInterval();

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
        (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVEHIGH | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    // OLED display
    Adafruit_Init();
    fillScreen(BLACK);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    setCursor(0, 64);
    char buf[CHAR_PER_LINE] = {0};
    sprintf(buf, "As Device %d", USER_ID);
    Outstr(buf);

    UART_PRINT("Board Initialized\n\r");

    // GPIO Registers
    MAP_GPIOIntRegister(GPIO_GROUP, EdgeHandler);
    MAP_GPIOIntTypeSet(GPIO_GROUP, GPIO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(GPIO_GROUP, MAP_GPIOIntStatus(GPIO_GROUP, false));
    MAP_GPIOIntEnable(GPIO_GROUP, GPIO_PIN);
}

static int set_time() {
    long retVal;

    g_time.tm_year = YEAR;
    g_time.tm_mon = MONTH;
    g_time.tm_day = DATE;
    g_time.tm_hour = HOUR;
    g_time.tm_min = MINUTE;
    g_time.tm_sec = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                       SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime), (unsigned char*) (&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES -- END
//****************************************************************************

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main() {
    long lRetVal = -1;
    BoardInit();

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    // Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    // Set time so that encryption can be used
    lRetVal = set_time();
    if (lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        while (1)
            ;
    }
    // Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if (lRetVal < 0) {
        ERR_PRINT(lRetVal);
        while (1)
            ;
    }

    socketID = lRetVal;

    // Main loop
    while (1) {
        // Character locked in after delay
        if (sysTickValue > REPEAT_KEY_DELAY) {
            prevButton = 0;
            StartStopwatch();
            StopStopwatch();
        }

        if (inputFlag) {
            inputFlag = 0;

            if (DecodeInput()) {
                DrawCharacter(messageIndex - 1);
            }
        }

        if (deleteFlag) {
            deleteFlag = 0;
            DrawCharacter(messageIndex + 1);
        }

        if (msgRecvFlag) {
            msgRecvFlag = 0;
            DrawReceived();
        }

        if (sendFlag) {
            pollShadowFlag = 0;
            pollShadow();

            sendMessage();

            // Clear bottom half of the screen
            fillRect(0, 72, 129, 129, BLACK);

            // Reset message buffer
            message[0] = '\0';
            messageIndex = 0;
            sendFlag = 0;
        }

        if (pollShadowFlag) {
            pollShadowFlag = 0;
            pollShadow();
        }
    }

    // EXIT and CLEANUP
    UART_PRINT("\n\r Exiting the Application\n\r");
    sl_Stop(SL_STOP_TIMEOUT);
    while (1)
        ;
}

