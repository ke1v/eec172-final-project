//// ----- Stopwatch -----
//int inRange(int value, int target, int range) {
//    return value >= target - range && value <= target + range;
//}
//
//void StartStopwatch() {
//    sysTickValue = 0;
//    stopwatchFlag = 1;
//    Timer_IF_Start(TIMERIO, TIMER_A, 1);
//}
//
//void StopStopwatch() {
//    stopwatchFlag = 0;
//    Timer_IF_Stop(TIMERIO, TIMER_A);
//}
//
//void StopwatchHandler(void) {
//    //
//    // Clear the timer interrupt.
//    //
//    Timer_IF_InterruptClear(TIMERIO);
//
//    if (stopwatchFlag) {
//        sysTickValue++;
//    }
//
//    if (sysTickValue > IO_TIMEOUT) {
//        StopStopwatch();
//    }
//}
//
//void ResetUpdateInterval() {
//    Timer_IF_Start(TIMERAWS, TIMER_A, AWS_INTERVAL);
//}
//
//void UpdateHandler(void) {
//    //
//    // Clear the timer interrupt.
//    //
//    Timer_IF_InterruptClear(TIMERAWS);
//
//    pollShadowFlag = 1;
//}
//
//// ----- IR Remote -----
//void EdgeHandler(void) {
//    MAP_GPIOIntClear(GPIO_GROUP, MAP_GPIOIntStatus(GPIO_GROUP, false));
//
//    if (MAP_GPIOPinRead(GPIO_GROUP, GPIO_PIN)) {
//        StartStopwatch();
//    } else {
//        StopStopwatch();
//
//        if (sysTickValue <= 0) {
//            return;
//        }
//
//        if (signalFlag) {
//            if (inRange(sysTickValue, ZERO_WIDTH, ACCEPTABLE_JITTER)) {
//                // Zero
//                readBuffer = (readBuffer << 1);
//                pulseWidth[numReadBits] = sysTickValue;
//                numReadBits++;
//            } else if (inRange(sysTickValue, ONE_WIDTH, ACCEPTABLE_JITTER)) {
//                // One
//                readBuffer = (readBuffer << 1) + 1;
//                pulseWidth[numReadBits] = sysTickValue;
//                numReadBits++;
//            }
//            if (numReadBits >= 32) {
//                signalFlag = 0;
//                inputFlag = 1;
//                numReadBits = 0;
//            }
//        } else if (inRange(sysTickValue, START_WIDTH, ACCEPTABLE_JITTER)) {
//            signalFlag = 1;
//        }
//    }
//}
//
//int DecodeInput(void) {
//    if (readBuffer == ONE) {
//        colorIndex++;
//        colorIndex %= NUM_COLORS;
//        return 1;
//    }
//
//    // Repeat button press
//    if (readBuffer == prevButton) {
//        buttonIndex++;
//        messageIndex--;
//    } else {
//        buttonIndex = 0;
//    }
//
//    if (messageIndex <= 0) {
//        messageIndex = 0;
//    }
//
//    prevButton = readBuffer; // for multipress
//    appliedColors[messageIndex] = colorIndex;
//
//    if (readBuffer == ZERO) {
//        message[messageIndex] = ' ';
//        prevButton = 0;
//    } else if (readBuffer == TWO) {
//        message[messageIndex] = BUTTON2[buttonIndex % 3];
//    } else if (readBuffer == THREE) {
//        message[messageIndex] = BUTTON3[buttonIndex % 3];
//    } else if (readBuffer == FOUR) {
//        message[messageIndex] = BUTTON4[buttonIndex % 3];
//    } else if (readBuffer == FIVE) {
//        message[messageIndex] = BUTTON5[buttonIndex % 3];
//    } else if (readBuffer == SIX) {
//        message[messageIndex] = BUTTON6[buttonIndex % 3];
//    } else if (readBuffer == SEVEN) {
//        message[messageIndex] = BUTTON7[buttonIndex % 4];
//    } else if (readBuffer == EIGHT) {
//        message[messageIndex] = BUTTON8[buttonIndex % 3];
//    } else if (readBuffer == NINE) {
//        message[messageIndex] = BUTTON9[buttonIndex % 4];
//    } else if (readBuffer == LAST) {
//        deleteFlag = 1;
//        prevButton = 0;
//        message[messageIndex] = '\0';
//        if (messageIndex > 0) {
//            messageIndex--;
//        }
//        return 1;
//    } else if (readBuffer == MUTE) {
//        message[messageIndex] = '\0';
//        prevButton = 0;
//        sendFlag = 1;
//        return 1;
//    } else {
//        return 0; // Ignore non-mapped buttons
//    }
//
//    if (messageIndex >= MAX_MSG_LEN - 1) {
//        message[messageIndex] = '\0';
//    } else {
//        messageIndex++;
//        StartStopwatch();
//    }
//
//    return 1;
//}