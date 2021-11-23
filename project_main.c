/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[2*STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char taskStack[STACKSIZE];
Char commTaskStack[STACKSIZE];

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

// Definition of the state machine
enum state {BOOTING=1, SHUTTING_DOWN, WAITING, BUTTON_PUSH, POWER_BUTTON_PUSH, FEED, SLEEP, EXERCISE, PET, WARNING, GAME_OVER, SENDING_DATA, NOT_SENDING_DATA};
enum state programState = BOOTING;
enum state petState = WAITING;
enum state dataState = NOT_SENDING_DATA;

//different foods for feeding
char foods[5][11] = {"yogurt", "porridge", "hotdog", "kebabfries", "pizza"};
int petFood = 0;

// Global variable for system time
float systemTime = 0.0;

// Global variables for MPU9250 data
float rawMPUData[7][50];
float cleanMPUData[7][48];
float derivates[6][47];
float averageDerivates[6];

// Pins' RTOS-variables and configuration
static PIN_Handle powerButtonHandle;
static PIN_State powerButtonState;
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle buzzerHandle;
static PIN_State buzzerState;

// Power button
PIN_Config powerButtonConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
   PIN_TERMINATE
};
PIN_Config powerButtonWakeConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
   PIN_TERMINATE
};
float powerButtonWasPushed = 0.0;

// Other button
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
   PIN_TERMINATE
};
float buttonWasPushed = 0.0;

// Red led
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
   PIN_TERMINATE
};

// Buzzer
PIN_Config buzzerConfig[] = {
   Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

// Buzzer sounds. First values are frequencies, second values are notes' lengths and third values are pauses between notes.
float bootingSound[9][3] = {{783.99, 100000, 550000},
                            {659.25, 100000, 550000},
                            {587.33, 100000, 100000},
                            {587.33, 100000, 100000},
                            {659.25, 100000, 100000},
                            {698.46, 100000, 100000},
                            {587.33, 100000, 100000},
                            {698.46, 100000, 100000},
                            {783.99, 100000, 0}};
float feedSound[3][3] = {{2000, 100000, 50000},
                         {3000, 100000, 50000},
                         {4000, 100000, 0}};
float sleepSound[3][3] = {{1000, 200000, 200000},
                          {800, 400000, 200000},
                          {600, 500000, 0}};
float exerciseSound[4][3] = {{5000, 100000, 50000},
                             {2000, 100000, 50000},
                             {5000, 100000, 50000},
                             {2000, 100000, 0}};
float petSound[9][3] = {{3000, 50000, 0},
                        {4000, 50000, 0},
                        {2000, 50000, 100000},
                        {3000, 50000, 0},
                        {4000, 50000, 0},
                        {2000, 50000, 100000},
                        {3000, 50000, 0},
                        {4000, 50000, 0},
                        {2000, 50000, 0}};
float warningSound[3][3] = {{700, 200000, 200000},
                            {700, 200000, 200000},
                            {700, 200000, 0}};
float shutDownSound[4][3] = {{698.46,100000, 50000},
                             {349.23, 110000,50000},
                             {174.61, 120000, 50000},
                             {87.31, 130000,0}};
float gameOverSound[3][3] = {{698.46,150000, 500000},
                             {349.23, 150000, 500000},
                             {174.61, 1000000, 0}};
float powerButtonSound[2][3] = {{1500, 100000, 0},
                                {1000, 100000, 0}};
float buttonSound[2][3] = {{1000, 100000, 0},
                           {1500, 100000, 0}};

// Calculation functions
void movavg(float *array, uint8_t array_size, uint8_t window_size, float *averages);
void calculateDerivates(float *array, uint8_t array_size, float *derivates);
int checkAverageDerivates(float *averageDerivates);
void playBuzzer(float sound[][3], int notes);
void sendMessage(char *payload);


// Power button interruption handler
Void powerFxn(PIN_Handle handle, PIN_Id pinId) {

    // If button is pushed down
    if (!PIN_getInputValue(pinId)) {
        powerButtonWasPushed = systemTime;
    // If button is released
    } else if (PIN_getInputValue(pinId)) {
        // Long push
        if (systemTime >= powerButtonWasPushed + 2) {
            programState = SHUTTING_DOWN;
            System_printf("Long power button push\n");
            System_flush();
            System_printf("Shutting down...\n");
            System_flush();
        // Short push
        } else if (systemTime > 1) {
            System_printf("Short power button push\n");
            System_flush();
            programState = POWER_BUTTON_PUSH;
            if (dataState == NOT_SENDING_DATA) {
                sendMessage("id:0301,session:start\0");
                sendMessage("id:0301,session:start\0");
                sendMessage("id:0301,session:start\0");
                dataState = SENDING_DATA;
            } else {
                sendMessage("id:0301,session:end\0");
                dataState = NOT_SENDING_DATA;
            }

        }
    }
}


// Other button interruption handler
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    /*
    // Blink led
    // If this led is on, the ambient light values stay very high (over 100)
    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
    */

    // If button is pushed down
    if (!PIN_getInputValue(pinId)) {
        buttonWasPushed = systemTime;
    // If button is released
    } else if (PIN_getInputValue(pinId)) {
        // Long push
        if (systemTime >= buttonWasPushed + 2) {
            System_printf("Long button push\n");
            System_flush();
            System_printf("Feeding...\n");
            System_flush();
            petState = FEED;
            char message[80];
            sprintf(message, "id:0301,EAT:%d,MSG1:Eating\0", petFood+1);
            sendMessage(message);
        // Short push
        } else if (systemTime > 1) {
            System_printf("Short button push\n");
            System_flush();
            programState = BUTTON_PUSH;
            petFood++;
            if (petFood == 5) {
                petFood = 0;
            }
            char output[80] = {"id:0301,MSG2:Selected food = "};
            strcat(output, foods[petFood]);
            System_printf(output);
            System_flush();
            sendMessage(output);
        }
    }
}


// Data transfer task
Void commTask(UArg arg0, UArg arg1) {
    char payload[50]; // message buffer
    uint16_t senderAddr;

    // Initialize radio for receiving
    int32_t result = StartReceive6LoWPAN();
    if(result != true) {
      System_abort("Wireless receive start failed");
    }

    // Receive messages in a loop
    while (1) {
        // NOTE: Do not send messages in this loop. It will clog the radio and
        // others will not be able to use the channel!

        // If true, there is a message waiting
        if (GetRXFlag()) {
            // Empty the message buffer
            memset(payload,0,50);
            // Read a message to the message buffer
            Receive6LoWPAN(&senderAddr, payload, 50);
            if (strstr(payload, "301,BEEP:Too late")) {
                System_printf("Game over\n");
                System_flush();
                programState = GAME_OVER;
            } else if (strstr(payload, "301,BEEP")) {
                System_printf(payload);
                System_flush();
                programState = WARNING;
            }
        }
    }
}


// UART task
Void uartTaskFxn(UArg arg0, UArg arg1) {
    char output[80];
    
    // UART-library settings
    UART_Handle uart;
    UART_Params uartParams;
    
    // Alustetaan sarjaliikenne 
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1
    
    // Open the connection to the serial port of the device in the constant Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
      System_abort("Error opening the UART");
    }

    while (1) {
        // Play bootingSound
        if (programState == BOOTING) {
            playBuzzer(bootingSound, 8);
            programState = WAITING;
        }
        // Play shutDownSound
        if (programState == SHUTTING_DOWN) {
            playBuzzer(shutDownSound, 4);
            programState = WAITING;
            sendMessage("id:0301,MSG1:Device turned off\0");
            // Taikamenot
            PIN_close(powerButtonHandle);
            PINCC26XX_setWakeup(powerButtonWakeConfig);
            Power_shutdown(NULL,0);
        }
        // Play warningSound
        if (programState == WARNING) {
            playBuzzer(warningSound, 3);
            programState = WAITING;
        }
        // Play gameOverSound
        if (programState == GAME_OVER) {
            playBuzzer(gameOverSound, 3);
            programState = WAITING;
        }

        if (programState == POWER_BUTTON_PUSH) {
            playBuzzer(powerButtonSound, 2);
            programState = WAITING;
        }

        if (programState == BUTTON_PUSH) {
            playBuzzer(buttonSound, 2);
            programState = WAITING;
        }


        //sprintf(output, "Time: %.0f\n\r", systemTime);
        //UART_write(uart, output, strlen(output)); // Use this to send commands when working from home.

/*
        // Red led turns on/off every second if this is included
        uint_t pinValue = PIN_getOutputValue( Board_LED1 );
        pinValue = !pinValue;
        PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
*/

        // Once per second, you can modify this
        Task_sleep(1000000 / Clock_tickPeriod);
    }
}


// Sensor task
Void sensorTaskFxn(UArg arg0, UArg arg1) {
    // General variables
    char output[80] = {0};
    int i = 0;
    int j = 0;

    // MPU9250 variables
    float ax, ay, az, gx, gy, gz;
	I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
	I2C_Params i2cMPUParams;
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    int MPUindex = 0;

    // OPT3001 variables
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    double OPTdata[10] = {0};
    int earlierTime = 0;
    int OPTindex = 0;
    int isDarkEnough = 0;

    // MPU9250 -SENSOR INITIALIZATION
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }
    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    // Wait 100ms for the MPU sensor to power up
	Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();
    // MPU setup and calibration
	System_printf("MPU9250: Setup and calibration...\n");
	System_flush();
	mpu9250_setup(&i2cMPU);
	System_printf("MPU9250: Setup and calibration OK\n");
	System_flush();
    I2C_close(i2cMPU);


    // OPT3001 -SENSOR INITIALIZATION
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
       System_abort("Error Initializing I2C\n");
    }
    // Setup the OPT3001 sensor for use
    // Before calling the setup function, insert 100ms delay with Task_sleep
    Task_sleep(100000 / Clock_tickPeriod);
    opt3001_setup(&i2c);
    I2C_close(i2c);

    earlierTime = (int)systemTime;

    while (1) {

        // OPT3001 DATA READ
        if ((int)systemTime == earlierTime+1) { // OPT3001 data is read once per second
            earlierTime = (int)systemTime;
            i2c = I2C_open(Board_I2C_TMP, &i2cParams);
            if (i2c == NULL) {
               System_abort("Error Initializing I2C\n");
            }
            // Read sensor data and print it to the Debug window as string
            OPTdata[OPTindex] = opt3001_get_data(&i2c);

            if (dataState == SENDING_DATA) {
                sprintf(output, "id:0301,light:%.2f", OPTdata[OPTindex]);
                sendMessage(output);
            }

            // Check whether it has been dark enough for 5 seconds
            if (OPTindex == 9) {
                for(i = 0; i < 10; i++) {
                    if(OPTdata[i] > 5) {
                        isDarkEnough = 0;
                        break;
                    } else if (i == 9){
                        isDarkEnough = 1;
                    }
                }
                if (isDarkEnough) {
                    if (petState != PET) {
                        System_printf("Sleeping...\n");
                        System_flush();
                        sendMessage("id:0301,ACTIVATE:1;1;2,MSG1:ZZZ\0");
                        petState = SLEEP;
                    }
                    memset(OPTdata, 0, 10);
                    OPTindex = 0;
                } else {
                    for(i = 0; i < 9; i++) {
                        OPTdata[i] = OPTdata[i+1];
                    }
                    petState = WAITING;
                }
            } else {
                OPTindex++;
            }

            I2C_close(i2c);
        }

        // MPU9250 DATA READ
        i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
        if (i2cMPU == NULL) {
            System_abort("Error Initializing I2CMPU\n");
        }

        // Get data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

        if (dataState == SENDING_DATA) {
            sprintf(output, "id:0301,ax:%.2f,ay:%.2f,az:%.2f,gx:%.2f,gy:%.2f,gz:%.2f\0", ax, ay, az, gx, gy, gz);
            sendMessage(output);
        }

        // Raw data into an array
        rawMPUData[0][MPUindex] = systemTime;
        rawMPUData[1][MPUindex] = ax;
        rawMPUData[2][MPUindex] = ay;
        rawMPUData[3][MPUindex] = az;
        rawMPUData[4][MPUindex] = gx;
        rawMPUData[5][MPUindex] = gy;
        rawMPUData[6][MPUindex] = gz;

        // If the rawMPUData array is full, do some calculations
        if (MPUindex == 49) {

            // Moving averages
            for (i = 0; i < 7; i++) {
                movavg(rawMPUData[i], 50, 3, cleanMPUData[i]);
            }

            // Derivates
            for (i = 1; i < 7; i++) {
                calculateDerivates(cleanMPUData[i], 48, derivates[i-1]);
            }

            // Average derivates
            for (i = 0; i < 6; i++) {
                movavg(derivates[i], 47, 47, &averageDerivates[i]);
            }

            // If an average derivate was big enough, 'restart' data collection
            if (checkAverageDerivates(averageDerivates)) {
                MPUindex = -1;
            } else {
                // If no average derivate was big enough, shift rawMPUData values left and continue data collection
                for (i = 0; i < 7; i++) {
                    for (j = 0; j < 49; j++) {
                        rawMPUData[i][j] = rawMPUData[i][j+1];
                    }
                }
            }

        }

        if (MPUindex < 49) {
            MPUindex++;
        }

        I2C_close(i2cMPU);


        // Play sounds
        if (petState == FEED)
            playBuzzer(feedSound, 3);
        else if (petState == SLEEP)
            playBuzzer(sleepSound, 3);
        else if (petState == EXERCISE)
            playBuzzer(exerciseSound, 4);
        else if (petState == PET)
            playBuzzer(petSound, 9);
        petState = WAITING;

        // Once per 100ms, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}


/* Used for calculating moving average for an array of float numbers.
 * Parameters:
 * - float *array: Original array.
 * - uint8_t array_size: Size of the original array.
 * - uint8_t window_size: How many numbers' average is calculated.
 * - float *averages: The array were the averaged values are stored.
 *                    Note that the size of this array should be at least
 *                    array_size - window_size + 1.
*/
void movavg(float *array, uint8_t array_size, uint8_t window_size, float *averages) {
    float temp = 0;
    int i = 0;
    int j = 0;

    for(i = 0; i <= array_size - window_size; i++){
        for(j = 0, temp = 0; j < window_size; j++) {
            temp += array[i+j];
        }
        averages[i] = temp / window_size;
    }
}


/* Used for calculating the difference between each two consecutive
 * values in an array of floats, a.k.a. derivates.
 * Parameters:
 * - float *array: Original array.
 * - uint8_t array_size: Size of the original array.
 * - float *derivates: Array where the derivates are stored.
 *                     Note that the size of this array must be
 *                     at least array_size - 1.
 *
*/
void calculateDerivates(float *array, uint8_t array_size, float *derivates) {
    int i = 0;

    for(i = 0; i <= array_size - 2; i++){
        derivates[i] = fabs(array[i+1] - array[i]) / 0.1;
    }
}


/* If any average derivate is big enough, this prints which one of the
 * axes it was and how big was the average derivate value.
 * Parameters:
 * - float *averageDerivates: Array containing the average derivate values.
 * Returns:
 * - 1 if any of the average derivates is big enough, 0 otherwise.
 */
int checkAverageDerivates(float *averageDerivates) {
    char output[60];
    if (averageDerivates[0] > 2) {
        sprintf(output, "X-axis movement (average derivate: %.2f)\n", averageDerivates[0]);
        System_printf(output);
        System_flush();
    }
    if (averageDerivates[1] > 2) {
        sprintf(output, "Y-axis movement (average derivate: %.2f)\n", averageDerivates[1]);
        System_printf(output);
        System_flush();
    }
    if (averageDerivates[2] > 3) {
        sprintf(output, "Z-axis movement (average derivate: %.2f)\n", averageDerivates[2]);
        System_printf(output);
        System_flush();
        petState = EXERCISE;
        System_printf("Exercising...\n");
        System_flush();
        sendMessage("id:0301,EXERCISE:4,MSG1:Exercising\0");
    }
    if ((averageDerivates[0] > 2 || averageDerivates[1] > 2) && averageDerivates[2] < 1) {
        petState = PET;
        System_printf("Being pet...\n");
        System_flush();
        sendMessage("id:0301,PET:3,MSG1:Being pet\0");
    }
    if (averageDerivates[0] > 2 || averageDerivates[1] > 2 || averageDerivates[2] > 3) {
        return 1;
    } else {
        return 0;
    }
}


/* Function that plays all the buzzer sounds.
 * Parameters:
 * - float sound[][3]: Array containing the frequencies, note lengths and pauses between notes.
 * - int notes: The amount of notes in a sound, a.k.a. the number of rows in the sound-array.
 */
void playBuzzer(float sound[][3], int notes) {
    int i = 0;

    if (petState != SLEEP){
        PIN_setOutputValue( ledHandle, Board_LED1, 1 );
    }

    for (i = 0; i < notes; i++) {
        buzzerOpen(buzzerHandle);
        buzzerSetFrequency(sound[i][0]);
        Task_sleep(sound[i][1] / Clock_tickPeriod);
        buzzerClose();
        Task_sleep(sound[i][2] / Clock_tickPeriod);
    }

    PIN_setOutputValue( ledHandle, Board_LED1, 0 );
}


// Sends messages to the gateway.
void sendMessage(char *payload) {
    uint16_t DestAddr = 0x1234;
    Send6LoWPAN(DestAddr, payload, strlen(payload));
    // Note! Radio must always be restored to the receiving state.
    // Note2! Do not check failure, only check failure when initializing (in commTask).
    StartReceive6LoWPAN();
}


// Clock function
Void clkFxn(UArg arg0) {
   systemTime = (float)Clock_getTicks() / 100000.0;
}


Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle commTaskHandle;
    Task_Params commTaskParams;

    // Initialize board
    Board_initGeneral();
    Init6LoWPAN();
    Board_initI2C();
    Board_initUART();
    
    // CLOCK INITIALIZATION
    // RTOS's clock variables
    Clock_Handle clkHandle;
    Clock_Params clkParams;
    // Init clock
    Clock_Params_init(&clkParams);
    clkParams.period = 1000000 / Clock_tickPeriod;
    clkParams.startFlag = TRUE;
    // Start using clock
    clkHandle = Clock_create((Clock_FuncPtr)clkFxn, 1000000 / Clock_tickPeriod, &clkParams, NULL);
    if (clkHandle == NULL) {
      System_abort("Clock create failed");
    }

    // Open the button and led pins
    powerButtonHandle = PIN_open(&powerButtonState, powerButtonConfig);
    if (!powerButtonHandle) {
       System_abort("Error initializing power button\n");
    }
    if (PIN_registerIntCb(powerButtonHandle, &powerFxn) != 0) {
       System_abort("Error registering power button callback");
    }

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if (!buttonHandle) {
      System_abort("Error initializing button pins\n");
    }
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
      System_abort("Error registering button callback function");
    }

    ledHandle = PIN_open(&ledState, ledConfig);
    if (!ledHandle) {
      System_abort("Error initializing LED pins\n");
    }

    // Open buzzer pin
    buzzerHandle = PIN_open(&buzzerState, buzzerConfig);
    if (buzzerHandle == NULL) {
        System_abort("Error initializing buzzer pin\n");
    }

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
    	System_abort("Pin open failed!");
    }
    
    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    
    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&commTaskParams);
    commTaskParams.stackSize = STACKSIZE;
    commTaskParams.stack = &commTaskStack;
    commTaskParams.priority=1;
    commTaskHandle = Task_create(commTask, &commTaskParams, NULL);
    if (commTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
    
    /* Start BIOS */
    BIOS_start();
    
    return (0);
}
