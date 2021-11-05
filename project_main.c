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

// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, DATA_READY, COLLECTING_DATA, SHOW_RESULTS };
enum state programState = WAITING;

// JTKJ: Exercise 3. Global variable for ambient light
double ambientLight = -1000.0;

// Global variable for system time
float systemTime = 0.0;

// Global variables for MPU9250 data
float rawMPUData[7][50];
float cleanMPUData[7][48];
float derivates[6][47];
float averageDerivates[6];

// JTKJ: Exercise 1. Add pins RTOS-variables and configuration here
static PIN_Handle powerButtonHandle;
static PIN_State powerButtonState;
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

// POWER BUTTON
PIN_Config powerButtonConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config powerButtonWakeConfig[] = {
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
   PIN_TERMINATE
};

// OTHER BUTTON
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, 
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

void movavg(float *array, uint8_t array_size, uint8_t window_size, float *averages);
void calculateDerivates(float *array, uint8_t array_size, float *derivates);

Void powerFxn(PIN_Handle handle, PIN_Id pinId) {

    // Turn off screen:
    // Display_clear(displayHandle);
    // Display_close(displayHandle);

    System_printf("Power button was pressed.");
    System_flush();

    // Sleep for 100ms...
    Task_sleep(100000 / Clock_tickPeriod);

    // Taikamenot
    PIN_close(powerButtonHandle);
    PINCC26XX_setWakeup(powerButtonWakeConfig);
    Power_shutdown(NULL,0);
}

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    /*
    // Wireless communication testing
    uint16_t DestAddr = 0x1234;
    char payload[16] = "ping";
    Send6LoWPAN(DestAddr, payload, strlen(payload));

    // Hox! Radio aina takaisin vastaanottotilaan ao. funktiokutssulla
    // Hox2! T�ss� ei en�� tarkisteta paluuarvoa.. tarkistus vain alustuksessa.
    StartReceive6LoWPAN();
    */

    /*
    // Blink led
    // If this led is on, the ambient light values stay very high (over 100)
    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
    */

    // Change program state
    System_printf("Button was pressed\n");
    System_flush();
    if (programState == WAITING) {
        programState = COLLECTING_DATA;
        System_printf("programState: COLLECTING_DATA\n");
        System_flush();
    } else if (programState == COLLECTING_DATA) {
        programState = SHOW_RESULTS;
        System_printf("programState: SHOW_RESULTS\n");
        System_flush();
    }
}

// Data transfer task
Void commTask(UArg arg0, UArg arg1) {
    char payload[16]; // message buffer
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

        // jos true, viesti odottaa
        if (GetRXFlag()) {
            // Empty the message buffer
            memset(payload,0,16);
            // Read a message to the message buffer
            Receive6LoWPAN(&senderAddr, payload, 16);
            // Print the received message to the console window
            System_printf(payload);
            System_flush();
        }
    }
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    // Setup here UART connection as 9600,8n1
    char output[80];
    
    // UART-kirjaston asetukset
    UART_Handle uart;
    UART_Params uartParams;
    
    // Alustetaan sarjaliikenne 
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 57600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1
    
    // Avataan yhteys laitteen sarjaporttiin vakiossa Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
      System_abort("Error opening the UART");
    }

    while (1) {
        // Print out sensor data as string to debug window if the state is correct
        if(programState == DATA_READY) {
            sprintf(output, "uartTask: %f\n", ambientLight);
            //System_printf(output);
            //System_flush();
            programState = WAITING;
        }

        sprintf(output, "Time: %.0f\n\r", systemTime);
        UART_write(uart, output, strlen(output));

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

Void sensorTaskFxn(UArg arg0, UArg arg1) {
    // General variables
    char output[80] = {0};

    // MPU9250 variables
    float ax, ay, az, gx, gy, gz;
	I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
	I2C_Params i2cMPUParams;
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    int nextValueIndex = 0;
    int i = 0;
    int j = 0;

    // OPT3001 variables
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    double data[5] = {30, 30, 30, 30, 30};
    int earlierTime = 0;
    int n = 0;
    int o = 0;
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
            data[n] = opt3001_get_data(&i2c);
            sprintf(output, "OPT3001: %f\n", data[n]);
            System_printf(output);
            System_flush();

            // Check whether it has been dark enough for 5 seconds
            if (n == 4) {
                for(o = 0; o < 5; o++) {
                    if(data[o] > 5) {
                        isDarkEnough = 0;
                        break;
                    } else if (o == 4){
                        isDarkEnough = 1;
                    }
                }
                if (isDarkEnough) {
                    System_printf("Sleeping... ");
                    System_flush();
                }
                for(o = 0; o < 4; o++) {
                    data[o] = data[o+1];
                }
            } else {
                n++;
            }

            // Save the sensor value into the global variable
            ambientLight = data[n];
            //programState = DATA_READY;

            I2C_close(i2c);
        }


        // MPU9250 DATA READ
        if (programState == COLLECTING_DATA) {
            i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
            if (i2cMPU == NULL) {
                System_abort("Error Initializing I2CMPU\n");
            }

            // Get data
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

            // TODO: USE THE FUNCTION IN liukuva_keskiarvo.c TO CLEAN THE DATA INSTEAD OF THIS MESS
            /* 1. Raaka data rawMPUData[6][50] taulukkoon
             * 2. Kun taulukko t�ynn�, tee keskiarvolaskut eli
             * movavg(rawMPUData[0], 50, 3, cleanMPUData[0]);
             * movavg(rawMPUData[1], 50, 3, cleanMPUData[1]); jne.
             * - Huomioitava, ett� t�ytyy olla cleanMPUData[6][48], kun window_size on 3.
             * - Yleistettyn�: cleanMPUData taulukon koko t�ytyy olla [6][rawMPUData-taulukon koko - window_size + 1]
             * 3. Lasketaan derivaatat
             * - T�t� varten voi kehitell� samalla periaatteella toimivan funktion kuin movavg.
             * - Funktiossa siis window_size olisi aina 2 ja keskiarvojen sijasta siell� laskettaisiin aina windowin derivaatta.
             * 4. Lasketaan derivaattojen keskiarvot
             * 5. siirret��n rawMPUData-taulukon alkoita vasemmalle.
             * 6. Otetaan uudet raa'at data arvot rawMPUData-taulukon loppuun. (vaihe 1)
             * 7. Siirryt��n vaiheeseen 2.
             *
             *
             *
             */

            // Raw data into an array
            rawMPUData[0][nextValueIndex] = systemTime;
            rawMPUData[1][nextValueIndex] = ax;
            rawMPUData[2][nextValueIndex] = ay;
            rawMPUData[3][nextValueIndex] = az;
            rawMPUData[4][nextValueIndex] = gx;
            rawMPUData[5][nextValueIndex] = gy;
            rawMPUData[6][nextValueIndex] = gz;

            // If the rawMPUData array is full, do some calculations
            if (nextValueIndex == 49) {

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

                // Shift rawMPUData values left
                for (i = 0; i < 7; i++) {
                    for (j = 0; j < 49; j++) {
                        rawMPUData[i][j] = rawMPUData[i][j+1];
                    }
                }
            }

            if (nextValueIndex < 49) {
                nextValueIndex++;
            }

            I2C_close(i2cMPU);
        }

        // Print the data collected from the last 5 seconds
        if (programState == SHOW_RESULTS && nextValueIndex == 49) {
            System_printf("rawMPUData:\n");
            System_flush();
            for (i = 0; i < 50; i++) {
                sprintf(output, "%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", rawMPUData[0][i], rawMPUData[1][i], rawMPUData[2][i], rawMPUData[3][i], rawMPUData[4][i], rawMPUData[5][i], rawMPUData[6][i]);
                System_printf(output);
                System_flush();
            }
            System_printf("cleanMPUData:\n");
            System_flush();
            for (i = 0; i < 48; i++) {
                sprintf(output, "%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", cleanMPUData[0][i], cleanMPUData[1][i], cleanMPUData[2][i], cleanMPUData[3][i], cleanMPUData[4][i], cleanMPUData[5][i], cleanMPUData[6][i]);
                System_printf(output);
                System_flush();
            }
            System_printf("derivates:\n");
            System_flush();
            for (i = 0; i < 47; i++) {
                sprintf(output, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", derivates[0][i], derivates[1][i], derivates[2][i], derivates[3][i], derivates[4][i], derivates[5][i]);
                System_printf(output);
                System_flush();
            }
            System_printf("averageDerivates:\n");
            System_flush();
            sprintf(output, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", averageDerivates[0], averageDerivates[1], averageDerivates[2], averageDerivates[3], averageDerivates[4], averageDerivates[5]);
            System_printf(output);
            System_flush();
            programState = WAITING;
            nextValueIndex = 0;
        }

        // Prevent data printing if the button was pressed before enough data was gathered
        if (programState == SHOW_RESULTS && nextValueIndex < 49) {
            System_printf("Cannot show results yet, not enough data\n");
            System_flush();
            programState = COLLECTING_DATA;
        }

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
 *                    array_size - window_size + 1
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
 *                     at least array_size - 1
*/
void calculateDerivates(float *array, uint8_t array_size, float *derivates) {
    int i = 0;

    for(i = 0; i <= array_size - 2; i++){
        derivates[i] = fabs(array[i+1] - array[i]) / 0.1;
    }
}

// Kellokeskeytyksen käsittelijä
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
    if(!powerButtonHandle) {
       System_abort("Error initializing power button\n");
    }
    if (PIN_registerIntCb(powerButtonHandle, &powerFxn) != 0) {
       System_abort("Error registering power button callback");
    }

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
      System_abort("Error initializing button pins\n");
    }
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
      System_abort("Error registering button callback function");
    }

    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
      System_abort("Error initializing LED pins\n");
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
