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

// Global variable for MPU9250 data
float finalDataTable[7][50];

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
    // JTKJ: Exercise 1. Blink either led of the device
    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
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

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    // JTKJ: Exercise 4. Setup here UART connection as 9600,8n1
    char input;
    char output[80];
    int m = 0;
    
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
        // JTKJ: Exercise 3. Print out sensor data as string to debug window if the state is correct
        //       Remember to modify state
        if(programState == DATA_READY) {
            sprintf(output, "uartTask: %f\n", ambientLight);
            //System_printf(output);
            //System_flush();
            programState = WAITING;
        }
/*
        if (programState == SHOW_RESULTS) {
            for(m = 0; m < 50; m++) {
                sprintf(output, "%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", finalDataTable[0][m], finalDataTable[1][m], finalDataTable[2][m], finalDataTable[3][m], finalDataTable[4][m], finalDataTable[5][m], finalDataTable[6][m]);
                finalDataTable[0][m] = 0;
                finalDataTable[1][m] = 0;
                finalDataTable[2][m] = 0;
                finalDataTable[3][m] = 0;
                finalDataTable[4][m] = 0;
                finalDataTable[5][m] = 0;
                finalDataTable[6][m] = 0;
                UART_write(uart, output, strlen(output));
            }
            programState = WAITING;
        }
*/
        // JTKJ: Exercise 4. Send the same sensor data string with UART

        //UART_write(uart, output, strlen(output));

        // Just for sanity check for exercise, you can comment this out
        // System_printf("uartTask\n");
        // System_flush();
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
    // MPU9250 variables
    float ax, ay, az, gx, gy, gz;
	char printableData[80] = {0};
	I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
	I2C_Params i2cMPUParams;
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    float rawMPUData[6][3];
    float cleanMPUData[6];
    int m = 0;
    float sum = 0.0;
    int i = 0;
    int j = 0;
    int k = 0;
    int l = 0;
    int isTableFull = 0;
    // OPT3001 variables
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cMessage;
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

    // JTKJ: Exercise 2. Setup the OPT3001 sensor for use
    //       Before calling the setup function, insert 100ms delay with Task_sleep
    Task_sleep(100000 / Clock_tickPeriod);
    opt3001_setup(&i2c);
    
    I2C_close(i2c);

    earlierTime = (int)systemTime;

    while (1) {

        // OPT3001 DATA READ
        /*
        if ((int)systemTime == earlierTime+1) { // OPT3001 data is read once per second
            earlierTime = (int)systemTime;
            i2c = I2C_open(Board_I2C_TMP, &i2cParams);
            if (i2c == NULL) {
               System_abort("Error Initializing I2C\n");
            }
            // JTKJ: Exercise 2. Read sensor data and print it to the Debug window as string
            data[n] = opt3001_get_data(&i2c);
            char merkkijono[20];
            sprintf(merkkijono, "OPT3001: %f\n", data[n]);
            System_printf(merkkijono);
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

            // JTKJ: Exercise 3. Save the sensor value into the global variable
            //       Remember to modify state
            ambientLight = data[n];
            programState = DATA_READY;

            I2C_close(i2c);
        }
        */

        // MPU9250 DATA READ
        if (programState == COLLECTING_DATA) {
            i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
            if (i2cMPU == NULL) {
                System_abort("Error Initializing I2CMPU\n");
            }

            // Get data
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

            // Average data
            rawMPUData[0][i] = ax;
            rawMPUData[1][i] = ay;
            rawMPUData[2][i] = az;
            rawMPUData[3][i] = gx;
            rawMPUData[4][i] = gy;
            rawMPUData[5][i] = gz;
            if (i == 2) {
                for(j = 0; j < 6; j++) {
                    for(k = 0; k < 3; k++) {
                        sum += rawMPUData[j][k];
                    }
                    cleanMPUData[j] = sum / 3;
                    sum = 0;
                }
                for(l = 0; l < 6; l++) {
                    rawMPUData[l][0] = rawMPUData[l][1];
                    rawMPUData[l][1] = rawMPUData[l][2];
                }
            } else {
                i++;
            }

            // Collects data continuously, when table is full, values are shifted left and the last value is placed to the end of the table
            if (isTableFull) {
                for(m = 0; m < 49; m++) {
                    finalDataTable[0][m] = finalDataTable[0][m+1];
                    finalDataTable[1][m] = finalDataTable[1][m+1];
                    finalDataTable[2][m] = finalDataTable[2][m+1];
                    finalDataTable[3][m] = finalDataTable[3][m+1];
                    finalDataTable[4][m] = finalDataTable[4][m+1];
                    finalDataTable[5][m] = finalDataTable[5][m+1];
                    finalDataTable[6][m] = finalDataTable[6][m+1];
                }
            }

            finalDataTable[0][m] = systemTime;
            finalDataTable[1][m] = cleanMPUData[0];
            finalDataTable[2][m] = cleanMPUData[1];
            finalDataTable[3][m] = cleanMPUData[2];
            finalDataTable[4][m] = cleanMPUData[3];
            finalDataTable[5][m] = cleanMPUData[4];
            finalDataTable[6][m] = cleanMPUData[5];

            // TODO: implement average derivate calculations

            /*
            sprintf(printableData, "Raw data:\t%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", systemTime, ax, ay, az, gx, gy, gz);
            System_printf(printableData);
            System_flush();
            */
            /*
            sprintf(printableData, "Averaged data:\t%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", systemTime, cleanMPUData[0], cleanMPUData[1], cleanMPUData[2], cleanMPUData[3], cleanMPUData[4], cleanMPUData[5]);
            System_printf(printableData);
            System_flush();
            */
            I2C_close(i2cMPU);

            /*
            if (m == 49) {
                programState = SHOW_RESULTS;
                for(m = 0; m < 50; m++) {
                    sprintf(printableData, "%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", finalDataTable[0][m], finalDataTable[1][m], finalDataTable[2][m], finalDataTable[3][m], finalDataTable[4][m], finalDataTable[5][m], finalDataTable[6][m]);
                    finalDataTable[0][m] = 0;
                    finalDataTable[1][m] = 0;
                    finalDataTable[2][m] = 0;
                    finalDataTable[3][m] = 0;
                    finalDataTable[4][m] = 0;
                    finalDataTable[5][m] = 0;
                    finalDataTable[6][m] = 0;
                    System_printf(printableData);
                    System_flush();
                }
                programState = WAITING;
                m = 0;
            }
            */

            if (m < 49) {
                m++;
            } else {
                isTableFull = 1;
            }

        }

        if (programState == SHOW_RESULTS) {
            for(m = 0; m < 50; m++) {
                sprintf(printableData, "%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", finalDataTable[0][m], finalDataTable[1][m], finalDataTable[2][m], finalDataTable[3][m], finalDataTable[4][m], finalDataTable[5][m], finalDataTable[6][m]);
                finalDataTable[0][m] = 0;
                finalDataTable[1][m] = 0;
                finalDataTable[2][m] = 0;
                finalDataTable[3][m] = 0;
                finalDataTable[4][m] = 0;
                finalDataTable[5][m] = 0;
                finalDataTable[6][m] = 0;
                System_printf(printableData);
                System_flush();
            }
            programState = WAITING;
            m = 0;
            isTableFull = 0;
        }

        // Once per 100ms, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
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

    // Initialize board
    Board_initGeneral();
    Init6LoWPAN();
    
    // JTKJ: Exercise 2. Initialize i2c bus
    Board_initI2C();
    // JTKJ: Exercise 4. Initialize UART
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

    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button
    // Otetaan pinnit käyttöön ohjelmassa
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

    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
      System_abort("Error initializing LED pins\n");
    }

    // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
    // funktio buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
      System_abort("Error registering button callback function");
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
    
    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
    
    /* Start BIOS */
    BIOS_start();
    
    return (0);
}
