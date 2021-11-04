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
float finalDataTable[7][50];
float derivates[6][49];
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
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tÃ¤llÃ¤ vakiolla
};

PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tÃ¤llÃ¤ vakiolla
};

void derivateCalculations(int m);
// TODO: in the derivateCalculations function, we should pause the prints based on the systemTime -variable, rather than this one below
int derivateIndex = 0; // An index variable used to make sure the derivateCalculations -function does not print too often

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
    // Hox2! Tï¿½ssï¿½ ei enï¿½ï¿½ tarkisteta paluuarvoa.. tarkistus vain alustuksessa.
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

// Tiedonsiirtotaski
Void commTask(UArg arg0, UArg arg1) {
    char payload[16]; // viestipuskuri
    uint16_t senderAddr;

    // Radio alustetaan vastaanottotilaan
    int32_t result = StartReceive6LoWPAN();
    if(result != true) {
      System_abort("Wireless receive start failed");
    }

    // Vastaanotetaan viestejï¿½ loopissa
    while (1) {
        // HUOM! VIESTEJï¿½ EI SAA Lï¿½HETTï¿½ï¿½ Tï¿½SSï¿½ SILMUKASSA
        // Viestejï¿½ lï¿½htee niin usein, ettï¿½ se tukkii laitteen radion ja
        // kanavan kaikilta muilta samassa harjoituksissa olevilta!!

        // jos true, viesti odottaa
        if (GetRXFlag()) {
            // Tyhjennetï¿½ï¿½n puskuri (ettei sinne jï¿½ï¿½nyt edellisen viestin jï¿½miï¿½)
            memset(payload,0,16);
            // Luetaan viesti puskuriin payload
            Receive6LoWPAN(&senderAddr, payload, 16);
            // Tulostetaan vastaanotettu viesti konsoli-ikkunaan
            System_printf(payload);
            System_flush();
        }
    }
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    // Setup here UART connection as 9600,8n1
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
             * 2. Kun taulukko täynnä, tee keskiarvolaskut eli
             * movavg(rawMPUData[0], 50, 3, cleanMPUData[0]);
             * movavg(rawMPUData[1], 50, 3, cleanMPUData[1]); jne.
             * - Huomioitava, että täytyy olla cleanMPUData[6][48], kun window_size on 3.
             * - Yleistettynä: cleanMPUData taulukon koko täytyy olla [6][rawMPUData-taulukon koko - window_size + 1]
             * 3. Lasketaan derivaatat
             * - Tätä varten voi kehitellä samalla periaatteella toimivan funktion kuin movavg.
             * - Funktiossa siis window_size olisi aina 2 ja keskiarvojen sijasta siellä laskettaisiin aina windowin derivaatta.
             * 4. Lasketaan derivaattojen keskiarvot
             * 5. siirretään rawMPUData-taulukon alkoita vasemmalle.
             * 6. Otetaan uudet raa'at data arvot rawMPUData-taulukon loppuun.
             * 7. Siirrytään vaiheeseen 2.
             *
             *
             *
             */


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

            // Derivate calculations
            derivateCalculations(m);

            I2C_close(i2cMPU);

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

/* This function calculates the speed at which the MPU-sensor's values change,
 * a.k.a. it calculates derivates between two most recent values. This function should be
 * called everytime a value is added to the finalDataTable.
 *      Then, if the table is full, the function calculates the average derivate for each
 * axis. Practically this means that the function calculates what the average derivate
 * was during the last 5 seconds. If the average is over 3, the function prints a debug
 * string indicating which axis it was.
 *      For this function to work, you will need these global variables (for now):
 * - float finalDataTable[7][50];
 * - float derivates[6][49];
 * - float averageDerivates[6];
 * - int derivateIndex = 0;
 *
 */
void derivateCalculations(int m) {
    char output[80];

    // calculate derivates
    if (m > 0) {
        derivates[0][m-1] = fabs(finalDataTable[1][m] - finalDataTable[1][m-1]) / 0.1;
        derivates[1][m-1] = fabs(finalDataTable[2][m] - finalDataTable[2][m-1]) / 0.1;
        derivates[2][m-1] = fabs(finalDataTable[3][m] - finalDataTable[3][m-1]) / 0.1;
        derivates[3][m-1] = fabs(finalDataTable[4][m] - finalDataTable[4][m-1]) / 0.1;
        derivates[4][m-1] = fabs(finalDataTable[5][m] - finalDataTable[5][m-1]) / 0.1;
        derivates[5][m-1] = fabs(finalDataTable[6][m] - finalDataTable[6][m-1]) / 0.1;
    }

    // if table is full, calculate average derivates
    if (m == 49 && derivateIndex == 49) {
        // Calculate sums of the derivate values
        for(m = 0; m < 49; m++) {
            averageDerivates[0] += derivates[0][m];
            averageDerivates[1] += derivates[1][m];
            averageDerivates[2] += derivates[2][m];
            averageDerivates[3] += derivates[3][m];
            averageDerivates[4] += derivates[4][m];
            averageDerivates[5] += derivates[5][m];
        }
        // Shift values left to make room for the next value
        for(m = 0; m < 48; m++) {
            derivates[0][m] = derivates[0][m+1];
            derivates[1][m] = derivates[1][m+1];
            derivates[2][m] = derivates[2][m+1];
            derivates[3][m] = derivates[3][m+1];
            derivates[4][m] = derivates[4][m+1];
            derivates[5][m] = derivates[5][m+1];
        }
        // Calculate average for each axis' derivates
        averageDerivates[0] = averageDerivates[0] / 49;
        averageDerivates[1] = averageDerivates[1] / 49;
        averageDerivates[2] = averageDerivates[2] / 49;
        averageDerivates[3] = averageDerivates[3] / 49;
        averageDerivates[4] = averageDerivates[4] / 49;
        averageDerivates[5] = averageDerivates[5] / 49;

        // If average derivate is over 3, print debug info and zero all values
        if (averageDerivates[0] > 3) {
            sprintf(output, "X-axis motion (averageDerivates[0]: %.2f)\n", averageDerivates[0]);
            System_printf(output);
            System_flush();
            for(m = 0; m < 49; m++) {
                derivates[0][m] = 0;
                derivates[1][m] = 0;
                derivates[2][m] = 0;
                derivates[3][m] = 0;
                derivates[4][m] = 0;
                derivates[5][m] = 0;
            }
            derivateIndex = 0;
        }
        if (averageDerivates[1] > 3) {
            sprintf(output, "Y-axis motion (averageDerivates[1]: %.2f)\n", averageDerivates[1]);
            System_printf(output);
            System_flush();
            for(m = 0; m < 49; m++) {
                derivates[0][m] = 0;
                derivates[1][m] = 0;
                derivates[2][m] = 0;
                derivates[3][m] = 0;
                derivates[4][m] = 0;
                derivates[5][m] = 0;
            }
            derivateIndex = 0;
        }
        if (averageDerivates[2] > 3) {
            sprintf(output, "Z-axis motion (averageDerivates[2]: %.2f)\n", averageDerivates[2]);
            System_printf(output);
            System_flush();
            for(m = 0; m < 49; m++) {
                derivates[0][m] = 0;
                derivates[1][m] = 0;
                derivates[2][m] = 0;
                derivates[3][m] = 0;
                derivates[4][m] = 0;
                derivates[5][m] = 0;
            }
            derivateIndex = 0;
        }

        // Zero all average values after each call
        averageDerivates[0] = 0;
        averageDerivates[1] = 0;
        averageDerivates[2] = 0;
        averageDerivates[3] = 0;
        averageDerivates[4] = 0;
        averageDerivates[5] = 0;
    }

    if (derivateIndex < 49) {
        derivateIndex++;
    }

}

// Kellokeskeytyksen kÃ¤sittelijÃ¤
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
