/*  Hakan Coşgunaras
 *  Project 14
 *
 *      This project is about reading from MAX30102 based heart beat sensor. You are supposed to implement
 *  a TCP/IP server on the launchpad that can be connected from your PC using Hercules, Putty,
*   etc, programs. Once you connect to the server, you can give the commands such as
*
*   “READ HEARTBEAT” <--- Reads from the heart beat sensor and displays
*   “TIME”           <--- Show the time acquired from NTP server
*   “SHUTDOWN”       <--- close the connection wait for another one
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include <ti/drivers/I2C.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>

/* Example/Board Header file */
#include "Board.h"
#include "iic_lib.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#define MAX30100_ID                 0x57 //My sensor's IIC address

// In this example MAX30100 is connected to EK-TM4C1294-XL launchpad
//
// PD0   (launchpad) --> SCL (MAX30100 breakout board)
// PD1   (launchpad) --> SDA
// +3.3V (launchpad) --> GND
// GND   (launchpad) --> VCC
//


#define HOSTNAME          "api.openweathermap.org"
#define REQUEST_URI       "/data/2.5/forecast/?id=315202&APPID="
#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define SOCKETTEST_IP     "192.168.1.34" //My launchpads IP address
#define TASKSTACKSIZE     4096
#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030



extern Semaphore_Handle semaphore0;     // posted by httpTask and pended by clientTask
extern Semaphore_Handle semaphore1;
extern Mailbox_Handle mailbox0;

char   tempstr[20];                     // temperature string

/*
 *  ======== printError ========
 */
void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}


bool sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd, connStat, numSend;
    bool retval=false;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        close(sockfd);
        return false;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     // convert port # to network order
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("sendData2Server::Error while connecting to server\n");
    }
    else {
        numSend = send(sockfd, data, size, 0);       // send data to the server
        if(numSend < 0) {
            System_printf("sendData2Server::Error while sending data to server\n");
        }
        else {
            retval = true;      // we successfully sent the temperature string
        }
    }
    System_flush();
    close(sockfd);
    return retval;
}

Void clientSocketTask(UArg arg0, UArg arg1)
{
    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);

        GPIO_write(Board_LED0, 1); // turn on the LED

        // connect to SocketTest program on the system with given IP/port
        // send hello message whihc has a length of 5.
        //
        if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, tempstr, strlen(tempstr))) {
            System_printf("clientSocketTask:: Temperature is sent to the server\n");
            System_flush();
        }

        GPIO_write(Board_LED0, 0);  // turn off the LED
    }
}

void getTimeStr(char *str)
{
    // dummy get time as string function
    // time needs to be gotten from NTP servers
    // This code needs to be improved
    //
    strcpy(str, "2021-01-07 12:34:56");
}

float getTemperature(void)
{
    // dummy return
    //
    return atof(tempstr);
}


Void serverSocketTask(UArg arg0, UArg arg1)
{
    int serverfd, new_socket, valread, len;
    struct sockaddr_in serverAddr, clientAddr;
    float temp;
    char buffer[30];
    char outstr[30], tmpstr[30];
    bool quit_protocol;

    serverfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverfd == -1) {
        System_printf("serverSocketTask::Socket not created.. quiting the task.\n");
        return;     // we just quit the tasks. nothing else to do.
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(INCOMING_PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Attaching socket to the port
    //
    if (bind(serverfd, (struct sockaddr *)&serverAddr,  sizeof(serverAddr))<0) {
         System_printf("serverSocketTask::bind failed..\n");

         // nothing else to do, since without bind nothing else works
         // we need to terminate the task
         return;
    }
    if (listen(serverfd, 3) < 0) {

        System_printf("serverSocketTask::listen() failed\n");
        // nothing else to do, since without bind nothing else works
        // we need to terminate the task
        return;
    }

    while(1) {

        len = sizeof(clientAddr);
        if ((new_socket = accept(serverfd, (struct sockaddr *)&clientAddr, &len))<0) {
            System_printf("serverSocketTask::accept() failed\n");
            continue;               // get back to the beginning of the while loop
        }

        System_printf("Accepted connection\n"); // IP address is in clientAddr.sin_addr
        System_flush();

        // task while loop
        //
        quit_protocol = false;
        do {

            // let's receive data string
            if((valread = recv(new_socket, buffer, 10, 0))<0) {

                // there is an error. Let's terminate the connection and get out of the loop
                //
                close(new_socket);
                break;
            }

            // let's truncate the received string
            //
            buffer[10]=0;
            if(valread<10) buffer[valread]=0;

            System_printf("message received: %s\n", buffer);

            if(!strcmp(buffer, "HELLO")) {
                strcpy(outstr,"GREETINGS 200\n");
                send(new_socket , outstr , strlen(outstr) , 0);
                System_printf("Server <-- GREETINGS 200\n");
            }
            else if(!strcmp(buffer, "GETTIME")) {
                getTimeStr(tmpstr);
                strcpy(outstr, "OK ");
                strcat(outstr, tmpstr);
                strcat(outstr, "\n");
                send(new_socket , outstr , strlen(outstr) , 0);
            }
            else if(!strcmp(buffer, "GETTEMP")) {
                temp = getTemperature();
                sprintf(outstr, "OK %5.2f\n", temp);
                send(new_socket , outstr , strlen(outstr) , 0);
            }
            else if(!strcmp(buffer, "QUIT")) {
                quit_protocol = true;     // it will allow us to get out of while loop
                strcpy(outstr, "BYE 200");
                send(new_socket , outstr , strlen(outstr) , 0);
            }
            else if (!strcmp(buffer, "READBPM"))
            {
                static float avg,avgbpm;
                int i;

                Semaphore_post(semaphore1);
                for(i=0;i<5;i++){ // 5 bpm values to be averaged
                Mailbox_pend(mailbox0, &avgbpm, BIOS_WAIT_FOREVER); //waiting for bpm values
                avg += avgbpm;
                }
                avg = avg/5;
                sprintf(outstr, "BPM %5.2f\n", avg);
                send(new_socket, outstr, strlen(outstr), 0);
            }

        }
        while(!quit_protocol);

        System_flush();
        close(new_socket);
    }

    close(serverfd);
    return;
}

/*
 *  ======== httpTask ========
 *  Makes a HTTP GET request
 */
Void httpTask(UArg arg0, UArg arg1)
{
    bool moreFlag = false;
    char data[64], *s1, *s2;
    int ret, temp_received=0, len;
    struct sockaddr_in addr;

    HTTPCli_Struct cli;
    HTTPCli_Field fields[3] = {
        { HTTPStd_FIELD_NAME_HOST, HOSTNAME },
        { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
        { NULL, NULL }
    };

    while(1) {

        System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME);
        System_flush();

        HTTPCli_construct(&cli);

        HTTPCli_setRequestFields(&cli, fields);

        ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME, 0);
        if (ret < 0) {
            HTTPCli_destruct(&cli);
            System_printf("httpTask: address resolution failed\n");
            continue;
        }

        ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
        if (ret < 0) {
            HTTPCli_destruct(&cli);
            System_printf("httpTask: connect failed\n");
            continue;
        }

        ret = HTTPCli_sendRequest(&cli, HTTPStd_GET, REQUEST_URI, false);
        if (ret < 0) {
            HTTPCli_disconnect(&cli);
            HTTPCli_destruct(&cli);
            System_printf("httpTask: send failed");
            continue;
        }

        ret = HTTPCli_getResponseStatus(&cli);
        if (ret != HTTPStd_OK) {
            HTTPCli_disconnect(&cli);
            HTTPCli_destruct(&cli);
            System_printf("httpTask: cannot get status");
            continue;
        }

        System_printf("HTTP Response Status Code: %d\n", ret);

        ret = HTTPCli_getResponseField(&cli, data, sizeof(data), &moreFlag);
        if (ret != HTTPCli_FIELD_ID_END) {
            HTTPCli_disconnect(&cli);
            HTTPCli_destruct(&cli);
            System_printf("httpTask: response field processing failed\n");
            continue;
        }

        len = 0;
        do {
            ret = HTTPCli_readResponseBody(&cli, data, sizeof(data), &moreFlag);
            if (ret < 0) {
                HTTPCli_disconnect(&cli);
                HTTPCli_destruct(&cli);
                System_printf("httpTask: response body processing failed\n");
                moreFlag = false;
            }
            else {
                // string is read correctly
                // find "temp:" string
                //
                s1=strstr(data, "temp");
                if(s1) {
                    if(temp_received) continue;     // temperature is retrieved before, continue
                    // is s1 is not null i.e. "temp" string is found
                    // search for comma
                    s2=strstr(s1, ",");
                    if(s2) {
                        *s2=0;                      // put end of string
                        strcpy(tempstr, s1+6);      // copy the string
                        temp_received = 1;
                    }
                }
            }

            len += ret;     // update the total string length received so far
        } while (moreFlag);

        System_printf("Received %d bytes of payload\n", len);
        System_printf("Temperature %s\n", tempstr);
        System_flush();                                         // write logs to console

        HTTPCli_disconnect(&cli);                               // disconnect from openweathermap
        HTTPCli_destruct(&cli);
        Semaphore_post(semaphore0);                             // activate socketTask

        Task_sleep(5000);                                       // sleep 5 seconds
    }
}

bool createTasks(void)
{
    static Task_Handle taskHandle1, taskHandle2, taskHandle3;
    Task_Params taskParams;
    Error_Block eb;

    Error_init(&eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle1 = Task_create((Task_FuncPtr)httpTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle2 = Task_create((Task_FuncPtr)clientSocketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle3 = Task_create((Task_FuncPtr)serverSocketTask, &taskParams, &eb);

    if (taskHandle1 == NULL || taskHandle2 == NULL || taskHandle3 == NULL) {
        printError("netIPAddrHook: Failed to create HTTP, Socket and Server Tasks\n", -1);
        return false;
    }

    return true;
}

/*
 *  ======== netIPAddrHook ========
 *  This function is called when IP Addr is added/deleted
 */
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    // Create a HTTP task when the IP address is added
    if (fAdd) {
        createTasks();
    }
}

Void heartBeatTask (UArg arg1, UArg arg2){

    Semaphore_pend(semaphore1, BIOS_WAIT_FOREVER); //Waiting for the semaphore signal to read bpm

    uint16_t ir[64];
   // uint16_t red[64];
    char buf[64]; //16 samples that 4 bytes for each
    int FifoRead, FifoWrite,i;
    char SampleReady[2];
    char Temp;
    char a_full;
    float bpm;
    long average=0;
    //DC filter values
    float filtered_w,prev_w,filtered_result,x;
    float alpha=0.95; // Response constant of the filter
    /*  DC Filter
     * w(t)= x(t)+alpha*w(t-1)
     * y(t)= w(t)-w(t-1)
     *
     * y(t): is the output of the filter
     * x(t): current input/value
     * w(t): intermediate value, acts like the history of the DC value
     *
     */


    IIC_OpenComm();

    IIC_readReg(MAX30100_ID, 0xFF,1,buf);
    System_printf("part ID 0x%x \n", buf[0]);

    // IIC_writeReg(MAX30100_ID,0x06,0x80); //Resetting all registers
    // Task_sleep(1); // Wait 1 ms for the resetting

    //IIC_writeReg(0x57,0x06,0x60); // power save mode

    IIC_readReg(MAX30100_ID, 0x07,1,buf);
    Temp = (buf[0] & 0xF8) | 0x07;
    IIC_writeReg(MAX30100_ID, 0x07, 0x07); //Setting resolution, 100 samples per second(100 Hz), pw=1600 us

    IIC_writeReg(MAX30100_ID, 0x06, 0x02); //Setting mode configuration reg(0x06) to HR only(0x02)
    //IIC_writeReg(MAX30100_ID, 0x06, 0x03); //Setting mode configuration reg(0x06) to HR and SPO2(0x03)

    IIC_writeReg(MAX30100_ID, 0x09, 0x0f); // Setting IRled Temp to 50 mA
    //IIC_writeReg(MAX30100_ID, 0x09, 0x90); // Setting red led Temp to 30.6 mA


    IIC_writeReg(MAX30100_ID,0x02,0); //Clearing the FIFO_WR_PTR
    IIC_writeReg(MAX30100_ID,0x03,0); //Clearing the OVF_COUNTER
    IIC_writeReg(MAX30100_ID,0x04,0); //Clearing the FIFO_RD_PTR


    IIC_writeReg(MAX30100_ID,0x01,0xf0); //Interrupt enable

/*
    // Getting out of power save mode
    IIC_readReg(0x57, 0x06, 1, buf);
    Temp = buf[0] & 0x7f;
    IIC_writeReg(0x57,0x06,Temp);
*/
        while (1)
    {
        // Checking that whether the fifo data is full or not
        IIC_readReg(MAX30100_ID, 0x00, 1, SampleReady);
        a_full = SampleReady[0] & 0x80;
        if (a_full > 1)
        { // if full then proceed to read the data


            IIC_readReg(MAX30100_ID, 0x05, 64, buf); //Reading the 16 samples
            for (i = 0; i < 16; i++)
            {
                ir[i] = (uint16_t) ((buf[(i * 4)] << 8) | buf[(i * 4) + 1]); //Getting data into array
                //ir[i]= (uint16_t)((buf[i*4]*16)+buf[(i*4)+1]);
                average += ir[i];
                /*//DC filter
                  if(i>0){
                    filtered_w = ir[i];
                    prev_w = ir[i-1];
                    filtered_w = x + alpha*prev_w;
                    filtered_result = filtered_w - prev_w;
                    ir[i]=(uint16_t)filtered_result;

            /*Mean Median Filter code here
             * Butterworth filter code here
             * BPM calculation here(the time between the two peaks)
             *
                }
                 */

            }

            average = average / 16;
            bpm=average;
            System_printf("average degeri %d \n", average);
            average = 0;
            /*
            for (i = 0; i < 16; i++)
            {
                System_printf("16 sample degeri %d , \n", ir[i]);
                //System_printf("ilk 6 sample degeri %d , %d , %d , %d , %d , %d\n", ir[0], ir[1],ir[2],ir[3],ir[4],ir[5]);
            }
             */
            System_flush();

        }
        Mailbox_post(mailbox0, &bpm, BIOS_NO_WAIT); // getting bpm values into queue

        IIC_writeReg(MAX30100_ID, 0x02, 0); //Clearing the FIFO_WR_PTR
        IIC_writeReg(MAX30100_ID, 0x03, 0); //Clearing the OVF_COUNTER
        IIC_writeReg(MAX30100_ID, 0x04, 0); //Clearing the FIFO_RD_PTR



    }

        IIC_CloseComm();



        System_flush();
}


/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the Project 14 process... \n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
