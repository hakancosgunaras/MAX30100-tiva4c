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



#define SOCKETTEST_IP     "192.168.1.34" //My launchpads IP address
#define TASKSTACKSIZE     4096
#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030



extern Semaphore_Handle semaphore0;     // posted by httpTask and pended by clientTask
extern Semaphore_Handle semaphore1;
extern Mailbox_Handle mailbox0;
/*
extern Semaphore_Handle semaphore2;
extern Semaphore_Handle semaphore3;
extern Swi_Handle swi0;
char temp_time[32];
int instant_time;
int ctr;
int printTime;
*/

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

Void httpTask(UArg arg0, UArg arg1)
{
    //do nothing
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



float getTemperature(void)
{
    // dummy return
    //
    return atof(tempstr);
}


Void serverSocketTask(UArg arg0, UArg arg1)
{

    float temp;

    int serverfd, new_socket, valread, len;
    struct sockaddr_in serverAddr, clientAddr;
    char buffer[30];
    char outstr[30], tmpstr[30];
    bool quit_protocol;

    serverfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverfd == -1)
    {
        System_printf(
                "serverSocketTask::Socket not created.. quiting the task.\n");
        return;     // we just quit the tasks. nothing else to do.
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(INCOMING_PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Attaching socket to the port
    //
    if (bind(serverfd, (struct sockaddr*) &serverAddr, sizeof(serverAddr)) < 0)
    {
        System_printf("serverSocketTask::bind failed..\n");

        // nothing else to do, since without bind nothing else works
        // we need to terminate the task
        return;
    }
    if (listen(serverfd, 3) < 0)
    {

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

    // DC filter variables
    float filtered_w,prev_w=0,filtered_result[16];
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

/*
    //Mean filter variables
    float M,m_sum,m_avg=0,m_result;
    int m_index,m_count;
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

                //Extracting DC offset
                filtered_w = ir[i] + alpha * prev_w;
                filtered_result[i] = filtered_w - prev_w;
                prev_w = filtered_w;
                average += filtered_result[i];

            }

            average = average / 16;
            bpm=average;
            System_printf("average degeri %d \n", average);
            average = 0;

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

void getTimeStr(char *str)
{
    // dummy get time as string function
    // time needs to be gotten from NTP servers
    // This code needs to be improved
    //
    strcpy(str, "2021-01-25 10:30:00");
}
/*
Void Timer_ISR(UArg arg1) // executed every second
{
    Swi_post(swi0);

}
Void SWI_ISR(UArg arg1)
{
// update seconds, then minutes if seconds exceeds 60
// update others if necessary

            Semaphore_post(semaphore2);
            Semaphore_pend(semaphore3,BIOS_WAIT_FOREVER);
            instant_time  = temp_time[0]*16777216 +  temp_time[1]*65536 + temp_time[2]*256 + temp_time[3]; // Calculating current time
            instant_time += 10800;
            instant_time += ctr++;
            System_printf("TIME: %s", ctime(&instant_time)); //convert time value to string and printf

            System_flush();


}





Void ntpSocketTask(UArg arg0, UArg arg1)
{
    while(1){
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //
        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);


        GPIO_write(Board_LED0, 1); // turn on the LED

        // connect to SocketTest program on the system with given IP/port
        // send hello message whihc has a length of 5.
        //
        connectNTP("128.138.140.44", 37,instant_time, strlen(instant_time));

        Semaphore_post(semaphore3);
        GPIO_write(Board_LED0, 0);  // turn off the LED

    }

}

void connectNTP(char *serverIP, int serverPort, char *data, int size)
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
*/
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
