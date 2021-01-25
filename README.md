# MAX30100-tiva4c
Reading MAX30100 heart beat sensor using TI EK-TM4C1294XL and implementing a TCP/IP server on the launchpad, so that certain commands can be given from PC using Hercules,Putty etc.  Also through tcp/ip the launchpad need to connect to a ntp server for getting the real time.


PURPOSE
The purpose of this project is to write an application on EK-TM4C1294XL that can connect to a NTP server to get the instant real time and also that can connect to a MAX30100 heart beat sensor to read the heart beat.


This project has multithreaded software architecture. It can give heartbeat rate and at the same time can connect to servers and clients.
The program using EMAC system to connect the Ethernet, and using TCP/IP protocol to connect servers and clients.
Heart Beat Reading
The given system architecture has a servertask that connect with a client(Hercules) to get the requests. When “READBPM” request arrived, the related case post a semaphore(semaphore1) to signal the heartBeatTask to start the reading and also pend a Mailbox(mailbox0) to get the read values in the mailbox queue.
heartBeatTask starts the IIC connection through sensor’s IIC address (0x57). Then setting the parameters of the sensor is needed.
IIC_writeReg(MAX30100_ID,0x07,0x07); 
Setting resolution with register 0x07. The LSB two bits are defines the pulsewidth(pw). “11” means that pw = 1600 us. The third bit defines samples per second(sps). Since at taken pw only sample rates that available are 50 and 100 sps. ”001” means 100 samples per second(100 Hz).
IIC_writeReg(MAX30100_ID, 0x06, 0x02); 

Setting mode configuration register (0x06) to HR only(0x02) mode.

IIC_writeReg(MAX30100_ID, 0x06, 0x03); 
Setting mode configuration register(0x06) to HR and SPO2(0x03)

IIC_writeReg(MAX30100_ID, 0x09, 0x0f); 
 Setting IRled current to 50 mA through register 0x09.

Given 3 line below is to clear the FIFO registers.
IIC_writeReg(MAX30100_ID,0x02,0); //Clearing the FIFO_WR_PTR
IIC_writeReg(MAX30100_ID,0x03,0); //Clearing the OVF_COUNTER
IIC_writeReg(MAX30100_ID,0x04,0); //Clearing the FIFO_RD_PTR


IIC_writeReg(MAX30100_ID,0x01,0xf0); 
	Enabling the interrupts start the sensor to read heart rate.
After the setting is done, the task starts its infinite loop to get the 16 samples from Fifo Data(0x05) when the Fifo is full (this is done by reading the msb of 0x00 register).
The samples includes the heart rate values with DC offsets. Therefore the DC offset is need to be extracted. I have tried to extract the DC values from 16 sample but could not manage it. DC offset extracting formulation is given below:
w(t)=x(t)+∝∗w(t−1)
y(t)=w(t)−w(t−1)
y(t):       is the output of the filter
x(t):       current input/value
w(t):      intermediate value, acts like the history of the DC value
α:          is the response constant of the filter (I chose 0.95)

After the DC values extracted, the sample values need to applied with Mean median filter to extract the noise and Butterworth filter to remove the higher level harmonies. Since DC offset still exist I could not proceeded to these two part. 
At the end of the loop theese average of the raw sample values is put into the mailbox queue. The mailbox queue has a max number of 5 messages and the size of messages are 4 bytes. 
And on the other side, the related case takes 5 average raw values and share with clients (Hercules).  


NTP Time Server
