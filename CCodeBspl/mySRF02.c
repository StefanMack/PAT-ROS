/* Einfaches C-Programm zum Auslesen des SRF04 Ultraschallsensors:
0x51 an Adresse 0x00 triggert eine Messung und Messwertausgabe in cm.
Anschließend kann der Messwert als MSB und LSB an Adresse 0x02
ausgelesen werden. Um diese Adresse zu setzen ist ein Schreibbefehl
mit Wert 0x00 nötig.
S. Mack, 28.2.2020 */ 

#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#include<unistd.h>
#include<signal.h>
#define BUFFER_SIZE 2


void sigfunc(); // von Signalhandler bei strg + c aufgerufen

int file; // Handle für das SysF des I2C busses

int main(){
   signal(SIGINT, sigfunc); // Signalhandler für strg + c
   printf("Starting the SRF02 test application\n");
   if((file=open("/dev/i2c-2", O_RDWR)) < 0){ // I2C-Bus 2
      perror("failed to open the bus\n");
      return 1;
   }
   if(ioctl(file, I2C_SLAVE, 0x70) < 0){ // Adresse Sensor: 0x70
      perror("Failed to connect to the sensor\n");
      return 1;
   }

   while(1){
      // Messwert in cm anfordern: 0x51 an 0x00 schreiben
      char writeBuffer[2] = {0x00,0x51};
      if(write(file, writeBuffer, 2)!=2){
         perror("Failed to reset the read address\n");
         return 1;
      }
      sleep(1);
      char addrBuffer[1] = {0x02}; 
      char buf[BUFFER_SIZE];
      write(file, addrBuffer, 1); // Register 0x02 zum lesen setzen
      if(read(file, buf, BUFFER_SIZE)!=BUFFER_SIZE){ // 2 Byte lesen
         perror("Failed to read in the buffer\n");
         return 1;
      }
      printf("Buffer: %#04x, %#04x \n", buf[0], buf[1]);
      int distance = (buf[0]<<8)|buf[1]; // aus MSB und LSB Integer 
      printf("The distance is %4.0d cm\n", distance);
   }
   close(file);
   return 0;
}

void sigfunc(){
   close(file);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}


