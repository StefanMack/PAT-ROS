/* Auslesen des ADC0 des BB über das SysFS.
Der Dateizugriff erfolgt mit Buffer via fread(). Angeblich kann 
schneller ohne Buffer mit read() gelesen werden.
S. Mack, 5.3.20
*/

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<signal.h>

// Hier für anderen AIN bei iio:device Zahl ändern 
#define ADC_PATH "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"

FILE* fp; // File Pointer als Handle für das SysF, global wg. sigfunc()
int readADC();
void sigfunc(); // von Signalhandler bei strg + c aufgerufen

int main(){
   signal(SIGINT, sigfunc); // Signalhandler für strg + c
   printf("Starting the myADC program\n");
   printf("The ADC Path is: " ADC_PATH "\n");

   while(1){
      printf("Reading ADC: %d LSB \n", readADC());
      //printf(" %d \n", readADC());
      sleep(1);
   }
}

int readADC(){
   char val[5];
   fp = fopen(ADC_PATH, "r");
   fread(val, sizeof(char), sizeof(val)-1, fp);
   fclose(fp);
   return(atoi(val));
}

void sigfunc(){
   fclose(fp);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}
