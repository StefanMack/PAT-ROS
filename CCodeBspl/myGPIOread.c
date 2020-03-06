/* Auslesen des GPIO115 (P9_27) des BB über das SysFS.
Kompilieren mit: gcc myGPIOread.c  -o myGPIOread.
Byteweises Lesen mit read(). Durch lseek(), das den
File-Deskriptor immer wieder an den Anfang setzt, muss
das SysF nich bei jedem Lesevorgang geöffnet/geschlossen
werden.
Angelehnt an elinux.org/RPi_GPIO_Code_Samples#sysfs
S. Mack, 5.3.20 */

#include<stdio.h>
#include<stdlib.h>
#include<sys/stat.h>
#include<sys/types.h>
#include<fcntl.h>
#include<unistd.h>
#include<signal.h>

#define GPIO_PATH "/sys/class/gpio/gpio115"

int fp; // File Pointer global wegen writeGPIO(),  sigfunc()

void writeGPIO(char filename[], char value[]);
void sigfunc();

int main(){
   signal(SIGINT, sigfunc);
   printf("The GPIO Path to read is: " GPIO_PATH "\n");
   writeGPIO("/direction", "in"); // GPIO als Eingang
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", "/value"); // write path and filename
   char val[1];
   fp = open(fullFileName, O_RDONLY); // open file for reading   
   if (-1 == fp){
      fprintf(stderr, "Failed to open gpio value for reading!\n");
      return(-1);
   }

   while(1){
      if( -1 == read(fp, &val, 1)){ // Ein Byte lesen, an &val schreiben
         fprintf(stderr, "Failed to read value!\n");
         return(-1);
      }
      lseek(fp, 0L, SEEK_SET);
      printf("Value GPIO String: %x \n", val[0]);
      printf("Value GPIO Int: %d \n", atoi(val));
      sleep(1);
  }   
   return 0;
}

void writeGPIO(char filename[], char wr_val[]){
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", filename); // write path and filename
   fp = open(fullFileName, O_WRONLY); // open file for writing
   if (-1 == fp) {
      fprintf(stderr, "Failed to open gpio direction for writing!\n");
   }   
   // write value in das File schreiben (* bei sizeof() wichtig, damit
   // Anzahl der Arrayelemente als Output.
   if (-1 == write(fp, &wr_val, sizeof(*wr_val))){
      fprintf(stderr, "Failed to open gpio direction for writing!\n");
   }
   close(fp);  // close the file using the file pointer
}


void sigfunc(){
   close(fp);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}
