/* Ansteuerung des GPIO117 (P9_25) des BB über das SysFS mittels
byteweises Schreiben. lseek() nach dem write()-Befehl scheint anders
als bei myGPIOread.c nicht nötig zu sein.
Die Funktion wrtieGPIO() könnte man auch für das Toggeln verwenden,
wenn close() getrennt davon ausgeführt wird.
Kompilieren mit: gcc myBlink.c -o myBlink.
S. Mack, 1.3.20 */

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<signal.h>
#include<fcntl.h>

#define GPIO_PATH "/sys/class/gpio/gpio117"

int fp; // Global wegen sigfunc()

void writeGPIO(char filename[], char value[]);
void sigfunc(); // schließt SysF bei strg + c

int main(){
   signal(SIGINT, sigfunc); // Signalhandler
   printf("The blinking GPIO Path is: " GPIO_PATH "\n");
   writeGPIO("/direction", "out");
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", "/value"); // write path and $
   fp = open(fullFileName, O_WRONLY); // open file for writing

   while(1){
      printf("Turning GPIO on\n");
      write(fp, "1", sizeof("1"));  // send the value to the file
      sleep(1);
      printf("Turning GPIO off\n");
      write(fp, "0", sizeof("0"));  // send the value to the file
      sleep(1);
   }
}

void writeGPIO(char filename[], char wr_val[]){
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", filename); // write path and fil$
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
