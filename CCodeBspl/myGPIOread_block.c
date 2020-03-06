/* Auslesen des GPIO115 (P9_27) des BB über das SysFS.
Kompilieren mit: gcc myGPIOread.c -lm -o myGPIOread.
Blockweises Lesen mit fread(). 
Ohne fseek() und fflush() wird immer nur der erste Wert wiederholt
ausgegeben, d.h. dann müsste das SysF nach jedem Lesevorgang wieder 
geschlossen werden.

Der byteweise Zugriff auf das SysFS, siehe myGPIOread.c ist die
bessere Methode, GPIOs auszulesen.

S. Mack, 2.3.20 */

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<signal.h>

#define GPIO_PATH "/sys/class/gpio/gpio115"

FILE* fp; // File Pointer global wegen fclose() in sigfunc()

void writeGPIO(char filename[], char value[]);
void sigfunc();

int main(){
   signal(SIGINT, sigfunc);
   printf("Starting the myGPIOread program\n");
   printf("The GPIO Path to read is: " GPIO_PATH "\n");
   writeGPIO("/direction", "in"); // GPIO als Eingang
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", "/value"); // write path and filename
   char val[1];
   fp = fopen(fullFileName, "r"); // open file for reading
   
   while(1){
      fseek(fp, 0L, SEEK_SET); // setzt Lesezeiger an Anfang
      fread(val, sizeof(char), 1, fp); // Einblock Größe Byte lesen
      printf("Value GPIO String: %x \n", val[0]);
      printf("Value GPIO Int: %d \n", atoi(val));
      fflush(fp); // nicht gelesene Zeichen im Puffer löschen
      sleep(1);
  }
   
   return 0;
}

void writeGPIO(char filename[], char value[]){
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", filename); // write path and filename
   fp = fopen(fullFileName, "w+"); // open file for writing
   fprintf(fp, "%s", value);  // send the value to the file
   fclose(fp);  // close the file using the file pointer
}

void sigfunc(){
   fclose(fp);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}
