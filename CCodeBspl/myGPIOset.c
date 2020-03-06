/* Quelle Buch D. Molloy.
Über das SysFS interaktiv einen GPIO schalten.
Verwendung von fprintf() also mit Buffer.
Kompilieren mit gcc myGPIOset.c -lm -o myGPIOset
S. Mack, 5.3.20 */

#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#define GPIO_PATH "/sys/class/gpio/gpio117"

void writeGPIO(char filename[], char value[]);

int main(int argc, char* argv[]){
   if(argc!=2){
	printf("Usage is myGPIOset  and one of:\n");
        printf("   on or off\n");
	printf(" e.g. mysetGPIO on\n");
        return 2;
   }
   printf("Starting the myGPIOset program\n");
   printf("The current GPIO Path is: " GPIO_PATH "\n");

   writeGPIO("/direction", "out");
   // select whether command is on, off, flash or status
   if(strcmp(argv[1],"on")==0){
        printf("Turning GPIO on\n");
        writeGPIO("/value", "1");
   }
   else if (strcmp(argv[1],"off")==0){
        printf("Turning GPIO off\n");
        writeGPIO("/value", "0");
   }
   else{
	printf("Invalid command!\n");
   }
   printf("Finished the myGPIOset Program\n");
   return 0;
}

void writeGPIO(char filename[], char value[]){
   FILE* fp;   // create a file pointer fp
   char  fullFileName[100];  // to store the path and filename
   sprintf(fullFileName, GPIO_PATH "%s", filename); // write path and filename
   fp = fopen(fullFileName, "w+"); // open file for writing
   fprintf(fp, "%s", value);  // send the value to the file
   fclose(fp);  // close the file using the file pointer
}
