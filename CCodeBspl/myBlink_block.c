/* C-Programmbeispiel für die Ansteuerung der GPIOs über das SysFS.
Blockweiser Dateizugriff.
Hier wird der GPIO Pin P9_25 (GPIO 117) getoggelt.
fflush() ist wichtig, damit Wert wirklich in SysF geschrieben wird.
Generell ist bitweiser Dateizugriff mit write besser. Siehe
myBink.c
S. Mack, 5.3.20 */


#include <stdio.h>
#include<stdlib.h>
#include <unistd.h> //nötig für sleep()
#include<signal.h>

FILE* fp; // Global wegen sigfunc()

void sigfunc(); // schließt SysF bei strg + c

int main()
{
   signal(SIGINT, sigfunc);

   fp = fopen("/sys/class/gpio/gpio117/direction", "w");
   fprintf(fp,"out");
   fflush(fp);
   fclose(fp);

   fp = fopen("/sys/class/gpio/gpio117/value", "w");
   while(1){
      fprintf(fp,"%d",1);
      fflush(fp);
      sleep(1);
      fprintf(fp,"%d",0);
      fflush(fp);
      sleep(1);
   }
}

void sigfunc(){
   fclose(fp);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}
