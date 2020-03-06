/* Ansteuern des PWM-Ausgangs am P9_14 (GPIO 50) via SysFS
Im SRP-Image muss mit "config-pin -a p9.14 pwm" der Pin konfiguriert werden,
da sein Default-Zustand GPIO ist.
P9_14 gehört zu dem SysF /sys/class/pwm/pwm-4:0. Dies kann bei anderen Images
anders sein.
Der Output dieses Programms ist für einen Modellbauservo gedacht, kann aber
auch mit einer LED visualisiert werden.
Code teilweise übernommen aus:
www.element14.com/community/community/project14/visionthing/blog/2019/11/16/beagleboard-ai-brick-recovery-procedure
S. Mack, 5.3.20
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void setupPWM() {
   // Bash Befehl Pinkonfiguration
   int status = system("config-pin -a p9.14 pwm");
   printf("Exitcode config-pin: %d\n", status);
   
   FILE *period, *pwm;
   pwm = fopen("/sys/class/pwm/pwm-4:0/enable", "w");
   fseek(pwm,0,SEEK_SET);
   fprintf(pwm,"%d",1);
   fflush(pwm);
   fclose(pwm);
     
   period = fopen("/sys/class/pwm/pwm-4:0/period", "w");
   fseek(period,0,SEEK_SET);
   fprintf(period,"%d",20000000);
   fflush(period);
   fclose(period);
}  

void pwm_duty(int duty_cycle)
{
   FILE *duty;
   duty = fopen("/sys/class/pwm/pwm-4:0/duty_cycle", "w");
   fseek(duty,0,SEEK_SET);
   fprintf(duty,"%d",duty_cycle);
   fflush(duty);
   fclose(duty);
}
 
int main() { 
   setupPWM();
   while(1){ 
      pwm_duty(1200000);
      sleep(1);
      pwm_duty(4800000);
      sleep(1);
   }
}
