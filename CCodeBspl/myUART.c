/* UART-Kommunikation mit dem Arduino mit Firmware UARTComm_BB_Ardu.ino.
Wesentliche Teile uebernommen von J. Plate
netzmafia.de/skripten/hardware/RasPi/RasPi_Serial.html

S. Mack, 5.3.20 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>


int open_serial(void);
int sendbytes(char*, int, int);
void sigfunc(); // von Signalhandler bei strg + c aufgerufen
int fd; // File-Handle fuer UART-Schnittstelle, global wegen sigfunc()

int main(){
   // Bash Befehl Pinkonfiguration UART2
   system("config-pin -a p9.21 uart");
   system("config-pin -a p9.22 uart");

   printf("Vorher Arduino Reset drücken - Abbruch mit strg + c\n");
   printf("...zwei Sekunden warten...\n");
   usleep(2000000);

   fd = open_serial(); 
   signal(SIGINT, sigfunc); // Signalhandler für strg + c

   char rxbuf[101];
   char txbuf[1]="1";
   int rxanz;
   int ziffer=0;
   char char_ziffer='0';

   rxanz = read(fd, (void*)rxbuf, 100);
   if (rxanz < 0) perror("Read failed!");
   else if (rxanz == 0) perror("No data!");
   else {
      rxbuf[rxanz] = '\0';      /* Stringterminator */
      printf("Arduino sagt: %s", rxbuf);
   }

   while(1){
      char_ziffer = ziffer +'0';
      txbuf[0]=char_ziffer;
      sendbytes(txbuf,1,fd);
      usleep(1000000);
      rxanz = read(fd, (void*)rxbuf, 100);
      if (rxanz < 0) perror("Read failed!");
      else if (rxanz == 0) perror("No data!");
      else {
         rxbuf[rxanz] = '\0';
         printf("Arduino sagt: %s", rxbuf);
      }
      ziffer = (ziffer + 1)%10;
   }
   close(fd);
   return 0;
}



int open_serial(void){
   /*  Oeffnet seriellen Port
   Gibt das Filehandle zurueck oder -1 bei Fehler */

   int fd;                    /* Filedeskriptor */
   struct termios options;    /* Schnittstellenoptionen */

   /* Port oeffnen - read/write, kein "controlling tty", Status von DCD ignorieren */
   fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY | O_NDELAY);
   if(fd >= 0) {
      /* get the current options */
      fcntl(fd, F_SETFL, 0);
      if (tcgetattr(fd, &options) != 0) return(-1);
      memset(&options, 0, sizeof(options)); /* Structur loeschen, ggf. vorher sichern
                                          und bei Programmende wieder restaurieren */
      /* Baudrate setzen */
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);

      /* setze Optionen */
      options.c_cflag &= ~PARENB;         /* kein Paritybit */
      options.c_cflag &= ~CSTOPB;         /* 1 Stoppbit */
      options.c_cflag &= ~CSIZE;          /* 8 Datenbits */
      options.c_cflag |= CS8;

      /* 19200 bps, 8 Datenbits, CD-Signal ignorieren, Lesen erlauben */
      options.c_cflag |= (CLOCAL | CREAD);

      /* Kein Echo, keine Steuerzeichen, keine Interrupts */
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_iflag = IGNPAR;           /* Parity-Fehler ignorieren */
      options.c_oflag &= ~OPOST;          /* setze "raw" Input */
      options.c_cc[VMIN]  = 0;            /* warten auf min. 0 Zeichen */
      options.c_cc[VTIME] = 10;           /* Timeout 1 Sekunde */
      tcflush(fd,TCIOFLUSH);              /* Puffer leeren */
      if (tcsetattr(fd, TCSAFLUSH, &options) != 0) return(-1);
   }
   return(fd);
}



int sendbytes(char * Buffer, int Count, int fd) {
   int sent;
   sent = write(fd, Buffer, Count);
   if (sent < 0) {
      perror("sendbytes failed - error!");
      return -1;
   }
   if (sent < Count) perror("sendbytes failed - truncated!");
   return(sent);
}

void sigfunc(){
   close(fd);
   sleep(1);
   printf("\n");
   printf("Bye, bye, ...\n");
   exit(0);
}

