/* Beispiel AIN0 auslesen in C++ aus Buch D. Molloy.
Gleiche Funktionaliät wie myADC.c.
Kompilieren mit g++ myADC.cpp -o myADC
S. Mack, 1.3.20*/

#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include<unistd.h>

using namespace std;

#define ADC_PATH "/sys/bus/iio/devices/iio:device0/in_voltage"

int readAnalog(int number){
   stringstream ss;
   ss << ADC_PATH << number << "_raw";
   fstream fs;
   fs.open(ss.str().c_str(), fstream::in);
   fs >> number;
   fs.close();
   return number;
}

int main(int argc, char* argv[]){
   cout << "Starting the myADC  program" << endl;
   while(1){
      int value = readAnalog(0);
      cout << "Reading ADC: " << value << " LSB \n" << endl;
      sleep(1);
   }
   return 0;
}
