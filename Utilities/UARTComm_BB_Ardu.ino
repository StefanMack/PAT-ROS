/* Arduinoprogramm zum Testen der UART-Kommunikation zwischen BeagleBone
und Arduino. Nach dem Start sendet der Arduino im Sekundentakt eine
Bereitschaftsinfo.
Nach Empfang eines Zeichens vom BeagleBone startet die Funktion loop().
Darin wird nach jedem empfangenem Zeichen eine ADC-Messung ausgelöst.
Das empfangene Zeichen und der ADC.Wert wird über UART an den BeagleBone
gesendet.
Testen der Kommunikation auf dem BeagleBone mit
minicom -b 115200 -o -D /dev/ttyOxx mit xx Nummer der UART-Schnittstelle.
Vorher mit config-pin -a px.y uart die TX und RX Pins konfigurieren.
!! Arduino mit 3,3 V Pegel verwenden !!
S. Mack, 5.3.20 */


int adc_val= 0;  // Ausgabewert ADC
int inByte = 0;  // Auf RX empfangenes Byte

void setup()
{
  pinMode(13,OUTPUT);
  int i=0;
  Serial.begin(115200, SERIAL_8N1);
  while(Serial.available()<=0){   // Warten bis Zeichen auf RX zum Start
    Serial.print(i,DEC);
    Serial.println(": Arduino bereit...warte auf Input...");
    delay(1000);
    i+=1;
  }
}

void loop()
{
  if (Serial.available() > 0) {   // Zeichen empfangen
    inByte = Serial.read();
    Serial.flush();
    adc_val = analogRead(A0);
    Serial.write(inByte);
    Serial.print(",");
    Serial.println(adc_val,DEC);
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);    
  }
}
