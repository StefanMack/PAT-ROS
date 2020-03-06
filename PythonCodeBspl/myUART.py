#!/usr/bin/python3
# -*- coding: utf-8 -*-
# UART-Kommunikation mit dem Arduinoprogramm UARTComm_BB_Ardu
# mittels Python-Bibliothek pySerial
# Bei der Messanfrage des BB an den Arduino wird eine Ziffer von
# 0...1 mit gesendet. Da der Arduino nur ein ASCII-Zeichen als
# Startsignal für den Messvorgang liest, gehen keine zweistelligen
# Zahlen.
#
# S. Mack, 5.3.20

from subprocess import call
import serial
import sys
from time import sleep

print("Python-Interpreter: {}\n".format(sys.version))
# Pinkonfiguration UART setzen
command = "config-pin -a p9.21  uart"
call(command, shell= True) # execute command in Bash Shell
command = "config-pin -a p9.22  uart"
call(command, shell= True)



ziffer = 0 # Zähler von 0...9, nur Ziffern, da ASCII an Arduino 

print('Vorher Arduino Reset drücken - Abbruch mit strg + c')
print('...zwei Sekunden warten...')
sleep(2)

try:
    port = serial.Serial(port='/dev/ttyO2', baudrate = 115200, timeout = 1)
    sleep(1) # nötig für eventuellen Reset Arduinoprogramm
    print('Arduino sagt: ', port.readline().decode("utf-8")) # Testen ob Arduino bereit
    sleep(0.5)
    port.readline() # Dummy Read
    port.flushInput() # UART Eingangspuffer leeren
    print('Messungen werden vom Arduino angefordert...')
    while True:
        # Mit encode() wird aus Ziffer Byte des ASCII-zeichens
        port.write(str(ziffer).encode()) # ADC-Messung anfordern
        result = port.readline() # Von Schnittstelle bis \n lesen
        print('Arduino sagt: {}'.format(str(result)))
        sleep(1)
        ziffer = (ziffer +1)%10
except KeyboardInterrupt:
    print()
    print('Programm mit strg + c abgebrochen bzw. Laufzeitfehler')  
finally:
    port.close() # Serielle Schnittstelle schliessen.
    print('Schnittstelle geschlossen')

