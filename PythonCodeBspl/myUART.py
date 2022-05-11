#!/usr/bin/python3
# -*- coding: utf-8 -*-
# UART-Kommunikation mit dem Arduinoprogramm UARTComm_BB_Ardu
# mittels Python-Bibliothek pySerial
# Bei der Messanfrage des BB an den Arduino wird eine Ziffer von
# 0...1 mit gesendet. Da der Arduino nur ein ASCII-Zeichen als
# Startsignal für den Messvorgang liest, gehen keine zweistelligen
# Zahlen.
# P9_21 ist TX, P9_22 ist RX am BB fuer UART2 (Grove Buchse auf BB-Board oberhalb SD-Karte
# P9_24 ist TX, P9_26 ist RX am BB fuer UART1 (Grove Buchse auf Cape GPIO 15,14)
# P9_11 ist RX, P9_13 ist TX am BB fuer UART4 (Grove Buchse auf Cape GPIO 30,31)
# ACHTUNG: Falls Cape verwendet wird, richtigen Spannungspegel einstellen, auf 
# BB-Board nur 3,3 V Pegel verwenden. 5 V Pegel funktioniert vermutlich nicht auf Cape

# S. Mack, 11.5.22

from subprocess import call
import serial
import sys
from time import sleep

print("Python-Interpreter: {}\n".format(sys.version))
# Pinkonfiguration UART setzen
#command = "config-pin p9.21  uart" # Grove Buchse auf BB Bord ueber SD-Karte
#command = "config-pin p9.24  uart" # Grove Buchse auf BB cape GPIO 15
command = "config-pin p9.11  uart" # Grove Buchse auf BB cape GPIO 30
call(command, shell= True) # execute command in Bash Shell
#command = "config-pin p9.22  uart" # Grove Buchse auf BB Bord ueber SD-Karte
#command = "config-pin p9.26  uart" # Grove Buchse auf BB cape GPIO 14
command = "config-pin p9.13  uart" # Grove Buchse auf BB cape GPIO 31
call(command, shell= True)



ziffer = 0 # Zähler von 0...9, nur Ziffern, da ASCII an Arduino 

print('Vorher Arduino Reset drücken - Abbruch mit strg + c')
print('...zwei Sekunden warten...')
sleep(2)

try:
    #port = serial.Serial(port='/dev/ttyO2', baudrate = 115200, timeout = 1)
    #port = serial.Serial(port='/dev/ttyO1', baudrate = 115200, timeout = 1)
    port = serial.Serial(port='/dev/ttyO4', baudrate = 115200, timeout = 1)
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

