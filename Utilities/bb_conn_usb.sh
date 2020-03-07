#!/bin/bash
# Skript zum Herstellen einer SSH-Verbindung mit dem BB via USB
# Skript muss von Linux-PC ausgeführt werden
# Beim ASCII-Bild müssen bestimmte Zeichen mit \ "escaped" werden.
# S. Mack, 25.12.19

echo ............................................................
echo Verbindung wird über USB-Netzadresse 192.168.6.2 hergestellt
echo ............................................................
echo
echo -e "\
          ._
         |  )
         ; (,--.__;|
    ____('_,.__.--.|___
   /_____#___\`3________\\
  /______#______________\\
 /_______________________\\
   |                   |
   |                   |
   |                   |
   |                   |
   |___________________|"

echo

ssh 192.168.6.2 -l beagle -p 22
