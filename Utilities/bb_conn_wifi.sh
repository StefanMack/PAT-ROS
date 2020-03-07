#!/bin/bash
# Skript zum Herstellen einer SSH-Verbindung mit dem BB via WiFi
# Skript muss von Linux PC aus ausgeführt werden. IP-Adresse der
# WiFi-Verbindung ist je nach Router unterschiedlich!
# Der Variablen wifi_ip unten korrekte IP-Adresse zuweisen. 
# Beim ASCII-Bild müssen bestimmte Zeichen mit \ "escaped" werden.
# S. Mack, 25.12.19


wifi_ip=192.168.178.77 
echo ............................................................
echo Verbindung wird über WiFi-Adresse $wifi_ip hergestellt
echo ............................................................
echo
echo -e "\
  ,-~~-.___.
 / |  '     \ 
(  )         0 
 \_/-, ,----' 
    ====           // 
   /  \-'~;    /~~~(O) 
  /  __/~|   /       |  
=(  _____| (_________|"

echo
ssh 192.168.178.71 -l beagle -p 22

