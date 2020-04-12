#!/bin/bash
# Skript zum Herstellen einer SSH-Verbindung mit dem BB via WiFi
# Skript muss von Linux PC aus ausgeführt werden. IP-Adresse der
# WiFi-Verbindung ist je nach Router unterschiedlich! Alternativ
# kann auch der Hostname verwendet werden, wenn dieser im WLAN-
# Netz einmalig ist.
# Der Variablen wifi_ip unten korrekte IP-Adresse oder Hostnamen zuweisen. 
# Beim ASCII-Bild müssen bestimmte Zeichen mit \ "escaped" werden.
# S. Mack, 10.4.20


wifi_ip=beaglebone.local 
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
ssh $wifi_ip -l beagle -p 22

