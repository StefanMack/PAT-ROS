#!/bin/bash
# Bash Skript, damit beim BBG nach dem Booten das WLAN aktiviert wird.
# Funktioniert wohl nur, wenn als crontab nicht nur unter dem User beagle
# sondern allgemein als root ausgeführt.
# S. Mack, 10.4.20

# USR3-LED (sonst fuer eMMC) ausschalten
echo none > /sys/class/leds/beaglebone\:green\:usr3/trigger

# check if iwgetid command returns a wireless connection e.g. wlan0
# command output is piped to Nirvana (>/dev/null)
iwgetid >/dev/null
# check return value $? of iwgetid command
if [ $? != 0 ]
then
    # Restart the wireless interface (disable necessary but causess error
    # message that ist piped to Nirvana (2>/dev/null)
    connmanctl disable wifi 2>/dev/null
    connmanctl enable wifi
else
   # USR3-LED (sonst fuer eMMC) blinken lassen
   echo timer > /sys/class/leds/beaglebone\:green\:usr3/trigger
fi
