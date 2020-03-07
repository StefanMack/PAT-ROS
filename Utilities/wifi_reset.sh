#!/bin/bash
# Bash Skript, damit beim BBG nach dem Booten das WLAN aktiviert wird.
# Funktioniert wohl nur, wenn als crontab nicht nur unter dem User beagle
# sondern allgemein als root ausgeführt.
# S. Mack, 14.12.19

# check if iwgetid command returns a wireless connection e.g. wlan0
# command output is piped to Nirvana (>/dev/null)
iwgetid >/dev/null
# check return value $? of iwgetid command
if [ $? != 0 ]
then
    # Restart the wireless interface (disable necessary but causess error
    # message that ist piped to Nirmana (2>/dev/null)
    connmanctl disable wifi 2>/dev/null
    connmanctl enable wifi
fi
