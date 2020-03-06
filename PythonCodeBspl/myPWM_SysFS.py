#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Programm zum Testen eines Modellbauservos an Pin P9_14
# Achtung: Je nach Kernel kann dieser PWM-Pin unterschiedlichen pwmchips
# zugeordnet sein. Beim Ubuntu-ROS-Melodic Kernel ist es pwmchip4.
# Siehe:
# stackoverflow.com/questions/50204329/pwm-chip-to-pin-mapping-on-beaglebone-black-v4-14
# Der Pin muss via config-pin auf PWM konfiguriert werden. 
# Funktioniert mit Micor Servo BM-306: 50 Hz, 3 bzw. 12 % PWM für die beiden 
# Extremstellungen
# S. Mack, 5.3.20


from subprocess import call 
import sys
import time

print("Python-Interpreter: {}\n".format(sys.version))

pin = "P9_14"
command = "config-pin -a " + pin + " pwm"
call(command, shell= True) # execute command in Bash Shell

setup = open('/sys/class/pwm/pwmchip4/pwm-4:0/period', 'a')
setup.write('20000000') # ns, equals 50 Hz frequency
setup.flush()
setup.close()
setup = open('/sys/class/pwm/pwmchip4/pwm-4:0/duty_cycle', 'a')
setup.write('1200000') # 3 % duty-cycle in ns
setup.flush()
setup.close()
setup = open('/sys/class/pwm/pwmchip4/pwm-4:0/enable', 'a')
setup.write('1') # enable pwm output
setup.flush()
setup.close()
print('PWM Output on Pin P9_14 enabled...')

setup = open('/sys/class/pwm/pwmchip4/pwm-4:0/duty_cycle', 'a')
try:
    while True:
        time.sleep(1)
        setup.write('4800000') # 12 % duty-cycle in ns
        setup.flush()
        time.sleep(1)
        setup.write('1200000') # 3 % duty-cycle in ns
        setup.flush()
#Falls Strg+c gedrueckt wird
except KeyboardInterrupt:
    setup.close()
    setup = open('/sys/class/pwm/pwmchip4/pwm-4:0/enable', 'a')
    setup.write('0') # disnable pwm output
    setup.flush()
    setup.close()
    print("")
    print("...PMW-Output disabled. Byebye...")
