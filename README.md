# Projekt Sensor-/Regelungssysteme mit BeagleBone, Ubuntu, ROS und Simulink - Master Mechatronik, Hochschule Reutlingen

*Projektarbeit: Embedded Linux auf einem BeagleBone unter ROS, Filter- und Regelungsentwicklung unter Simulink zur Steuerung eines autonomen Roboterfahrzeugs.*

Im Sensorsystemeprojekt erhält jede Studentengruppe ein mit einem BeagleBone-Platinencomputer ausgestattetes Roboterfahrzeug. Dieses Fahrzeug soll autonom einen Raum entlang der Wand absuchen, Kollisionen vermeiden, Hindernissen ausweichen und einen vorgegebenen Gegenstand auf dem Boden identifizieren. Dazu muss das Fahrzeug mit einer Leistungselektronik für die Antriebe und mit geeigneten Sensoren ausgestattet werden.  
Da für die Messung des Wandabstands nur ein einzelner Ultraschallsensor verwendet werden darf, muss dessen Signal mit einem Kalman-Filter aufbereitet werden.   
Die Softwareentwicklung sowohl auf dem BeagleBone aus auch auf dem PC geschieht unter Linux (Ubuntu 18.04 LTS) inklusive der Middleware ROS (Robot Operation System Melodic).
   
![Kommunikationsstruktur](/BB_ROS_Comm) 
 
**Das Roboterfahrzeug als mechatronisches System besitzt die Komponenten Sensordatenerfassung, Motorsteuerung und Kalman-Filter / Regelung. Jeder dieser Komponenten ist ein ROS-Knoten also ein Prozess zugeordnet.
Dabei werden die Knoten Sensordatenerfassung und Motorsteuerung auf dem BeagleBone ausgeführt. Der Knoten Kalman-Filter / Regelung wird mit Simulink auf dem PC erstellt und dort auch ausgeführt. Der BeagleBone und der PC befinden sich im selben WLAN und bilden ein ROS-Netzwerk.  
Wie in der Grafik dargestellt liefert der Sensorknoten beispielsweise Abstandsmesswerte (Topic A) an den
Filter-/Regelungsknoten. Dieser wiederum liefert Solldrehzahlen der Räder (Topic B) an die Motorsteuerung, damit das Roboterfahrzeug im konstanten Abstand entlang der Wand fährt.**

![Softwarestruktur ROS](/BB_ROS_Struc)

In diesem Repository finden Sie die Projektanleitung als PDF-Datei. Darin werden die BeagleBone-Hardware sowie die Schnittstellen zu den Peripheriemodulen (Sensoren und Aktoren) eingehend behandelt. Auch finden sich dort Anleitungen zur Softwareerstellung mit C, Python und Simulink sowie Basisinformationen zum Umgang mit Embedded Linux.
Weiter sind hier als Beispiele Quellcodedateien für C-, Python-, und Simulink-Programme sowie für ein ROS-Package zu finden.
  
Das verwendete SRP-Image (Stand 14.2.20) für den BeagleBone finden sie [hier als Download](https://www.magentacloud.de/lnk/zKiMtt0m).

Bisher wurde diese Projektarbeit ohne ROS mit modellbasierter Softwareerzeugung und Cross-Compiler komplett unter Simulink durchgeführt. Leider hatte uns Mathworks hier etwas im Stich gelassen, da das Support Package für den BeagleBone nicht für neuere Debianversionen als 7.9 aktualisiert wurde. Im Zuge der Umstellung der Labor-PCs auf Ubuntu-Linux konnte das Support Package für den BeagleBone dann gar nicht mehr verwendet werden, da es seltsamerweise nur für Windows verfügbar ist.  
Die Not macht erfinderisch: Das Projekt wurde auf Ubuntu 18.04 und ROS Melodic umgestellt. Simulink wird nun nur noch dazu verwendet, einen ROS-Knoten für die Regelung und den Kalman-Filter zu erstellen.  
Insgesamt funktioniert die Projektarbeit nun wesentlich besser und die Studierenden sind zufriedener!

Wegen der vorrübergehenden Schließung der Hochschulen infolge der Corona-Pandemie wurde eine "to-go Version" des Roboters entwickelt. Damit können die Studierenden die Projektarbeit auch Zuhause durchführen.

![Neuer RoboToGo für die Projektarbeit Zuhause](/RoboToGo)

Prof. Dr. S. Mack, 12. April 2020.

License
-----
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons Lizenzvertrag" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />Dieses Werk ist lizenziert unter einer <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Namensnennung - Weitergabe unter gleichen Bedingungen 4.0 International Lizenz</a>.
