# Projekt Sensor-/Regelungssysteme mit BeagleBone, Ubuntu, ROS und Simulink - Master Mechatronik, Hochschule Reutlingen

*Embedded Linux auf einem BeagleBone unter ROS mit Anwendung durch ein Roboterfahrzeug.*

Im Sensorsystemeprojekt erhält jede Studentengruppe ein mit einem BeagleBone-Platinencomputer ausgestattetes Roboterfahrzeug. Dieses Fahrzeug soll autonom einen Raum absuchen, Kollisionen vermeiden, Hindernissen ausweichen und einen vorgegebenen Gegenstand auf dem Boden identifizieren. Dazu muss das Fahrzeug mit einer Leistungselektronik für die Antriebe und mit geeigneten Sensoren ausgestattet werden.  
Da für die Messung des Wandabstands nur ein einzelner Ultraschallsensor verwendet werden darf, muss dessen Signal mit einem Kalman-Filter aufbereitet werden.   
Sowohl auf dem BeagleBone aus auch auf dem PC für die Softwareentwicklung befindet sich ein Ubuntu 18.04 LTS Betriebssystem inklusive der Middleware ROS.
   
![Kommunikationsstruktur](BB_ROS_Comm.png)  
 
Das Roboterfahrzeug als mechatronisches System besitzt die Komponenten Sensordatenerfassung, Motorsteuerung und Kalman-Filter / Regelung. Jeder dieser Komponenten ist ein ROS-Knoten also ein Prozess zugeordnet.
Dabei werden die Knoten Sensordatenerfassung und Motorsteuerung auf dem BB ausgeführt. Der Knoten Kalman-Filter / Regelung wird mit Simulink auf dem PC erstellt und dort auch ausgeführt.
Wie in der Grafik dargestellt liefert der Sensorknoten Abstandsmesswerte (Topic A) an den
Filter-/Regelungsknoten. Dieser wiederum liefert Solldrehzahlen der Räder (Topic B) an die Motorsteuerung.

![Softwarestruktur ROS](BB_ROS_Struc.png)

In diesem Repository finden Sie die Projektanleitung als PDF-Datei. Darin werden die BeagleBone-Hardware sowie die Schnittstellen zu den Peripheriemodulen (Sensoren und Aktoren) eingehend behandelt. Auch finden sich dort Anleitungen zur Softwareerstellung mit C, Python und MATLAB/Simulink sowie Basisinformationen zum Umgang mit Embedded Linux.
Weiter sind hier als Beispiele Quellcodedateien für C-, Python-, und SIMULINK-Programme sowie für ein ROS-Package zu finden.
  
Das verwendete Image für den BeagleBone finden sie hier als Download.

Bisher hatten wir diese Projektarbeit ohne ROS mit modellbasierter Softwareerzeugung und Crosscompilern unter Simulink durchgeführt. Leider hat uns Mathworks hier etwas im Stich gelassen, da das Support Package für den BeagleBone nicht für eine neuere Debianversion als 7.9 aktualisiert wurde. Im Zuge der Umstellung der Labor-PCs auf Ubuntu-Linux konnte das Support Package für den BeagleBone dann gar nicht mehr verwendet werden, da es seltsamerweise nur für Windows verfügbar ist.
Die Not macht erfinderisch: Das Projekt wurde auf Ubuntu 18.04 und ROS Melodic umgestellt. Simulink wird nun nur noch dazu verwendet, einen ROS-Knoten für die Regelung zu erstellen. Insgesamt funktioniert die Projektarbeit nun wensentlich besser und die Studierenden sind zufriedener!

Prof. Dr. S. Mack, 6. März 2020.
