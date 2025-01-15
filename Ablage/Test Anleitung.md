# Anleitung zum Testen einzelner Funktionen

### Erkennung von AruCo-Markern und senden der extrahierten Daten

* Wie in der generellen Anleitung beschrieben:
    + Roboter starten
    + Über SSH mit Roboter verbinden
    + ArucoDistance.py auf Roboter starten
  
* Auf lokaler Maschine in `ros2_ws/src/ar_pipe_server/test` navigieren
* Node starten um auf Topic der Daten vom Roboter zuzugreifen
  ```bash
  python3 aruco_listener.py
  ```
* Im Terminal werden ID, x-Offset und Distanz zu erkannten Markern angezeigt
>[!NOTE]
>Der x-Offset ist hier immer ein negativer Wert aufgrund der Art wie OpenCV 
>die Lage des Markers im Bild berechnet. Mehr Infos dazu in der OpenCV Dokumentation.

### Abrufen der Odometry Daten des Roboters

* Wie in der generellen Anleitung beschrieben:
    + Roboter starten
    + Über SSH mit Roboter verbinden
    + Turtlebot Node starten
* Auf lokaler Maschine in `ros2_ws/src/ar_pipe_server/test` navigieren
* Node starten um auf Topic der Daten vom Roboter zuzugreifen
  ```bash
  python3 odometry_listener.py
  ```
* Im Terminal wird die aktuelle Ausrichtung des Roboters angezeigt
>[!NOTE]
>Die Werte der Ausrichtung bewegen sich zwischen 0.0 und 3.14 bzw. -3.14

### Bewegungskommandos ablesen

* Wie in der generellen Anleitung beschrieben:
    + Roboter starten
    + Über SSH mit Roboter verbinden
    + ArucoDistance.py auf Roboter starten
>[!CAUTION]
>Die Turtlebot Node nicht starten, der Roboter soll keine Bewegungskommandos empfangen.

* PipeServer wie in genereller Anleitung beschrieben starten.
* Goal senden mit `mode: 2` oder `mode: 1`
* Werden vom Roboter Marker erkannt, werden die korrespondierenden Bewegungsbefehle auf dem Terminal augegeben