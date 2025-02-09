# Anleitung für den Quellcode des Projekts Robotik FS3
### Projekt erstellt von Ronja Rid und Sebastian Kley

>[!IMPORTANT]
> Stellen Sie sicher das der Akku des Roboters auf mindestens 50% aufgeladen ist.
> Bei einer Akkuladung unter ca. 35% kann die Leistung des Differenzialantriebs
> evtl. nicht ausreichen.

### Starten des Roboters und Verbindungsaufbau
* Akku einschalten, mit Roboter verbinden und Schiebeschalter am Roboter betätigen, auf bestätigung durch Audio-Signal warten
* WLAN-Verbindung mit Roboter aufbauen (Network ID: ipek2)
  * in Konsole Verbindung aufbauen mit:
  
  ```bash
  ssh ubuntu@10.42.0.1
  ```
* Passwort eingeben
  `ipek2023`

### Node zur erkennung von AruCo-Markern auf Roboter starten

* Im mit Roboter verbundenem Terminal folgenden Befehl ausführen:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```  
* Neues Terminal öffnen, mit Roboter verbinden und folgenden Befehl ausführen:
```bash
python3 ArucoDistance.py
```

### Action Server auf Computer starten

* In ROS Workspace navigieren
>[!WARNING]
>Nicht bis in den src Ordner navigieren!

* Falls der Workspace noch nicht gebaut wurde in der Konsole `colcon build` ausführen
* Die Umgebung sourcen mit `source install/setup.bash`
* Den Server aktivieren mit:
  ```bash
  ros2 run ar_pipe_server PipeServer
  ```
>[!IMPORTANT]
>Die geöffneten Tabs in der Konsole nicht schließen während der Roboter gesteuert wird.

### Actions zum starten des Roboters senden

* Neue Konsole öffnen und die Umgebung sourcen `source install/setup.bash`
* Goal senden
  + velocity: Fahrgeschwindigkeit in m/s
  + distance: Abstand der zum anderen Roboter gehalten wird
  + mode: Fahrmodus der beim starten aktiviert wird
    + 0: Ruhemodus
    + 1: Drehen
    + 2: Fahren
  + Der Modus kann Standarmäßig auf 2 belassen werden da dies für die Funktion des Systems nicht relevant ist

```bash
ros2 action send_goal /velocity ar_interface/action/Mode "{velocity: 0.0, distance: 0.5, mode: 2}"
```


