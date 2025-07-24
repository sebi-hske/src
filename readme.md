# Anleitung zur Inbetriebnahme der Sortieranlage im Rahmen des Robotik Projekt 3 FS4
#### Projekt erstellt von Ronja Rid, Sebastian Kley, Nicholas Link

>[!IMPORTANT]
>Stellen Sie sicher das die Stromversorgung der Portalachse eingeschaltet ist, das USB-Kabel zum Verbinden mit den Mikrocontrollern jedoch ausgesteckt ist.
Auch sollten auf dem zur Steuerung angeschlossenen PC alle offenen Terminals beendet werden. Andernfalls kann es zum unerwarteten Verfahren der Achse kommen.

### Verbindungsaufbau mit den Mikrocontrollern der Portalachse
* USB-Kabel mit PC verbinden
* In neuem Terminal die Umgebung sourcen
* Das Startkommando ausführen:
``ros2 launch ro45_ros2_pickrobot_serial launch_nodes.py``

>[!IMPORTANT]
>Die geöffneten Tabs in der Konsole nicht schließen während die Achse gesteuert wird.

### Bauen der Pakete
* Terminal öffnen und in ROS workspace navigieren
>[!WARNING]
>Nicht bis in den `/src` Ordner navigieren!
* Falls der Workspace noch nicht gebaut wurde in der Konsole `colcon build` ausführen
* Die Umgebung sourcen mit `source install/setup.bash`

### Starten der Sortieranlage
* Die Node `central_control.py` starten und auf das beenden der Kalibrierung und das Anfahren der Startposition warten
* In einem neuem Terminal (Umgebung sourcen!) die Node `object_tracker.py` starten und den angezeigten Bildauschnitt kontrollieren
* Bei zufriedenstellendem Zuschneiden mit Taste "Q" bestätigen, andernfalls mit Taste "E" abbrechen
* Das Fließband mit dem Drehregler aktivieren und die Geschwindigkeit zwischen den beiden Strichen einstellen
* Objekte mit ausreichend Abstand auf das FLießband legen