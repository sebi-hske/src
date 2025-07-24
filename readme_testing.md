# Anleitung zum Testen wichtiger Funktionen

>[!IMPORTANT]
>Stellen Sie sicher das die Stromversorgung der Portalachse eingeschaltet ist, das USB-Kabel zum Verbinden mit den Mikrocontrollern jedoch ausgesteckt ist.
Auch sollten auf dem zur Steuerung angeschlossenen PC alle offenen Terminals beendet werden. Andernfalls kann es zum unerwarteten Verfahren der Achse kommen.

### Bauen der Pakete und Sourcen der Umgebung wie im Basis readme beschrieben

### Testen der Objekterkennung und Kassifizierung 
* USB-Kabel der Kamera mit PC verbinden
* In der Datei `object_tracker.py` stehen auskommentierte Funktionen bereit um etwa die extrahierten Objekte anzuzeigen
* Node `object_tracker.py` starten
* Anweisungen auf Bildschirm folgen
* Nach Bestätigung des Bildasuchnitts können Objekte aufgelegt werden
* Erkennung der Objekte sowie Richtige Klassifizierung prüfen (Output auf Konsole)

### Testen des Ablaufs der Steuerung
* Node `/input_output_testing/integrator_sim.py` starten
* Node `central_control.py` starten
* Auf Ende der Kalibrierung warten
* Nun können über ein neues Terminal Action Messages der Typen "intercept_object" sowie "move_to_pos" gesendet werden

### Testen des Reglers am Aufbau
#### Verbindungsaufbau mit den Mikrocontrollern der Portalachse wie in Basis readme beschrieben
#### Starten der Sortieranlage wie in Basis readme beschrieben
* Nach erreichen der Startposition können über ein neues Terminal Action Messages der Typen "intercept_object" sowie "move_to_pos" gesendet werden

### Testen der Topics
* Über RQT können die Topics /robot_command und /robot_position beobachtet werden um die Funktion der Publisher zu verifizieren