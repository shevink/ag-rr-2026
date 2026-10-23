# RR: Rechteckroboter - Navigation mit Rotation

## Beschreibung

Ihr steuer ein rechteckiges Fahrzeug auf einer euch bekannten Karte, welche Hindernisse enthält mit denen ihr nicht kollidieren dürft. Es ist nicht garantiert dass ihr durch jede Engstelle zwischen den Hindernissen passt. Einige Wege sind nur befahrbar, wenn ihr euren Roboter vorher korrekt ausgerichtet habt. Eure eigene Position ist euch jederzeit bekannt. Die Aufgabe besteht darin von eurer aktuellen Position aus zu einer euch vorgegeben Postion auf der Karte zu navigieren.

## Technische Spezifikation

### Kartenformat

Die Karte wird als SVG-Datei bereitgestellt. Sie enthält die Hindernisse mit ihren Koordinaten und Abmessungen und kann gleichzeitig als Untergrund in der Simulation verwendet werden. So ist es auch euch leicht möglich eigene Karten (zum Testen spezieller Sonderfälle) zu entwerfen und in die Simulation einzubinden.

Relevante Links:
- [SVG-Map-Pipeline](../shared/docs/svg_map_pipeline.md)
- [Gazebo-Map-Texturen](../shared/docs/gazebo_map_textures.md)
- [Bereitgestellte Karten](../shared/maps/)

### Topics

Eingaben sind euch bereitgestellte Topics. Ob ihr sie abonniert bleibt euch überlassen. Ausgaben errechnet ihr selbst und publiziert sie auf das entsprechende Topic. Die Benutzung dieser Topics ist verpflichtend. Das publizieren auf / abonnieren von Topics mit dem Präfix `/internal` (z. B. `/internal/racer/cmd_vel`) ist maximal zu Debugzwecken erlaubt. Das Nutzen dieser in der finalen Node bedarf vorheriger Absprache, das dies ggf. die Schwierigkeit der Aufgabe beinflusst. Das Verstecken solcher Topics ist uns technisch nicht möglich.

#### Eingaben
- `/goal` liefert die Position zu der Ihr navigieren sollt
	- jedes neue Ziel überschreibt sofort seinen Vorgänger
	- Frequenz: unbestimmt, 1x pro neuem Ziel
- `/vehicle/position` liefert euch eure eigene Postion und Ausrichtung relativ zur globalen Karte
	- Frequenz: 50Hz

#### Ausgaben
- `/planning/path` erwartet eine Polyline relativ zur globalen Karte

### Launchfile
Startet für eure Aufgabe eine der Launch-Dateien unter `launch/`:
- `without_obstacles.launch.py`
- `with_obstacles.launch.py`

## Zusatz
Siehe [Unbekannte Hindernisse](../shared/docs/obstacles.md).
