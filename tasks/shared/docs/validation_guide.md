# Lanelet2-Karten validieren

Empfohlener Ablauf:

1. Karte in JOSM visuell prüfen.
2. Karte mit Lanelet2 laden.
3. Fehler beheben und neu generieren.

## JOSM

```bash
sudo apt install josm
```

Lanelet2-Styles in JOSM unter `Bearbeiten -> Einstellungen -> Karteneinstellungen -> Kartenzeichnungsstile` eintragen:

- `lanelet2_maps/josm/lines.mapcss`
- `lanelet2_maps/josm/lanelets.mapcss`

Optional unter `Tagging-Voreinstellungen`:

- `lanelet2_maps/josm/laneletpresets.xml`

Prüfen:

- Werden Lanelets als Flächen angezeigt?
- Sind Begrenzungen sichtbar und korrekt gestrichelt/durchgezogen?
- Gibt es Lücken, Überschneidungen oder invertierte Lanelets?
- Meldet JOSM Warnungen oder Fehler?

## Python-Ladeprüfung

Karte wie in [ROS 2 / Gazebo](Ros2_gazebo_integration.md) laden und anschließend prüfen, ob Layer befüllt sind:

```python
print(len(lanelet_map.laneletLayer))
```

Falls Lanelet2 im Container fehlt:

```bash
sudo apt install ros-jazzy-lanelet2*
```

## Typische Fehler

| Meldung/Symptom | Ursache |
|---|---|
| XML-Parsefehler | ungültiges XML |
| `Element with id ... not found` | Referenz auf nicht vorhandene ID |
| `Left and right bounds are not aligned` | Grenzen zeigen nicht in dieselbe Richtung |
| `Linestring self-intersects` | Way schneidet sich selbst |
| `No lanelets found` | `type=lanelet` fehlt |

## Optional

```python
from lanelet2.validation import ValidityChecker

for issue in ValidityChecker().checkMap(lanelet_map):
    print(issue)
```
