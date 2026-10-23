# ROS 2 / Gazebo mit Lanelet2-Karten

Eine generierte `.osm`-Datei wird mit `lanelet2_io` geladen und danach für Visualisierung, Routing oder Regelinterpretation genutzt.

## Laden

```python
from lanelet2.core import Origin
from lanelet2.io import load
from lanelet2.projection import UtmProjector

projector = UtmProjector(Origin(51.34, 12.38))
lanelet_map = load("map.osm", projector)
```

Der Ursprung muss zu den Kartenkoordinaten passen. Nutzt beim Generieren und Laden denselben Ursprung.

## RViz

Lanelet-Grenzen können als `visualization_msgs/MarkerArray` publiziert werden:

```python
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

def marker(points, marker_id, rgb):
    msg = Marker()
    msg.header.frame_id = "map"
    msg.id = marker_id
    msg.type = Marker.LINE_STRIP
    msg.action = Marker.ADD
    msg.scale.x = 0.1
    msg.color.a = 1.0
    msg.color.r, msg.color.g, msg.color.b = rgb
    msg.points = [Point(x=p.x, y=p.y, z=p.z) for p in points]
    return msg

def lanelet_markers(lanelet_map):
    markers = MarkerArray()
    marker_id = 0
    for lanelet in lanelet_map.laneletLayer:
        markers.markers.append(marker(lanelet.leftBound, marker_id, (1.0, 0.0, 0.0)))
        marker_id += 1
        markers.markers.append(marker(lanelet.rightBound, marker_id, (0.0, 1.0, 0.0)))
        marker_id += 1
        markers.markers.append(marker(lanelet.centerline, marker_id, (0.0, 0.0, 1.0)))
        marker_id += 1
    return markers
```

## Fehler

| Problem | Lösung |
|---|---|
| `No suitable projection found` | Ursprung näher an die Karte legen |
| Karte liegt in RViz weit entfernt | Ursprung zwischen Generator und Loader angleichen |
| Leere Lanelet-Ebene | Datei validieren und Pfad prüfen |
| Routing findet keinen Pfad | Lanelets geometrisch verbinden |

## Links

- [Lanelet2 IO](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_io)
- [Lanelet2 Projection](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_projection)
- [Lanelet2 Routing](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_routing)
