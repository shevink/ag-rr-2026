# Prozedurale Lanelet2-Kartenerstellung

Ziel: Ein Generator erzeugt eine gültige Lanelet2-OSM-Datei (`.osm`), die validiert und in ROS 2 geladen werden kann.

## Ablauf

1. Straßennetz planen.
2. Geometrie in lokalen Metern erzeugen.
3. Lokale Koordinaten nach WGS84 umrechnen.
4. Nodes, Ways und Relations in OSM-XML schreiben.
5. Datei validieren.

## Koordinaten

Intern sollte der Generator in einem lokalen kartesischen System arbeiten. Beim Export:

```text
lat = origin_lat + y / 111320.0
lon = origin_lon + x / (111320.0 * cos(origin_lat))
```

Der Ursprung muss nahe am Kartenmittelpunkt liegen und beim Laden wiederverwendet werden.

## Struktur und Tags

Primitive, XML-Struktur, Pflichtfelder und Tag-Konventionen siehe [Lanelet2-Format](lanelet_format.md).

Generator-spezifisch:

- Übliche Fahrspurbreite: ca. 3,5 m.

## Häufige Generatorfehler

| Problem | Ursache |
|---|---|
| Lanelet hat keine Fläche | Grenzen überlappen oder zeigen gegeneinander |
| Lanelets sind nicht verbunden | fehlende gemeinsame Endpunkte zwischen Segmenten |
| Way/Relation referenziert unbekannte ID | Reihenfolge der Ausgabe oder verworfene Elemente |

Lade-, Validierungs- und Projektionsfehler siehe [Validierung](validation_guide.md) und [ROS 2 / Gazebo](Ros2_gazebo_integration.md).

## Siehe auch

- [Lanelet2-Format](lanelet_format.md)
- [Validierung](validation_guide.md)
- [ROS 2 / Gazebo](Ros2_gazebo_integration.md)
- [Ressourcen](lanelet_resources.md)
