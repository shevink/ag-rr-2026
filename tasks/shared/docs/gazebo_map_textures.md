# Gazebo-Map-Texturen

`ag_map_gazebo` lädt und wechselt Kartentexturen in Gazebo Harmonic. SVG-Karten werden als binäre Graustufen-PNGs erzeugt und als Ground-Plane-Textur verwendet.

Kartenübersicht und Generierungs-/Konvertierungsbefehle siehe [SVG-Map-Pipeline](svg_map_pipeline.md). Die PNGs liegen nach der Konvertierung unter `tasks/shared/maps/` und müssen als Texturen im `ag_map_gazebo`-Package verfügbar sein.

## Voraussetzungen

- ROS 2 Jazzy
- Gazebo Harmonic (`gz-sim8`)
- `ros_gz_sim`
- Python-Pakete `cairosvg` und `Pillow`

```bash
pip install cairosvg Pillow
```

## Nutzung

```bash
colcon build --packages-select ag_map_gazebo
source install/setup.bash

ros2 launch racer gazebo.launch.yaml
ros2 service call /load_map ag_map_gazebo/srv/LoadMap "{map_name: 'hard_map'}"
```

Gültige Namen: `easy_map`, `medium_map`, `hard_map`, `maze_map`.

Service:

```text
string map_name
---
bool success
string message
```

## Neue Karte

1. Generator in `tasks/shared/map_generation/` anlegen.
2. PNG erzeugen und in die Gazebo-Texturen kopieren.
3. Kartennamen in `src/ag_map_gazebo/src/map_switcher_node.cpp` registrieren.
4. Package neu bauen.
