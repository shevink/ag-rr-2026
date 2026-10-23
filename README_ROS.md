# Algorithmische Geometrie SoSe 2026

Projekt-Repository für die Aufgaben des Moduls Algorithmische Geometrie in Kooperation mit dem HTWK Smart Driving Team.

## Aufgaben

- [Kreuzungserkennung](tasks/ke_kreuzungserkennung/aufgabe.md)
- [Lanelet-Kartengenerierung](tasks/lkg_lanelet-kartengenerierung/aufgabe.md)
- [Lanelet-Parken](tasks/lp_lanelet-parken/aufgabe.md)
- [Navigation mit rechteckigem Roboter](tasks/rr_rechteckroboter/aufgabe.md)
- [Navigation mit rundem Roboter](tasks/sr_saugroboter/aufgabe.md)

## Docker

Voraussetzungen:

- Docker
- Docker Compose

Build und Start, Beispiel Linux:

```bash
docker compose --profile ag-linux build
docker compose --profile ag-linux up -d
docker exec -it ag-linux bash
```

Auf Linux vor dem Start von GUI-Anwendungen:

```bash
xhost +local:docker
```

Profile:

| Profil | Zweck |
|---|---|
| `ag-linux` | Linux mit X11-GUI |
| `ag-linux-nvidia` | Linux mit NVIDIA-GPU |
| `ag-windows` | Windows/WSLg oder externer X-Server |
| `ag-vnc` | GUI im Browser unter `http://localhost:6080` |
| `ag-debug` | Minimaler ROS-Container |

Stoppen:

```bash
docker compose --profile ag-linux down
```

## Devcontainer

Das Repository enthält einen Devcontainer. Ein Code Editor wie VSCode mit Dev Containers Extension oder Zed kann das Projekt damit direkt im Container öffnen. So kann ohne lokale Installation von Dependencies, wie ROS 2, gearbeitet werden.

## ROS

Im Container:

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch racer gazebo.launch.yaml
ros2 launch racer display.launch.yaml
```
