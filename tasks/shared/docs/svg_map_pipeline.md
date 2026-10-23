# SVG-Map-Pipeline

Die SVG-Karten nutzen `viewBox`-Koordinaten: Ursprung oben links, x nach rechts, y nach unten. Eine `viewBox`-Einheit entspricht 100 Pixeln.

## Karten

| Datei | Größe | Beschreibung |
|---|---:|---|
| `easy_map.svg` | 8 x 5 | einfache Straßen, Kreuzungen, Parkbucht |
| `medium_map.svg` | 8 x 6 | geschwungene Fahrspuren |
| `hard_map.svg` | 10 x 10 | Kreisverkehr, Sternkreuzung |
| `maze_map.svg` | 8 x 8 | prozedurales Labyrinth mit Start/Ziel |

## Farben

- Schwarz (`0`): frei befahrbar
- Weiß (`255`): Hindernis

Die PNG-Ausgabe ist ein Graustufenbild (`L`) mit genau diesen zwei Pixelwerten.

## Generieren

```bash
python3 tasks/shared/map_generation/generate_easy.py --seed 42 --output tasks/shared/maps/easy_map.svg
python3 tasks/shared/map_generation/generate_medium.py --seed 42 --output tasks/shared/maps/medium_map.svg
python3 tasks/shared/map_generation/generate_hard.py --seed 42 --output tasks/shared/maps/hard_map.svg
python3 tasks/shared/map_generation/generate_maze.py --seed 42 --size 8 --crossing 0.15 --output tasks/shared/maps/maze_map.svg
```

## Konvertieren

```bash
python3 tasks/shared/map_generation/convert_to_png.py --input tasks/shared/maps/easy_map.svg
python3 tasks/shared/map_generation/convert_to_png.py --all --output-dir tasks/shared/maps
```
