# Lanelet2-OSM-Format

Lanelet2 nutzt OSM-XML. Für dieses Projekt reichen diese Primitive:

| Lanelet2 | OSM | Zweck |
|---|---|---|
| Point | `<node>` | Punkt mit `lat`, `lon`, optional `ele` |
| Linestring | `<way>` | geordnete Punktfolge |
| Lanelet | `<relation type=lanelet>` | befahrbares Spursegment |
| RegulatoryElement | `<relation type=regulatory_element>` | Verkehrsregel |

## Aufbau

```xml
<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="lanelet2">
  <bounds minlat="..." minlon="..." maxlat="..." maxlon="..."/>
  <!-- nodes -->
  <!-- ways -->
  <!-- relations -->
</osm>
```

Reihenfolge: Nodes vor Ways, Ways vor Relations.

## Nodes

```xml
<node id="1" version="1" lat="51.123456" lon="12.654321">
  <tag k="ele" v="0.0"/>
</node>
```

Pflicht:

- eindeutige positive `id`
- `version="1"`
- WGS84-Koordinaten

## Ways

```xml
<way id="100" version="1">
  <nd ref="1"/>
  <nd ref="2"/>
  <tag k="type" v="line_thin"/>
  <tag k="subtype" v="solid"/>
</way>
```

Regeln:

- mindestens zwei Nodes
- keine direkt doppelten Node-Referenzen
- keine Selbstüberschneidung
- Boundary-Ways brauchen ein `type`-Tag
- Richtung ist für Lanelet-Grenzen relevant

Häufige Tags:

| Tags | Bedeutung |
|---|---|
| `type=line_thin`, `subtype=solid` | durchgezogene Linie |
| `type=line_thin`, `subtype=dashed` | gestrichelte Linie |
| `type=curbstone`, `subtype=high` | Bordstein |
| `type=road_border` | Rand der befahrbaren Fläche |
| `type=virtual` | virtuelle Grenze |
| `type=stop_line` | Haltelinie |
| `type=traffic_sign` | Verkehrszeichen |
| `type=traffic_light` | Ampel |

## Lanelets

```xml
<relation id="200" version="1">
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="location" v="urban"/>
  <member type="way" ref="100" role="left"/>
  <member type="way" ref="101" role="right"/>
</relation>
```

Pflicht:

- `type=lanelet`
- `left`-Way
- `right`-Way

Optional:

- `centerline`
- `regulatory_element`
- `speed_limit`, z. B. `30 km/h`
- `participant:*`, z. B. `participant:vehicle=yes`

`left` und `right` müssen in Fahrtrichtung zeigen. Der befahrbare Bereich liegt zwischen ihnen.

## Regulatory Elements

```xml
<relation id="300" version="1">
  <tag k="type" v="regulatory_element"/>
  <tag k="subtype" v="speed_limit"/>
  <tag k="sign_type" v="50 km/h"/>
  <member type="relation" ref="200" role="referring_relations"/>
</relation>
```

Häufige Subtypen:

| Subtyp | Zweck |
|---|---|
| `speed_limit` | Geschwindigkeitsbegrenzung |
| `traffic_light` | Ampel |
| `traffic_sign` | Verkehrszeichen |
| `right_of_way` | Vorfahrt |
| `all_way_stop` | Stopps an allen Zufahrten |

## Checkliste

- Alle IDs sind eindeutig und positiv.
- Jedes Primitive hat `version="1"`.
- Alle Referenzen zeigen auf existierende Elemente.
- Lanelet-Grenzen zeigen in dieselbe Richtung.
- Linestrings schneiden sich nicht selbst.
- Geschwindigkeit und Teilnehmer sind ableitbar oder explizit gesetzt.
