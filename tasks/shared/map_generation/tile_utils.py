import json


def load_tile_base_walls(tileset_path):
    """Load tile base walls from a tileset JSON file.

    Returns a dict mapping tile name to list of wall coordinate lists.
    Tiles with empty wall lists are skipped.
    """
    with open(tileset_path) as f:
        data = json.load(f)
    tile_walls = {}
    for tile in data:
        name = tile['name']
        walls = tile['walls']
        if not walls:
            continue
        if name not in tile_walls:
            tile_walls[name] = walls
    return tile_walls
