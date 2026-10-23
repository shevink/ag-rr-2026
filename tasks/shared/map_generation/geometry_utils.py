import math


def rotate_walls(walls, rot_90):
    result = []
    angle = rot_90 * (math.pi / 2.0)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    for wall in walls:
        rotated = []
        for x, y in wall:
            centered_x = x - 0.5
            centered_y = y - 0.5
            x_new = cos_a * centered_x - sin_a * centered_y
            y_new = sin_a * centered_x + cos_a * centered_y
            rotated.append((x_new + 0.5, y_new + 0.5))
        result.append(rotated)
    return result


def transform(coords, dx, dy, angle_rad):
    result = []
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    for x, y in coords:
        x_new = cos_a * x - sin_a * y + dx
        y_new = sin_a * x + cos_a * y + dy
        result.append((x_new, y_new))
    return result


def approximate_curve(cps, segments):
    result = []
    n = len(cps) - 1
    for i in range(segments + 1):
        t = i / segments
        x, y = 0.0, 0.0
        for j, (cx, cy) in enumerate(cps):
            coeff = math.comb(n, j) * (t ** j) * ((1 - t) ** (n - j))
            x += coeff * cx
            y += coeff * cy
        result.append((x, y))
    return result


def create_star_intersection(center, n_roads, radius):
    cx, cy = center
    polygons = []
    inner_radius = radius * 0.3
    outer_poly = []
    for i in range(n_roads * 2):
        angle = (math.pi * i) / n_roads
        r = radius if i % 2 == 0 else inner_radius
        outer_poly.append((cx + r * math.cos(angle),
                           cy + r * math.sin(angle)))
    polygons.append(outer_poly)
    for i in range(n_roads):
        angle = (2 * math.pi * i) / n_roads
        spoke = [
            (cx + inner_radius * math.cos(angle),
             cy + inner_radius * math.sin(angle)),
            (cx + radius * 1.5 * math.cos(angle - 0.15),
             cy + radius * 1.5 * math.sin(angle - 0.15)),
            (cx + radius * 1.5 * math.cos(angle + 0.15),
             cy + radius * 1.5 * math.sin(angle + 0.15)),
        ]
        polygons.append(spoke)
    return polygons


def create_roundabout(center, inner_r, outer_r, n):
    cx, cy = center
    outer_ring = []
    inner_ring = []
    for i in range(n):
        angle = (2 * math.pi * i) / n
        outer_ring.append((cx + outer_r * math.cos(angle),
                           cy + outer_r * math.sin(angle)))
        inner_ring.append((cx + inner_r * math.cos(angle),
                           cy + inner_r * math.sin(angle)))
    return [outer_ring, inner_ring]
