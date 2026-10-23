import os
import tempfile


def test_svg_renderer_default_constructor():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    assert r.width == 800
    assert r.height == 600
    assert r.viewbox == '0 0 800 600'
    assert len(r.elements) == 1


def test_svg_renderer_custom_constructor():
    from svg_renderer import SVGRenderer
    r = SVGRenderer(width=400, height=300, viewbox='0 0 400 300')
    assert r.width == 400
    assert r.height == 300


def test_add_rect_creates_rect_element():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    r.add_rect(x=10, y=20, w=100, h=50, fill='#ff0000')
    assert len(r.elements) == 2
    assert r.elements[1].tag == 'rect'
    assert r.elements[1].attrib.get('fill') == '#ff0000'


def test_add_polygon_creates_path_with_z():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    points = [(0, 0), (10, 0), (10, 10), (0, 10)]
    r.add_polygon(points)
    assert len(r.elements) == 2
    path_elem = r.elements[1]
    assert path_elem.tag == 'path'
    d = path_elem.attrib.get('d', '')
    assert d.endswith(' Z')
    assert 'M' in d
    assert 'L' in d


def test_add_polygon_default_colors():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    r.add_polygon([(0, 0), (1, 0), (1, 1)])
    path_elem = r.elements[1]
    assert path_elem.attrib.get('fill') == '#0a0a0a'
    assert path_elem.attrib.get('stroke') == '#ffffff'


def test_polygon_custom_fill_stroke():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    r.add_polygon([(0, 0), (1, 0), (1, 1)],
                  fill='#ff0000', stroke='#00ff00', stroke_width=2.0)
    path_elem = r.elements[1]
    assert path_elem.attrib.get('fill') == '#ff0000'
    assert path_elem.attrib.get('stroke') == '#00ff00'


def test_background_rect_default():
    from svg_renderer import SVGRenderer, BACKGROUND
    r = SVGRenderer()
    bg = r.elements[0]
    assert bg.tag == 'rect'
    assert bg.attrib.get('fill') == BACKGROUND


def test_save_produces_valid_svg():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    r.add_polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    with tempfile.NamedTemporaryFile(suffix='.svg', delete=False,
                                     mode='w') as f:
        r.save(f.name)
        f.flush()
        with open(f.name, 'r') as f2:
            content = f2.read()
            assert '<?xml' in content
            assert '<svg' in content
            assert 'xmlns="http://www.w3.org/2000/svg"' in content
            assert '</svg>' in content
        os.unlink(f.name)


def test_multiple_polygons():
    from svg_renderer import SVGRenderer
    r = SVGRenderer()
    r.add_polygon([(0, 0), (1, 0), (1, 1)])
    r.add_polygon([(2, 2), (3, 2), (3, 3)])
    assert len(r.elements) == 3


def test_color_constants_correct():
    from svg_renderer import OBSTACLE_FILL, OBSTACLE_STROKE, BACKGROUND
    assert OBSTACLE_FILL == '#0a0a0a'
    assert OBSTACLE_STROKE == '#ffffff'
    assert BACKGROUND == '#000000'
