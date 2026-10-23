import os
import tempfile


def _gen_medium_svg(seed=42):
    from generate_medium import generate_medium_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    generate_medium_map(seed=seed, output_path=f.name)
    f.flush()
    content = open(f.name).read()
    os.unlink(f.name)
    return content


def test_generate_medium_creates_svg():
    content = _gen_medium_svg()
    assert '<?xml' in content
    assert '<svg' in content
    assert '</svg>' in content


def test_generate_medium_has_black_background():
    content = _gen_medium_svg()
    assert 'fill="#000000"' in content


def test_generate_medium_has_obstacle_polygons():
    content = _gen_medium_svg()
    assert 'fill="#0a0a0a"' in content
    assert 'stroke="#ffffff"' in content


def test_generate_medium_deterministic():
    a = _gen_medium_svg(seed=42)
    b = _gen_medium_svg(seed=42)
    assert a == b


def test_generate_medium_has_output_path():
    from generate_medium import generate_medium_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    output = f.name
    f.close()
    generate_medium_map(seed=42, output_path=output)
    assert os.path.exists(output)
    content = open(output).read()
    assert len(content) > 0
    os.unlink(output)
