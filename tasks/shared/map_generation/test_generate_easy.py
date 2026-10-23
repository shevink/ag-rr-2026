import os
import tempfile


def _gen_easy_svg(seed=42):
    from generate_easy import generate_easy_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    generate_easy_map(seed=seed, output_path=f.name)
    f.flush()
    content = open(f.name).read()
    os.unlink(f.name)
    return content


def test_generate_easy_creates_svg():
    content = _gen_easy_svg()
    assert '<?xml' in content
    assert '<svg' in content
    assert '</svg>' in content


def test_generate_easy_has_black_background():
    content = _gen_easy_svg()
    assert 'fill="#000000"' in content


def test_generate_easy_has_obstacle_polygons():
    content = _gen_easy_svg()
    assert 'fill="#0a0a0a"' in content
    assert 'stroke="#ffffff"' in content


def test_generate_easy_deterministic():
    a = _gen_easy_svg(seed=42)
    b = _gen_easy_svg(seed=42)
    assert a == b


def test_generate_easy_has_output_path():
    from generate_easy import generate_easy_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    output = f.name
    f.close()
    generate_easy_map(seed=42, output_path=output)
    assert os.path.exists(output)
    content = open(output).read()
    assert len(content) > 0
    os.unlink(output)
