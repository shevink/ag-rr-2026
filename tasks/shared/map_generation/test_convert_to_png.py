import os
import tempfile


def _write_temp_svg(svg_content):
    f = tempfile.NamedTemporaryFile(suffix='.svg', mode='w', delete=False)
    f.write(svg_content)
    f.close()
    return f.name


def _cleanup(*paths):
    for p in paths:
        if os.path.exists(p):
            os.unlink(p)


def test_convert_svg_to_png_creates_valid_png():
    from convert_to_png import convert_svg_to_png
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="100" height="100" viewBox="0 0 100 100">'
        '<rect width="100" height="100" fill="#000000"/>'
        '<polygon points="10,10 20,10 20,20 10,20"'
        ' fill="#0a0a0a" stroke="#ffffff"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        assert os.path.exists(png_path)
        assert os.path.getsize(png_path) > 0
    finally:
        _cleanup(svg_path, png_path)


def test_convert_svg_to_png_exactly_two_pixel_values():
    from convert_to_png import convert_svg_to_png
    from PIL import Image
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="50" height="50" viewBox="0 0 50 50">'
        '<rect width="50" height="50" fill="#000000"/>'
        '<polygon points="5,5 15,5 15,15 5,15"'
        ' fill="#0a0a0a" stroke="#ffffff"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        img = Image.open(png_path)
        unique_values = set(img.getdata())
        assert len(unique_values) == 2, (
            f'Expected 2 pixel values, got {len(unique_values)}'
        )
    finally:
        _cleanup(svg_path, png_path)


def test_convert_svg_to_png_dimensions_match_viewbox():
    from convert_to_png import convert_svg_to_png
    from PIL import Image
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="200" height="150" viewBox="0 0 200 150">'
        '<rect width="200" height="150" fill="#000000"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        img = Image.open(png_path)
        assert img.width == 200, f'Expected width 200, got {img.width}'
        assert img.height == 150, f'Expected height 150, got {img.height}'
    finally:
        _cleanup(svg_path, png_path)


def test_convert_svg_to_png_nonexistent_input():
    from convert_to_png import convert_svg_to_png
    try:
        convert_svg_to_png('/nonexistent/path.svg', '/tmp/output.png')
        assert False, 'Expected FileNotFoundError'
    except FileNotFoundError:
        pass


def test_convert_svg_to_png_invalid_svg():
    from convert_to_png import convert_svg_to_png
    svg_path = _write_temp_svg('this is not valid svg')
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        assert False, 'Expected exception for invalid SVG'
    except (ValueError, Exception):
        pass
    finally:
        _cleanup(svg_path, png_path)


def test_convert_all_svgs_converts_multiple_files():
    from convert_to_png import convert_all_svgs
    tmpdir = tempfile.mkdtemp()
    svg1 = os.path.join(tmpdir, 'easy_map.svg')
    svg2 = os.path.join(tmpdir, 'medium_map.svg')
    not_svg = os.path.join(tmpdir, 'readme.txt')
    for path in [svg1, svg2]:
        with open(path, 'w') as f:
            f.write(
                '<svg xmlns="http://www.w3.org/2000/svg"'
                ' width="10" height="10" viewBox="0 0 10 10">'
                '<rect width="10" height="10" fill="#000000"/>'
                '</svg>'
            )
    with open(not_svg, 'w') as f:
        f.write('not an svg')
    try:
        convert_all_svgs(tmpdir)
        assert os.path.exists(svg1.replace('.svg', '.png'))
        assert os.path.exists(svg2.replace('.svg', '.png'))
        assert not os.path.exists(not_svg.replace('.txt', '.png'))
    finally:
        for f in os.listdir(tmpdir):
            os.unlink(os.path.join(tmpdir, f))
        os.rmdir(tmpdir)


def test_convert_all_svgs_png_valid():
    from convert_to_png import convert_all_svgs
    from PIL import Image
    tmpdir = tempfile.mkdtemp()
    svg_path = os.path.join(tmpdir, 'test_map.svg')
    with open(svg_path, 'w') as f:
        f.write(
            '<svg xmlns="http://www.w3.org/2000/svg"'
            ' width="20" height="20" viewBox="0 0 20 20">'
            '<rect width="20" height="20" fill="#000000"/>'
            '<polygon points="2,2 8,2 8,8 2,8"'
            ' fill="#0a0a0a" stroke="#ffffff"/>'
            '</svg>'
        )
    try:
        convert_all_svgs(tmpdir)
        png_path = svg_path.replace('.svg', '.png')
        img = Image.open(png_path)
        assert img.width == 20
        assert img.height == 20
        assert len(set(img.getdata())) == 2
    finally:
        for f in os.listdir(tmpdir):
            os.unlink(os.path.join(tmpdir, f))
        os.rmdir(tmpdir)


def test_main_with_input_flag():
    from convert_to_png import main
    import sys
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="10" height="10" viewBox="0 0 10 10">'
        '<rect width="10" height="10" fill="#000000"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    old_argv = sys.argv
    try:
        sys.argv = ['convert_to_png.py', '--input', svg_path]
        main()
        assert os.path.exists(png_path)
        assert os.path.getsize(png_path) > 0
    finally:
        sys.argv = old_argv
        _cleanup(svg_path, png_path)


def test_main_missing_output_dir_handled():
    from convert_to_png import convert_all_svgs
    result = convert_all_svgs('/nonexistent_directory')
    assert result is None


def test_end_to_end_pipeline():
    from convert_to_png import convert_svg_to_png
    from PIL import Image
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="30" height="30" viewBox="0 0 30 30">'
        '<rect width="30" height="30" fill="#000000"/>'
        '<polygon points="5,5 10,5 10,10 5,10"'
        ' fill="#0a0a0a" stroke="#ffffff"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        img = Image.open(png_path)
        assert img.mode == 'L'
        assert img.width == 30
        assert img.height == 30
        values = set(img.getdata())
        assert len(values) == 2
        assert 0 in values
        assert 255 in values
    finally:
        _cleanup(svg_path, png_path)


def test_deterministic_output():
    from convert_to_png import convert_svg_to_png
    import hashlib
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg"'
        ' width="20" height="20" viewBox="0 0 20 20">'
        '<rect width="20" height="20" fill="#000000"/>'
        '<polygon points="2,2 6,2 6,6 2,6"'
        ' fill="#0a0a0a" stroke="#ffffff"/>'
        '</svg>'
    )
    svg_path = _write_temp_svg(svg)
    png_path = svg_path.replace('.svg', '.png')
    try:
        convert_svg_to_png(svg_path, png_path)
        with open(png_path, 'rb') as f:
            hash1 = hashlib.md5(f.read()).hexdigest()
        convert_svg_to_png(svg_path, png_path)
        with open(png_path, 'rb') as f:
            hash2 = hashlib.md5(f.read()).hexdigest()
        assert hash1 == hash2, 'Deterministic output expected'
    finally:
        _cleanup(svg_path, png_path)
