import xml.etree.ElementTree as ET

OBSTACLE_FILL = '#0a0a0a'
OBSTACLE_STROKE = '#ffffff'
BACKGROUND = '#000000'

SVG_NAMESPACE = 'http://www.w3.org/2000/svg'
ET.register_namespace('', SVG_NAMESPACE)


class SVGRenderer:
    def __init__(self, width=800, height=600, viewbox='0 0 800 600'):
        self.width = width
        self.height = height
        self.viewbox = viewbox
        self.elements = []
        bg = ET.Element('rect', {
            'width': '100%', 'height': '100%', 'fill': BACKGROUND
        })
        self.elements.append(bg)

    def add_rect(self, x, y, w, h, fill):
        rect = ET.Element('rect', {
            'x': str(x), 'y': str(y),
            'width': str(w), 'height': str(h),
            'fill': fill
        })
        self.elements.append(rect)

    def add_polygon(self, points, fill=OBSTACLE_FILL,
                    stroke=OBSTACLE_STROKE, stroke_width=1.0):
        path_data = 'M ' + ' L '.join(f'{x},{y}' for x, y in points) + ' Z'
        path = ET.Element('path', {
            'd': path_data,
            'fill': fill,
            'stroke': stroke,
            'stroke-width': str(stroke_width)
        })
        self.elements.append(path)

    def add_text(self, x, y, text, fill='#ffffff', font_size=0.3):
        text_elem = ET.Element('text', {
            'x': str(x), 'y': str(y),
            'fill': fill,
            'font-size': str(font_size),
            'font-family': 'sans-serif',
            'text-anchor': 'middle',
            'dominant-baseline': 'central'
        })
        text_elem.text = text
        self.elements.append(text_elem)

    def save(self, filepath):
        svg = ET.Element('svg', {
            'xmlns': SVG_NAMESPACE,
            'width': str(self.width),
            'height': str(self.height),
            'viewBox': self.viewbox
        })
        for elem in self.elements:
            svg.append(elem)
        tree = ET.ElementTree(svg)
        tree.write(filepath, encoding='UTF-8', xml_declaration=True)
