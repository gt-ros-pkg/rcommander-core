import os
import warnings
from random import choice, shuffle

from PyQt4.QtGui import QPainterPath, QColor, QTransform, QBrush, QPen, QImage, QPrinter, QPainter, QFontMetrics, QFontMetricsF, QFont, QTextLayout, QTextOption, QPainter#, QPixmap
from PyQt4.QtCore import Qt, QSize, QSizeF, QPointF, QRectF
from PyQt4.QtSvg import QSvgGenerator, QSvgRenderer
from OpenGL import GL

from nodebox.util import _copy_attr, _copy_attrs

__all__ = [
        "DEFAULT_WIDTH", "DEFAULT_HEIGHT",
        "inch", "cm", "mm",
        "RGB", "HSB", "CMYK",
        "CENTER", "CORNER",
        "MOVETO", "LINETO", "CURVETO", "CLOSE",
        "LEFT", "RIGHT", "CENTER", "JUSTIFY",
        "NORMAL","FORTYFIVE",
        "NUMBER", "TEXT", "BOOLEAN","BUTTON",
        "NodeBoxError",
        "Point", "Grob", "BezierPath", "PathElement", "ClippingPath", "Rect", "Oval", "Color", "Transform", "Image", "Text",
        "Variable", "Canvas",
        ]

DEFAULT_WIDTH, DEFAULT_HEIGHT = 1000, 1000

inch = 72
cm = 28.3465
mm = 2.8346

RGB = "rgb"
HSB = "hsb"
CMYK = "cmyk"

CENTER = "center"
CORNER = "corner"

MOVETO = QPainterPath.MoveToElement
LINETO = QPainterPath.LineToElement
CURVETO = QPainterPath.CurveToElement
CLOSE = "close"

LEFT = Qt.AlignLeft
RIGHT = Qt.AlignRight
CENTER = Qt.AlignHCenter
JUSTIFY = Qt.AlignJustify

NORMAL = 1
FORTYFIVE = 2

NUMBER = 1
TEXT = 2
BOOLEAN = 3
BUTTON = 4

_STATE_NAMES = {
    '_outputmode':    'outputmode',
    '_colorrange':    'colorrange',
    '_fillcolor':     'fill',
    '_strokecolor':   'stroke',
    '_strokewidth':   'strokewidth',
    '_transform':     'transform',
    '_transformmode': 'transformmode',
    '_fontname':      'font',
    '_fontsize':      'fontsize',
    '_align':         'align',
    '_lineheight':    'lineheight',
}

#def _save():
#    NSGraphicsContext.currentContext().saveGraphicsState()

#def _restore():
#    NSGraphicsContext.currentContext().restoreGraphicsState()

class CMSUnmanaged:    
    def convertCMYK(self, clr):
        return clr.toCmyk()

    def convertRGB(self, clr, forced=False):
        if clr.spec() == QColor.Rgb:
            return clr
        return clr.toRgb()
cms = CMSUnmanaged()

class NodeBoxError(Exception): pass

class Point(object):

    def __init__(self, *args):
        if len(args) == 2:
            self.x, self.y = args
        elif len(args) == 1:
            self.x, self.y = args[0]
        elif len(args) == 0:
            self.x = self.y = 0.0
        else:
            raise NodeBoxError, "Wrong initializer for Point object"

    def __repr__(self):
        return "Point(x=%.3f, y=%.3f)" % (self.x, self.y)
        
    def __eq__(self, other):
        if other is None: return False
        return self.x == other.x and self.y == other.y
        
    def __ne__(self, other):
        return not self.__eq__(other)

class Grob(object):
    """A GRaphic OBject is the base class for all DrawingPrimitives."""

    def __init__(self, ctx):
        """Initializes this object with the current context."""
        self._ctx = ctx

    def draw(self):
        """Appends the grob to the canvas.
           This will result in a draw later on, when the scene graph is rendered."""
        self._ctx.canvas.append(self)
        
    def copy(self):
        """Returns a deep copy of this grob."""
        raise NotImplementedError, "Copy is not implemented on this Grob class."
        
    def inheritFromContext(self, ignore=()):
        attrs_to_copy = list(self.__class__.stateAttributes)
        [attrs_to_copy.remove(k) for k, v in _STATE_NAMES.items() if v in ignore]
        _copy_attrs(self._ctx, self, attrs_to_copy)
        
    def checkKwargs(self, kwargs):
        remaining = [arg for arg in kwargs.keys() if arg not in self.kwargs]
        if remaining:
            raise NodeBoxError, "Unknown argument(s) '%s'" % ", ".join(remaining)
    checkKwargs = classmethod(checkKwargs)

class TransformMixin(object):

    """Mixin class for transformation support.
    Adds the _transform and _transformmode attributes to the class."""
    
    def __init__(self):
        self._reset()
        
    def _reset(self):
        self._transform = Transform()
        self._transformmode = CENTER
        
    def _get_transform(self):
        return self._transform
    def _set_transform(self, transform):
        self._transform = Transform(transform)
    transform = property(_get_transform, _set_transform)
    
    def _get_transformmode(self):
        return self._transformmode
    def _set_transformmode(self, mode):
        self._transformmode = mode
    transformmode = property(_get_transformmode, _set_transformmode)
        
    def translate(self, x, y):
        self._transform.translate(x, y)
        
    def reset(self):
        self._transform = Transform()

    def rotate(self, degrees=0, radians=0):
        self._transform.rotate(-degrees,-radians)

    def translate(self, x=0, y=0):
        self._transform.translate(x,y)

    def scale(self, x=1, y=None):
        self._transform.scale(x,y)

    def skew(self, x=0, y=0):
        self._transform.skew(x,y)
        
class ColorMixin(object):
    
    """Mixin class for color support.
    Adds the _fillcolor, _strokecolor and _strokewidth attributes to the class."""

    def __init__(self, **kwargs):
        try:
            self._fillcolor = Color(self._ctx, kwargs['fill'])
        except KeyError:
            self._fillcolor = Color(self._ctx)
        try:
            self._strokecolor = Color(self._ctx, kwargs['stroke'])
        except KeyError:
            self._strokecolor = None
        self._strokewidth = kwargs.get('strokewidth', 1.0)
        
    def _get_fill(self):
        return self._fillcolor
    def _set_fill(self, *args):
        self._fillcolor = Color(self._ctx, *args)
    fill = property(_get_fill, _set_fill)
    
    def _get_stroke(self):
        return self._strokecolor
    def _set_stroke(self, *args):
        self._strokecolor = Color(self._ctx, *args)
    stroke = property(_get_stroke, _set_stroke)
    
    def _get_strokewidth(self):
        return self._strokewidth
    def _set_strokewidth(self, strokewidth):
        self._strokewidth = max(strokewidth, 0.0001)
    strokewidth = property(_get_strokewidth, _set_strokewidth)

class BezierPath(Grob, TransformMixin, ColorMixin):
    """A BezierPath provides a wrapper around QPainterPath."""
    
    stateAttributes = ('_fillcolor', '_strokecolor', '_strokewidth', '_transform', '_transformmode')
    kwargs = ('fill', 'stroke', 'strokewidth')

    def __init__(self, ctx, path=None, **kwargs):
        super(BezierPath, self).__init__(ctx)
        TransformMixin.__init__(self)
        ColorMixin.__init__(self, **kwargs)
        self._segment_cache = None
        self._qPath_segment_cache = None
        if path is None:
            self._qPath = QPainterPath()
        elif isinstance(path, (list,tuple)):
            self._qPath = QPainterPath()
            self.extend(path)
        elif isinstance(path, BezierPath):
            self._qPath = QPainterPath(path._qPath)
            _copy_attrs(path, self, self.stateAttributes)
        elif isinstance(path, QPainterPath):
            self._qPath = path
        else:
            raise NodeBoxError, "Don't know what to do with %s." % path
        self._qPath.setFillRule(Qt.WindingFill)

    def _get_path(self):
        warnings.warn("The 'path' attribute is deprecated. Please use _qPath instead.", DeprecationWarning, stacklevel=2)
        return self._qPath
    path = property(_get_path)

    def copy(self):
        return self.__class__(self._ctx, self)

    ### Path methods ###

    def moveto(self, x, y):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.moveTo(x, y)

    def lineto(self, x, y):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.lineTo(x, y)

    def curveto(self, x1, y1, x2, y2, x3, y3):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.cubicTo(x1, y1, x2, y2, x3, y3)

    def closepath(self):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.closeSubpath() # XXX: Is this correct?

    def setlinewidth(self, width):
        self.linewidth = width

    def _get_bounds(self):
        try:
            r = self._qPath.boundingRect()
            return (r.x(), r.y()), (r.width(), r.height())
        except:
            # Path is empty -- no bounds
            return (0,0) , (0,0)

    bounds = property(_get_bounds)

    def contains(self, x, y):
        return self._qPath.contains(QPointF(x,y))
        
    ### Basic shapes ###
    
    def rect(self, x, y, width, height):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.addRect(x, y, width, height)
        
    def oval(self, x, y, width, height):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.addEllipse(x, y, width, height)
        
    def line(self, x1, y1, x2, y2):
        self._segment_cache = None
        self._qPath_segment_cache = None
        self._qPath.moveTo(x1, y1)
        self._qPath.lineTo(x2, y2)

    ### List methods ###

    def __getitem__(self, index):
        if self._qPath_segment_cache == None:
            self._set_qPath_segment_cache()
        cmd, el = self._qPath_segment_cache[index]
        return PathElement(cmd, el)

    def __iter__(self):
        for i in range(len(self)):
            yield self[i]

    def __len__(self):
        if self._qPath_segment_cache == None:
            self._set_qPath_segment_cache()
        return len(self._qPath_segment_cache)

    def _set_qPath_segment_cache(self):
        self._qPath_segment_cache = []
        count = self._qPath.elementCount()
        for i in xrange(count):
            el = self._qPath.elementAt(i)
            if el.type == CLOSE:
                self._qPath_segment_cache.append((el.type, ()))
            elif el.type in [LINETO, MOVETO]:
                self._qPath_segment_cache.append((el.type, ((el.x, el.y),)))
            elif el.type == CURVETO:
                ctrl1 = self._qPath.elementAt(i+1)
                ctrl2 = self._qPath.elementAt(i+2)
                self._qPath_segment_cache.append((el.type, ((el.x, el.y), (ctrl1.x, ctrl1.y), (ctrl2.x, ctrl2.y))))

    def extend(self, pathElements):
        self._segment_cache = None
        self._qPath_segment_cache = None
        for el in pathElements:
            if isinstance(el, (list, tuple)):
                x, y = el
                if len(self) == 0:
                    cmd = MOVETO
                else:
                    cmd = LINETO
                self.append(PathElement(cmd, ((x, y),)))
            elif isinstance(el, PathElement):
                self.append(el)
            else:
                raise NodeBoxError, "Don't know how to handle %s" % el

    def append(self, el):
        self._segment_cache = None
        self._qPath_segment_cache = None
        if el.cmd == MOVETO:
            self.moveto(el.x, el.y)
        elif el.cmd == LINETO:
            self.lineto(el.x, el.y)
        elif el.cmd == CURVETO:
            self.curveto(el.ctrl1.x, el.ctrl1.y, el.ctrl2.x, el.ctrl2.y, el.x, el.y)
        elif el.cmd == CLOSE:
            self.closepath()
            
    def _get_contours(self):
        from nodebox.graphics import bezier
        return bezier.contours(self)
    contours = property(_get_contours)

    ### Drawing methods ###

    def _get_transform(self):
        trans = self._transform.copy()
        if (self._transformmode == CENTER):
            (x, y), (w, h) = self.bounds
            deltax = x+w/2
            deltay = y+h/2
            t = Transform()
            t.translate(-deltax,-deltay)
            trans.prepend(t)
            t = Transform()
            t.translate(deltax,deltay)
            trans.append(t)
        return trans
    transform = property(_get_transform)

    def _draw(self, painter):
        painter.save()
        self.transform.concat(painter)
        if self._fillcolor:
            painter.setBrush(self._fillcolor.qColor)
        else:
            painter.setBrush(Qt.NoBrush)
        if self._strokecolor:
            painter.setPen(QPen(self._strokecolor.qColor, self._strokewidth, Qt.SolidLine, Qt.FlatCap, Qt.MiterJoin))
        else:
            painter.setPen(Qt.NoPen)
        painter.drawPath(self._qPath)
        painter.restore()
        
    def fit(self, x=None, y=None, width=None, height=None, stretch=False):

        """Fits this path to the specified bounds.
    
        All parameters are optional; if no parameters are specified, nothing will happen.
        Specifying a parameter will constrain its value:
    
        - x: The path will be positioned at the specified x value
        - y: The path will be positioned at the specified y value
        - width: The path will be of the specified width
        - height: The path will be of the specified height
        - stretch: If both width and height are defined, either stretch the path or
                keep the aspect ratio.
        """

        (px, py), (pw, ph) = self.bounds
        t = Transform()
        if x is not None and y is None:
            t.translate(x, py)
        elif x is None and y is not None:
            t.translate(px, y)
        elif x is not None and y is not None:
            t.translate(x, y)
        else:
            t.translate(px, py)
        if width is not None and height is None:
            t.scale(width / pw)
        elif width is None and height is not None:
            t.scale(height / ph)
        elif width is not None and height is not None:
            if stretch:
                t.scale(width /pw, height / ph)
            else:
                t.scale(min(width /pw, height / ph))
        t.translate(-px, -py)
        self._qPath = t.transformBezierPath(self)._qPath
        
    ### Mathematics ###
    
    def segmentlengths(self, relative=False, n=10):
        import bezier
        if relative: # Use the opportunity to store the segment cache.
            if self._segment_cache is None:
                self._segment_cache = bezier.segment_lengths(self, relative=True, n=n)
            return self._segment_cache
        else:
            return bezier.segment_lengths(self, relative=False, n=n)

    def _get_length(self, segmented=False, n=10):
        import bezier
        return bezier.length(self, segmented=segmented, n=n)
    length = property(_get_length)
        
    def point(self, t):
        import bezier
        return bezier.point(self, t)
        
    def points(self, amount=100):
        import bezier
        if len(self) == 0:
            raise NodeBoxError, "The given path is empty"

        # The delta value is divided by amount - 1, because we also want the last point (t=1.0)
        # If I wouldn't use amount - 1, I fall one point short of the end.
        # E.g. if amount = 4, I want point at t 0.0, 0.33, 0.66 and 1.0,
        # if amount = 2, I want point at t 0.0 and t 1.0
        try:
            delta = 1.0/(amount-1)
        except ZeroDivisionError:
            delta = 1.0

        for i in xrange(amount):
            yield self.point(delta*i)
            
    def addpoint(self, t):
        import bezier
        self._qPath = bezier.insert_point(self, t)._qPath
        self._segment_cache = None
        self._qPath_segment_cache = None

    ### Clipping operations ###

    def intersects(self, other):
        return self._qPath.intersects(other._qPath)

    def union(self, other, flatness=0.6):
        return BezierPath(self._ctx, self._qPath.united(other._qPath))

    def intersect(self, other, flatness=0.6):
        return BezierPath(self._ctx, self._qPath.intersected(other._qPath))

    def difference(self, other, flatness=0.6):
        return BezierPath(self._ctx, self._qPath.subtracted(other._qPath))

    def xor(self, other, flatness=0.6):
        union = self._qPath.united(other._qPath)
        intersection = self._qPath.intersected(other._qPath)
        return BezierPath(self._ctx, union.subtracted(intersection))

class PathElement(object):

    def __init__(self, cmd=None, pts=None):
        self.cmd = cmd
        if cmd == MOVETO:
            assert len(pts) == 1
            self.x, self.y = pts[0]
            self.ctrl1 = Point(pts[0])
            self.ctrl2 = Point(pts[0])
        elif cmd == LINETO:
            assert len(pts) == 1
            self.x, self.y = pts[0]
            self.ctrl1 = Point(pts[0])
            self.ctrl2 = Point(pts[0])
        elif cmd == CURVETO:
            assert len(pts) == 3
            self.ctrl1 = Point(pts[0])
            self.ctrl2 = Point(pts[1])
            self.x, self.y = pts[2]
        elif cmd == CLOSE:
            assert pts is None or len(pts) == 0
            self.x = self.y = 0.0
            self.ctrl1 = Point(0.0, 0.0)
            self.ctrl2 = Point(0.0, 0.0)
        else:
            self.x = self.y = 0.0
            self.ctrl1 = Point()
            self.ctrl2 = Point()

    def __repr__(self):
        if self.cmd == MOVETO:
            return "PathElement(MOVETO, ((%.3f, %.3f),))" % (self.x, self.y)
        elif self.cmd == LINETO:
            return "PathElement(LINETO, ((%.3f, %.3f),))" % (self.x, self.y)
        elif self.cmd == CURVETO:
            return "PathElement(CURVETO, ((%.3f, %.3f), (%.3f, %s), (%.3f, %.3f))" % \
                (self.ctrl1.x, self.ctrl1.y, self.ctrl2.x, self.ctrl2.y, self.x, self.y)
        elif self.cmd == CLOSE:
            return "PathElement(CLOSE)"
            
    def __eq__(self, other):
        if other is None: return False
        if self.cmd != other.cmd: return False
        return self.x == other.x and self.y == other.y \
            and self.ctrl1 == other.ctrl1 and self.ctrl2 == other.ctrl2
        
    def __ne__(self, other):
        return not self.__eq__(other)

class ClippingPath(Grob):

    def __init__(self, ctx, path):
        self._ctx = ctx
        self.path = path
        self._grobs = []
        
    def append(self, grob):
        self._grobs.append(grob)
        
    def _draw(self, painter):
        painter.save()
        cp = self.path.transform.transformBezierPath(self.path)
        # ugly hack to make widgets render antialiased clipping paths nicely
        # this can be further improved
        if hasattr(painter, "_screen"):
            bounds = cp._qPath.boundingRect()

            x = bounds.x()
            y = bounds.y()
            w = bounds.width()
            h = bounds.height()

            image = QImage(x+w, y+h,QImage.Format_ARGB32)
            image.fill(Qt.transparent)

            _painter = QPainter()
            _painter.begin(image)
            _painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)

            for grob in self._grobs:
                grob._draw(_painter)

            path = QPainterPath()
            path.addRect(0, 0, x+w, y+h)
            path = path.subtracted(cp._qPath)

            _painter.setCompositionMode(QPainter.CompositionMode_Clear)
            _painter.fillPath(path, QColor(0))
            _painter.end()
            
            painter.drawImage(QPointF(0, 0), image)
        else:
            painter.setClipPath(cp._qPath)
            for grob in self._grobs:
                grob._draw(painter)
        painter.restore()

class Rect(BezierPath):

    def __init__(self, ctx, x, y, width, height, **kwargs):
        warnings.warn("Rect is deprecated. Use BezierPath's rect method.", DeprecationWarning, stacklevel=2)
        p=QPainterPath()
        p.addRect(x, y, width, height)
        super(Rect, self).__init__(ctx, p, **kwargs)

    def copy(self):
        raise NotImplementedError, "Please don't use Rect anymore"

class Oval(BezierPath):

    def __init__(self, ctx, x, y, width, height, **kwargs):
        warnings.warn("Oval is deprecated. Use BezierPath's oval method.", DeprecationWarning, stacklevel=2)
        p=QPainterPath()
        p.addEllipse(x, y, width, height)
        super(Oval, self).__init__(ctx, p, **kwargs)

    def copy(self):
        raise NotImplementedError, "Please don't use Oval anymore"

class Color(object):

    def __init__(self, ctx, *args):
        self._ctx = ctx
        params = len(args)

        # Decompose the arguments into tuples. 
        if params == 1 and isinstance(args[0], tuple):
            args = args[0]
            params = len(args)

        if params == 1 and args[0] is None:
            clr = QColor(0, 0, 0)
        elif params == 1 and isinstance(args[0], Color):
            if self._ctx._outputmode == RGB:
                clr = args[0]._rgb
            else:
                clr = args[0]._cmyk
        elif params == 1 and isinstance(args[0], QColor):
            clr = args[0]
        elif params == 1: # Gray, no alpha
            args = self._normalizeList(args)
            g, = args
            clr = QColor.fromRgbF(g, g, g)
        elif params == 2: # Gray and alpha
            args = self._normalizeList(args)
            g, a = args
            clr = QColor.fromRgbF(g, g, g, a)
        elif params == 3 and self._ctx._colormode == RGB: # RGB, no alpha
            args = self._normalizeList(args)
            r,g,b = args
            clr = QColor.fromRgbF(r, g, b)
        elif params == 3 and self._ctx._colormode == HSB: # HSB, no alpha
            args = self._normalizeList(args)
            h, s, b = args
            if h == 1.0:
                h = .99998
            clr = QColor.fromHsvF(h, s, b, 1)
        elif params == 4 and self._ctx._colormode == RGB: # RGB and alpha
            args = self._normalizeList(args)
            r,g,b, a = args
            clr = QColor.fromRgbF(r, g, b, a)
        elif params == 4 and self._ctx._colormode == HSB: # HSB and alpha
            args = self._normalizeList(args)
            h, s, b, a = args
            if h == 1.0:
                h = .99998
            clr = QColor.fromHsvF(h, s, b, a)
        elif params == 4 and self._ctx._colormode == CMYK: # CMYK, no alpha
            args = self._normalizeList(args)
            c, m, y, k  = args
            clr = QColor.fromCmykF(c, m, y, k)
        elif params == 5 and self._ctx._colormode == CMYK: # CMYK and alpha
            args = self._normalizeList(args)
            c, m, y, k, a  = args
            clr = QColor.fromCmykF(c, m, y, k, a)
        else:
            clr = QColor(0, 0, 0)

        self._cmyk = cms.convertCMYK(clr) #self._cmyk = clr.colorUsingColorSpaceName_(NSDeviceCMYKColorSpace)
        self._rgb = cms.convertRGB(clr) # clr.colorUsingColorSpaceName_(NSDeviceRGBColorSpace)

    def __repr__(self):
        return "%s(%.3f, %.3f, %.3f, %.3f)" % (self.__class__.__name__, self.red,
                self.green, self.blue, self.alpha)

    def _get_qColor(self):
        if self._ctx._outputmode == RGB:
            return self._rgb
        else:
            return cms.convertRGB(self._rgb, True)
    qColor = property(_get_qColor)
        
    def copy(self):
        new = self.__class__(self._ctx)
        new._rgb = QColor(self._rgb)
        new._updateCmyk()
        return new

    def _updateCmyk(self):
        self._cmyk = cms.convertCMYK(self._rgb)

    def _updateRgb(self):
        self._rgb = cms.convertRGB(self._cmyk)

    def _get_hue(self):
        hue = self._rgb.hueF()
        if round(hue, 4) == 1.0:
            return 1.0
        if hue < 0.0:
            return 0.0
        return hue
    def _set_hue(self, val):
        val = self._normalize(val)
        if val == 1.0:
           val = .99998
        h, s, b, a = self.hsba
        self._rgb = QColor.fromHsvF(val, s, b, a).toRgb()
        self._updateCmyk()
    h = hue = property(_get_hue, _set_hue, doc="the hue of the color")

    def _get_saturation(self):
        return self._rgb.saturationF()
    def _set_saturation(self, val):
        val = self._normalize(val)
        h, s, b, a = self.hsba
        self._rgb = QColor.fromHsvF(h, val, b, a).toRgb()
        self._updateCmyk()
    s = saturation = property(_get_saturation, _set_saturation, doc="the saturation of the color")

    def _get_brightness(self):
        return self._rgb.valueF()
    def _set_brightness(self, val):
        val = self._normalize(val)
        h, s, b, a = self.hsba
        self._rgb = QColor.fromHsvF(h, s, val, a).toRgb()
        self._updateCmyk()
    v = brightness = property(_get_brightness, _set_brightness, doc="the saturation of the color")

    def _get_hsba(self):
        c = self._rgb
        return c.hueF(), c.saturationF(), c.valueF(), c.alphaF()
    def _set_hsba(self, values):
        values = self._normalizeList(values)
        h, s, b, a = values
        self._rgb = QColor.fromHsvF(h, s, b, a).toRgb()
        self._updateCmyk()
    hsba = property(_get_hsba, _set_hsba, doc="the hue, saturation, brightness and alpha of the color")

    def _get_red(self):
        return self._rgb.redF()
    def _set_red(self, val):
        val = self._normalize(val)
        r, g, b, a = self.rgba
        self._rgb.setRgbF(val, g, b, a)
        self._updateCmyk()        
    r = red = property(_get_red, _set_red, doc="the red component of the color")

    def _get_green(self):
        return self._rgb.greenF()
    def _set_green(self, val):
        val = self._normalize(val)
        r, g, b, a = self.rgba
        self._rgb.setRgbF(r, val, b, a)
        self._updateCmyk()        
    g = green = property(_get_green, _set_green, doc="the green component of the color")

    def _get_blue(self):
        return self._rgb.blueF()
    def _set_blue(self, val):
        val = self._normalize(val)
        r, g, b, a = self.rgba
        self._rgb.setRgbF(r, g, val, a)
        self._updateCmyk()        
    b = blue = property(_get_blue, _set_blue, doc="the blue component of the color")

    def _get_alpha(self):
        return self._rgb.alphaF()
    def _set_alpha(self, val):
        val = self._normalize(val)
        r, g, b, a = self.rgba
        self._rgb.setRgbF(r, g, b, val)
        self._updateCmyk()        
    a = alpha = property(_get_alpha, _set_alpha, doc="the alpha component of the color")
   
    def _get_rgba(self):
        c = self._rgb
        return c.redF(), c.greenF(), c.blueF(), c.alphaF()
    def _set_rgba(self, values):
        values = self._normalizeList(values)
        r, g, b, a = values
        self._rgb.setRgbF(r, g, b, a)
        self._updateCmyk()        
    rgba = property(_get_rgba, _set_rgba, doc="the red, green, blue and alpha values of the color")

    def _get_cyan(self):
        return self._cmyk.cyanF()
    def _set_cyan(self, val):
        val = self._normalize(val)
        c, m, y, k, a = self.cmyka
        self. _cmyk.setCmykF(val, m, y, k, a)
        self._updateRgb()        
    c = cyan = property(_get_cyan, _set_cyan, doc="the cyan component of the color")

    def _get_magenta(self):
        return self._cmyk.magentaF()
    def _set_magenta(self, val):
        val = self._normalize(val)
        c, m, y, k, a = self.cmyka
        self._cmyk.setCmykF(c, val, y, k, a)
        self._updateRgb()        
    m = magenta = property(_get_magenta, _set_magenta, doc="the magenta component of the color")

    def _get_yellow(self):
        return self._cmyk.yellowF()
    def _set_yellow(self, val):
        val = self._normalize(val)
        c, m, y, k, a = self.cmyka
        self._cmyk.setCmykF(c, m, val, k, a)
        self._updateRgb()        
    y = yellow = property(_get_yellow, _set_yellow, doc="the yellow component of the color")

    def _get_black(self):
        return self._cmyk.blackF()
    def _set_black(self, val):
        val = self._normalize(val)
        c, m, y, k, a = self.cmyka
        self._cmyk.setCmykF(c, m, y, val, a)
        self._updateRgb()        
    k = black = property(_get_black, _set_black, doc="the black component of the color")

    def _get_cmyka(self):
        c = self._cmyk
        return c.cyanF(), c.magentaF(), c.yellowF(), c.blackF(), c.alphaF()
    cmyka = property(_get_cmyka, doc="a tuple containing the CMYKA values for this color")

    def blend(self, otherColor, factor):
        r = self.red*(1-factor) + otherColor.red*factor
        g = self.green*(1-factor) + otherColor.green*factor
        b = self.blue*(1-factor) + otherColor.blue*factor
        a = self.alpha*(1-factor) + otherColor.alpha*factor
        return self.__class__(self._ctx, QColor.fromRgbF(r, g, b, a))

    def _normalize(self, v):
        """Bring the color into the 0-1 scale for the current colorrange"""
        return clamp(v / self._ctx._colorrange)

    def _normalizeList(self, lst):
        """Bring the color into the 0-1 scale for the current colorrange"""
        return map(lambda x:clamp(x / self._ctx._colorrange), lst)

color = Color

def clamp(v):
    return max(min(v, 1.0), 0.0)

class Transform(object):

    def __init__(self, transform=None):
        if transform is None:
            transform = QTransform()
        elif isinstance(transform, Transform):
            transform = QTransform(transform._qtransform)
        elif isinstance(transform, (list, tuple)):
            matrix = tuple(transform)
            transform = QTransform(*matrix)
        elif isinstance(transform, QTransform):
            pass
        else:
            raise NodeBoxError, "Don't know how to handle transform %s." % transform
        self._qtransform = transform
        
    def _get_transform(self):
        warnings.warn("The 'transform' attribute is deprecated. Please use _nsAffineTransform instead.", DeprecationWarning, stacklevel=2)
        return self._qtransform
    transform = property(_get_transform)
    
    def set(self, painter):
        painter.setTransform(self._qtransform)

    def concat(self, painter):
        trans = painter.transform()
        painter.setTransform(self._qtransform * trans)

    def copy(self):
        return Transform(self)

    def __repr__(self):
        return "<%s [%.3f %.3f %.3f %.3f %.3f %.3f]>" % ((self.__class__.__name__,)
                 + tuple(self))

    def __iter__(self):
        for value in self.matrix:
            yield value

    def _get_matrix(self):
        q = self._qtransform
        return (q.m11(), q.m12(), q.m21(), q.m22(), q.m31(), q.m32())
    def _set_matrix(self, value):
        self._qtransform = QTransform(*value)
    matrix = property(_get_matrix, _set_matrix)

    def rotate(self, degrees=0, radians=0):
        if degrees:
            self._qtransform.rotate(degrees)
        else:
            self._qtransform.rotateRadians(radians)

    def translate(self, x=0, y=0):
        self._qtransform.translate(x, y)

    def scale(self, x=1, y=None):
        if y is None:
            y = x
        self._qtransform.scale(x, y)

    def skew(self, x=0, y=0):
        import math
        x = math.pi * x / 180.
        y = math.pi * y / 180.
        t = Transform()
        t.matrix = 1, math.tan(y), -math.tan(x), 1, 0, 0
        self.prepend(t)

    def invert(self):
        try:
            self._qtransform, invertible = self._qtransform.inverted()
        except:
            pass

    def append(self, other):
        if isinstance(other, Transform):
            other = other._qtransform
        self._qtransform *= other

    def prepend(self, other):
        if isinstance(other, Transform):
            other = other._qtransform
        other = QTransform(other)
        other *= self._qtransform
        self._qtransform = other

    def transformPoint(self, point):
        if isinstance(point, (list, tuple)):
            point = QPointF(*point)
        elif isinstance(point, Point):
            point = QPointF(point.x, point.y)
        return self._qtransform.map(point)

    def transformBezierPath(self, path):
        if isinstance(path, BezierPath):
            path = BezierPath(path._ctx, path)
        else:
            raise NodeBoxError, "Can only transform BezierPaths"
        path._qPath = self._qtransform.map(path._qPath)
        return path

class Image(Grob, TransformMixin):

    stateAttributes = ('_transform', '_transformmode')
    kwargs = ()

    def __init__(self, ctx, path=None, x=0, y=0, width=None, height=None, alpha=1.0, image=None, data=None):
        """
        Parameters:
         - path: A path to a certain image on the local filesystem.
         - x: Horizontal position.
         - y: Vertical position.
         - width: Maximum width. Images get scaled according to this factor.
         - height: Maximum height. Images get scaled according to this factor.
              If a width and height are both given, the smallest 
              of the two is chosen.
         - alpha: transparency factor
         - image: optionally, an image.               
         - data: a stream of bytes of image data.
        """
        super(Image, self).__init__(ctx)
        TransformMixin.__init__(self)
        self._qsvg = None
        if data is not None:
            self.__qimage = QImage(data)
            if self.__qimage is None:
                raise NodeBoxError, "can't read image %r" % path
        elif image is not None:
            if isinstance(image, QImage):
                self.__qimage = image
            else:
                raise NodeBoxError, "Don't know what to do with %s." % image
        elif path is not None:
            if not os.path.exists(path):
                raise NodeBoxError, 'Image "%s" not found.' % path
            curtime = os.path.getmtime(path)
            try:
                image, lasttime = self._ctx._imagecache[path]
                if lasttime != curtime:
                    image = None
                elif isinstance(image, QSvgRenderer):
                    self._qsvg = image
                    image = None
            except KeyError:
                pass
            if image is None:
                if os.path.splitext(path)[1].lower() == '.svg':
                    if self._qsvg is None:
                        self._qsvg = QSvgRenderer()
                        if not self._qsvg.load(path):
                            raise NodeBoxError, 'Image "%s" is not a valid SVG file' % path
                    self._ctx._imagecache[path] = (self._qsvg, curtime)
                else:
                    image = QImage(path)
                    if image is None:
                        raise NodeBoxError, "Can't read image %s" % path
                    self._ctx._imagecache[path] = (image, curtime)
            self.__qimage = image
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.alpha = alpha
        self.debugImage = False

    def _get_qimage(self):
        if self.__qimage is not None:
            return self.__qimage
        elif self._qsvg is not None:
            svg = self._qsvg
            r = svg.viewBoxF()
            srcW = r.width()
            srcH = r.height()
            if self.width is not None or self.height is not None:
                if self.width is not None and self.height is not None:
                    factor = min(self.width / srcW, self.height / srcH)
                elif self.width is not None:
                    factor = self.width / srcW
                elif self.height is not None:
                    factor = self.height / srcH
                image = QImage(srcW*factor, srcH*factor, QImage.Format_ARGB32)
            else:
                image = QImage(srcW, srcH, QImage.Format_ARGB32)
            image.fill(Qt.transparent)
            painter = QPainter()
            painter.begin(image)
            svg.render(painter)
            painter.end()
            return image
        else:
            return None
    _qimage = property(_get_qimage)

    def _get_image(self):
        warnings.warn("The 'image' attribute is deprecated. Please use _qimage instead.", DeprecationWarning, stacklevel=2)
        return self._qimage
    image = property(_get_image)

    def copy(self):
        new = self.__class__(self._ctx)
        _copy_attrs(self, new, ('image', 'x', 'y', 'width', 'height', '_transform', '_transformmode', 'alpha', 'debugImage'))
        return new

    def getSize(self):
        if self.__qimage is not None:
            return float(self.__qimage.width()), float(self.__qimage.height())
        elif self._qsvg is not None:
            r = self._qsvg.viewBoxF()
            return (r.width(), r.height())
        else:
            return (0, 0)

    size = property(getSize)

    def _draw(self, painter):
        """Draw an image on the given coordinates."""
        if self._qsvg is not None:
            r = self._qsvg.viewBoxF()
            srcW = svgW = r.width()
            srcH = svgH = r.height()
        else:
            srcW, srcH = float(self.__qimage.width()), float(self.__qimage.height())
        srcRect = ((0, 0), (srcW, srcH))

        # Width or height given
        if self.width is not None or self.height is not None:
            if self.width is not None and self.height is not None:
                factor = min(self.width / srcW, self.height / srcH)
            elif self.width is not None:
                factor = self.width / srcW
            elif self.height is not None:
                factor = self.height / srcH
            painter.save()

            # Center-mode transforms: translate to image center
            if self._transformmode == CENTER:
                # This is the hardest case: center-mode transformations with given width or height.
                # Order is very important in this code.
                # Set the position first, before any of the scaling or transformations are done.
                # Context transformations might change the translation, and we don't want that.
                t = Transform()
                t.translate(self.x, self.y)
                t.concat(painter)
                
                # Set new width and height factors. Note that no scaling is done yet: they're just here
                # to set the new center of the image according to the scaling factors
                srcW = srcW * factor
                srcH = srcH * factor
                
                # Move image to newly calculated center.                
                dX = srcW / 2
                dY = srcH / 2
                t = Transform()
                t.translate(dX, dY)
                t.concat(painter)

                # Do current transformation.
                self._transform.concat(painter)

                # Move back to the previous position.                
                t = Transform()
                t.translate(-dX, -dY)
                t.concat(painter)

                # Finally, scale the image according to the factors.                
                t = Transform()
                t.scale(factor)
                t.concat(painter)          
            else:
                # Do current transformation
                self._transform.concat(painter)
                # Scale according to width or height factor
                t = Transform()
                t.translate(self.x, self.y) # Here we add the positioning of the image.
                t.scale(factor)
                t.concat(painter)

            # A debugImage draws a black rectangle instead of an image.
            if self.debugImage:
                painter.setBrush(QBrush(Qt.black))
                painter.setPen(QPen(Qt.NoPen))
                painter.drawRect(QRectF(0, 0, srcW / factor, srcH / factor))
            else:
                painter.setOpacity(self.alpha)
                if self._qsvg is not None:
                    self._qsvg.render(painter, QRectF(0, 0, svgW, svgH))
                else:
                    painter.drawImage(0, 0, self.__qimage)
            painter.restore()
        # No width or height given
        else:
            painter.save()
            x,y = self.x, self.y
            # Center-mode transforms: translate to image center
            if self._transformmode == CENTER:
                deltaX = srcW / 2
                deltaY = srcH / 2
                t = Transform()
                t.translate(x+deltaX, y+deltaY)
                t.concat(painter)
                x = -deltaX
                y = -deltaY
            # Do current transformation
            self._transform.concat(painter)
            # A debugImage draws a black rectangle instead of an image.
            if self.debugImage:
                painter.setBrush(QBrush(Qt.black))
                painter.setPen(QPen(Qt.NoPen))
                painter.drawRect(QRectF(x, y, srcW, srcH))
            else:
                t = Transform()
                t.translate(x, y)
                t.concat(painter)
                painter.setOpacity(self.alpha)
                if self._qsvg is not None:
                    self._qsvg.render(painter, QRectF(0, 0, svgW, svgH))
                else:
                    painter.drawImage(0, 0, self.__qimage)
            painter.restore()

b = (QFont.Bold, QFont.StyleNormal)
i = (QFont.Normal, QFont.StyleItalic)
o = (QFont.Normal, QFont.StyleOblique)
bi = (QFont.Bold, QFont.StyleItalic)
bo = (QFont.Bold, QFont.StyleOblique)
styles = {" Bold": b, "-Bold": b, " Italic": i, "-Italic": i, " Oblique": o, "-Oblique": o, " Bold Italic": bi, "-BoldItalic": bi, " Bold Oblique": bo, "-BoldOblique": bo}

class Text(Grob, TransformMixin, ColorMixin):

    stateAttributes = ('_transform', '_transformmode', '_fillcolor', '_fontname', '_fontsize', '_align', '_lineheight')
    kwargs = ('fill', 'font', 'fontsize', 'align', 'lineheight')
    fonts = {}
    styles = styles
    
    __dummy_color = QColor()
    
    def __init__(self, ctx, text, x=0, y=0, width=None, height=None, **kwargs):
        super(Text, self).__init__(ctx)
        TransformMixin.__init__(self)
        ColorMixin.__init__(self, **kwargs)
        self.text = unicode(text)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self._fontname = kwargs.get('font', "Helvetica")
        self._fontsize = kwargs.get('fontsize', 24)
        self._lineheight = max(kwargs.get('lineheight', 1.2), 0.01)
        self._align = kwargs.get('align', LEFT)

    def copy(self):
        new = self.__class__(self._ctx, self.text)
        _copy_attrs(self, new,
            ('x', 'y', 'width', 'height', '_transform', '_transformmode', 
            '_fillcolor', '_fontname', '_fontsize', '_align', '_lineheight'))
        return new
        
    def font_exists(cls, fontname):
        # Check if the font exists.
        f = QFont(fontname)
        if f.exactMatch():
            cls.fonts[fontname] = f
            return (f, True)
        else:
            styles = cls.styles
            for style in styles.keys():
                if fontname.endswith(style) or fontname.endswith(style.lower()):
                    fname = fontname[:-len(style)]
                    style = styles[style]
                    f = QFont(fname)
                    f.setWeight(style[0])
                    f.setStyle(style[1])
                    if f.exactMatch():
                        cls.fonts[fontname] = f
                        return (f, True)
        return (None, False)
    font_exists = classmethod(font_exists)

    def _get_font(self):
        try:
            f = self.fonts[self._fontname]
        except:
            f, exists = Text.font_exists(self._fontname)
            if not exists:
                f = QFont()
        f = QFont(f)
        f.setPointSizeF(self._fontsize / textScaleFactor)
        return f
    font = property(_get_font)

    def _getFontMetricsTextLayout(self):
        fm = QFontMetricsF(self.font)
        fm._lineSpacing = (fm.ascent() + fm.descent()) * self._lineheight
#        lineSpacing = (fm.ascent() + fm.descent() + fm.leading()) * self._lineheight

        if self.width is not None:
            w = self.width - 10
        else:
            w = 100000

        textOption = QTextOption()
        textOption.setAlignment(self._align)
        textOption.setWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        textOption.setUseDesignMetrics(True)

        textLayout = QTextLayout()
        textLayout.setFont(self.font)
        textLayout.setText(self.text)
        textLayout.setTextOption(textOption)
        textLayout.beginLayout()

        advance = widthUsed = 0

        while True:
            line = textLayout.createLine()
            if not line.isValid():
                break
            line.setLineWidth(w)
            line.setPosition(QPointF(0, advance))
            advance += fm._lineSpacing
            widthUsed = max(widthUsed, line.naturalTextWidth())
        textLayout.endLayout()

        if self.width is not None:
            textLayout._width = float(w)
        else:
            textLayout._width = widthUsed
            
        textLayout._height = textLayout.lineCount() * fm._lineSpacing
        return fm, textLayout      

    def _draw(self, painter):
        if self._fillcolor is None: return
        x, y = self.x, self.y
        fm, textLayout = self._getFontMetricsTextLayout()
        w = textLayout._width
        h = textLayout._height
        if self.width is None or self.width >= 10:
            dx = 10.0
        else:
            dx = self.width - 10.0
        dy = 0.0
        if self.width is not None:
            preferredWidth = self.width
            if self._align == RIGHT:
                x += preferredWidth - w
            elif self._align == CENTER:
                x += preferredWidth/2 - w/2
        
        painter.save()
        p = QPen(QBrush(self._fillcolor.qColor), 0)
        painter.setPen(p)
        
        # Center-mode transforms: translate to image center
        if self._transformmode == CENTER:
            # this isn't yet quite right
            deltaX = w / 2
            deltaY = h / 2
            t = Transform()
            t.translate(x+deltaX, y+deltaY-(fm.ascent() + fm.descent()) * (2.0-self._lineheight))
            t.concat(painter)
            self._transform.concat(painter)
            textLayout.draw(painter, QPointF(-deltaX-dx+10, -deltaY-dy))
        else:
            self._transform.concat(painter)
            textLayout.draw(painter, QPointF(x-dx+10, y-dy-(fm.ascent() + fm.descent()) * (2.0-self._lineheight)))

        painter.restore()
        return (w, h)

    def _get_metrics(self):
        fm, textLayout = self._getFontMetricsTextLayout()
        return textLayout._width, textLayout._height
    metrics = property(_get_metrics)

    def _get_path(self):
        x, y = self.x, self.y
        fm, textLayout = self._getFontMetricsTextLayout()
        w = textLayout._width
        h = textLayout._height
        if self.width is None or self.width >= 10:
            dx = 10.0
        else:
            dx = self.width - 10.0
        dy = 0.0
        if self.width is not None:
            preferredWidth = self.width
            if self._align == RIGHT:
                x += preferredWidth - w
            elif self._align == CENTER:
                x += preferredWidth/2 - w/2
        p = QPainterPath()
        index = 0
        x = x - dx + 10
        y = y - dy - (fm.ascent() + fm.descent()) * (1.2 - self._lineheight)
        for i in range(textLayout.lineCount()):
            line = textLayout.lineAt(i)
            dy = y + line.naturalTextRect().y()
            for j in range(line.textLength()):
                p.addText(x + line.cursorToX(index)[0], dy, self.font, self.text[index])
                index += 1
        path = BezierPath(self._ctx, p)
        path.inheritFromContext()
        return path
    path = property(_get_path)
    
class Variable(object):
    def __init__(self, name, type, default=None, min=0, max=100, value=None):
        self.name = name
        self.type = type or NUMBER
        if self.type == NUMBER:
            if default is None:
                self.default = 50
            else:
                self.default = default
            self.min = min
            self.max = max
        elif self.type == TEXT:
            if default is None:
                self.default = "hello"
            else:
                self.default = default
        elif self.type == BOOLEAN:
            if default is None:
                self.default = True
            else:
                self.default = default
        elif self.type == BUTTON:
            self.default = self.name
        self.value = value or self.default

    def sanitize(self, val):
        """Given a Variable and a value, cleans it out"""
        if self.type == NUMBER:
            try:
                return float(val)
            except ValueError:
                return 0.0
        elif self.type == TEXT:
            return unicode(str(val), "utf_8", "replace")
            try:
                return unicode(str(val), "utf_8", "replace")
            except:
                return ""
        elif self.type == BOOLEAN:
            if unicode(val).lower() in ("true", "1", "yes"):
                return True
            else:
                return False

    def compliesTo(self, v):
        """Return whether I am compatible with the given var:
             - Type should be the same
             - My value should be inside the given vars' min/max range.
        """
        if self.type == v.type:
            if self.type == NUMBER:
                if self.value < self.min or self.value > self.max:
                    return False
            return True
        return False

    def __repr__(self):
        return "Variable(name=%s, type=%s, default=%s, min=%s, max=%s, value=%s)" % (self.name, self.type, self.default, self.min, self.max, self.value)

class Canvas(Grob):

    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
        self.width = width
        self.height = height
        self.speed = None
        self.mousedown = False
        self.clear()

    def clear(self):
        self._grobs = self._container = []
        self._grobstack = [self._grobs]
        
    def _get_size(self):
        return self.width, self.height
    size = property(_get_size)

    def append(self, el):
        self._container.append(el)
        
    def __iter__(self):
        for grob in self._grobs:
            yield grob
            
    def __len__(self):
        return len(self._grobs)
        
    def __getitem__(self, index):
        return self._grobs[index]
        
    def push(self, containerGrob):
        self._grobstack.insert(0, containerGrob)
        self._container.append(containerGrob)
        self._container = containerGrob
        
    def pop(self):
        try:
            del self._grobstack[0]
            self._container = self._grobstack[0]
        except IndexError, e:
            raise NodeBoxError, "pop: too many canvas pops!"

    def _setTextScaleFactor(self, factor):
        global textScaleFactor
        textScaleFactor = factor
        
    def _get_qImage(self):
        img = QImage(self.width, self.height, QImage.Format_ARGB32)
        painter = QPainter()
        painter.begin(img)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.draw(painter)
        painter.end()
        return img
    _qImage = property(_get_qImage)

    def draw(self, painter):
        if self.background is not None:
            painter.save()
            painter.fillRect(0,0, self.width, self.height, self.background.qColor)
            painter.restore()
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform | QPainter.HighQualityAntialiasing)
        #GL.glHint(GL.GL_POLYGON_SMOOTH_HINT, GL.GL_NICEST);
        #GL.glEnable(GL.GL_LINE_SMOOTH)
        #GL.glEnable(GL.GL_POLYGON_SMOOTH)
        #GL.glEnable(GL.GL_MULTISAMPLE)
        #GL.glEnable(GL.GL_MULTISAMPLE_ARB)

        for grob in self._grobs:
            # try-except block to run cases where Grob's don't follow the Qt's Grob interface (ie Colors library)
            try:
                grob._draw(painter)
            except:
                pass

    def save(self, fname, format=None):
        if format is None:
            basename, ext = os.path.splitext(fname)
            format = ext[1:].lower() # Skip the dot
        if format == "svg":
            svgGen = QSvgGenerator()
            svgGen.setFileName(fname)
            svgGen.setSize(QSize(self.width, self.height))
            painter = QPainter()
            painter.begin(svgGen)
            self.draw(painter)
            painter.end()
        elif format == "pdf":
            printer = QPrinter(QPrinter.ScreenResolution)
            printer.setOutputFormat(QPrinter.PdfFormat)
            printer.setOutputFileName(fname)
            printer.setFullPage(True)
            try: # custom paper sizes are only supported as from Qt 4.4
                 printer.setPaperSize(QSizeF(self.width, self.height), QPrinter.Point)
            except:
                pass
            painter = QPainter()
            painter.begin(printer)
            self.draw(painter)
            painter.end()
        elif format in ("png", "tiff", "jpg", "jpeg"):
            img = QImage(self.width, self.height, QImage.Format_ARGB32)
            painter = QPainter()
            painter.begin(img)
            painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
            if format in ("jpg", "jpeg"):
                painter.fillRect(0, 0, self.width, self.height, Qt.white)
            self.draw(painter)
            painter.end()
            img.save(fname, None, 100)
        else:
            raise NodeBoxError("Unkown format", format)

def _test():
    import doctest, qt
    return doctest.testmod(qt)

if __name__=='__main__':
    _test()
