import re
import sys

from PyQt4.QtGui import *
from PyQt4.QtCore import *

from nodebox.gui.qt.ValueLadder import ValueLadder
from nodebox.util.PyFontify import fontify

whiteRE = re.compile(r"[ \t]+")
commentRE = re.compile(r"[ \t]*(#)")

def findWhitespace(s, pos=0):
    m = whiteRE.match(s, pos)
    if m is None:
        return pos
    return m.end()

stringPat = r"q[^\\q\n]*(\\[\000-\377][^\\q\n]*)*q"
stringOrCommentPat = stringPat.replace("q", "'") + "|" + stringPat.replace('q', '"') + "|#.*"
stringOrCommentRE = re.compile(stringOrCommentPat)

def removeStringsAndComments(s):
    items = []
    while 1:
        try:
            m = stringOrCommentRE.search(s)
        except TypeError:
            m = None
        if m:
            start = m.start()
            end = m.end()
            items.append(s[:start])
            if s[start] != "#":
                items.append("X" * (end - start))  # X-out strings
            s = s[end:]
        else:
            items.append(s)
            break
    return "".join([unicode(el) for el in items])

Config = {}

def loadConfig():
    if sys.platform == "darwin":
        defaultFontfamily = "Monaco"
        defaultFontsize = 11
    elif sys.platform == "win32":
        defaultFontfamily = "Courier New"
        defaultFontsize = 9
    else:
        defaultFontfamily = "Bitstream Vera Sans Mono"
        defaultFontsize = 9
        
    global Config
    settings = QSettings()
    font = settings.value("font", QVariant(QFont(defaultFontfamily, defaultFontsize)))
    font.convert(QVariant.Font)
    Config["font"] = font.toPyObject()
    colorsLightBackground = (
            ("normal", "#000000"),
            ("keyword", "#0000FF"),
            ("identifier", "#FF0000"),
            ("string", "#FF00FF"),
            ("comment", "#808080"))
    colorsDarkBackground = (
            ("normal", "#FFFFFF"),
            ("keyword", "#00FFFF"),
            ("identifier", "#AAFF00"),
            ("string", "#FF55FF"),
            ("comment", "#808080"))
    if QApplication.instance().palette().color(QPalette.Window).valueF() < 0.5:
        colors = colorsDarkBackground
    else:
        colors = colorsLightBackground
    for name, color in colors:
        Config["%sfontcolor" % name] = settings.value(
                "%sfontcolor" % name, QVariant(color)).toString()

def saveConfig():
    settings = QSettings()
    for key, value in Config.items():
        settings.setValue(key, QVariant(value))

class PythonHighlighter(QSyntaxHighlighter):
    Formats = {}

    def __init__(self, parent=None):
        super(PythonHighlighter, self).__init__(parent)
        if len(Config) == 0:
            loadConfig()
            self.initializeFormats()
        if isinstance(parent, QTextEdit):
            font = PythonHighlighter.Formats["normal"].font()
            if sys.platform == "darwin":
                if QFontInfo(font).fixedPitch() and font.pointSize() <= 10:
                    font.setStyleStrategy(QFont.NoAntialias)
            parent.setFont(font)
        self.stringRe = QRegExp(r"""(:?"["]".*"["]"|'''.*''')""")
        self.stringRe.setMinimal(True)
        self.tripleSingleRe = QRegExp(r"""'''(?!")""")
        self.tripleDoubleRe = QRegExp(r'''"""(?!')''')

    @staticmethod
    def initializeFormats():
        baseFormat = QTextCharFormat()
        font = Config["font"]
        baseFormat.setFont(font)
        for name in ("normal", "keyword", "identifier", "string", "comment"):
            format = QTextCharFormat(baseFormat)
            format.setForeground(QColor(Config["%sfontcolor" % name]))
            PythonHighlighter.Formats[name] = format

    def highlightMultiline(self, text):
        NORMAL, TRIPLESINGLE, TRIPLEDOUBLE = range(3)

        self.setCurrentBlockState(NORMAL)
        if text.indexOf(self.stringRe) != -1:
            return

        # This is fooled by triple quotes inside single quoted strings
        for i, state in ((text.indexOf(self.tripleSingleRe),
                          TRIPLESINGLE),
                         (text.indexOf(self.tripleDoubleRe),
                          TRIPLEDOUBLE)):
            if self.previousBlockState() == state:
                if i == -1:
                    i = text.length()
                    self.setCurrentBlockState(state)
                self.setFormat(0, i + 3,     
                               PythonHighlighter.Formats["string"])
            elif i > -1:
                self.setCurrentBlockState(state)
                self.setFormat(i, text.length(),
                               PythonHighlighter.Formats["string"])

    def highlightBlock(self, text):
        self.setFormat(0, text.length(), PythonHighlighter.Formats["normal"])

        for tag, start, end, sublist in fontify(unicode(text), 0):
            self.setFormat(start, end-start, PythonHighlighter.Formats[tag])

        self.highlightMultiline(text)

class PyDETextView(QTextEdit):
    def __init__(self, parent=None):
        QTextEdit.__init__(self, parent)
        self.usesTabs = 0
        self.indentSize = 4
        self.valueLadder = None
        PythonHighlighter(self)
    
    def setFont(self, font):
        QTextEdit.setFont(self, font)
        self.emit(SIGNAL("fontChanged()"))
        
    def string(self):
        return unicode(self.toPlainText())                    

    def insertText_(self, text):
        cursor = self.textCursor()
        cursor.insertText(text)

    def selectedRange(self):
        cursor = self.textCursor()
        start = location = cursor.selectionStart()
        end = cursor.selectionEnd()
        length = end - start
        return (location, length)

    def setSelectedRange_(self, rng):
        cursor = QTextCursor(self.textCursor())
        cursor.clearSelection()
        cursor.setPosition(rng[0], QTextCursor.MoveAnchor)
        cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor, rng[1])
        self.setTextCursor(cursor)

    def hideValueLadder(self):
        if self.valueLadder is not None:           
            self.valueLadder.hide()
            if self.valueLadder.dirty:
                pass
        self.valueLadder = None        

    def mouseReleaseEvent(self, event):
        self.hideValueLadder()
        QTextEdit.mouseReleaseEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.valueLadder is not None:
            self.valueLadder.mouseDragged_(event)
        else:
            QTextEdit.mouseMoveEvent(self, event)

    def mousePressEvent(self, event):
        if int(QApplication.keyboardModifiers()) & Qt.ControlModifier > 0:
            screenPoint = viewPoint = event.pos()
            c = self.cursorForPosition(screenPoint).position()
            txt = self.string()
            try:
                if txt[c] in "1234567890.":
                    # Find full number
                    begin = c
                    end = c
                    try:
                        while txt[begin-1] in "1234567890.":
                            begin-=1
                    except IndexError:
                        pass
                    try:
                        while txt[end+1] in "1234567890.":
                            end+=1
                    except IndexError:
                        pass
                    end+=1
                    self.valueLadder = ValueLadder(self, eval(txt[begin:end]), (begin,end), screenPoint, viewPoint)
            except IndexError:
                pass
        else:
            QTextEdit.mousePressEvent(self, event)

    def getLinesForRange(self, rng):
        userCursor = self.textCursor()
        cursor = QTextCursor(userCursor)
        cursor.setPosition(rng[0], QTextCursor.MoveAnchor)
        cursor.movePosition(QTextCursor.StartOfLine, QTextCursor.MoveAnchor)
        start = location = cursor.position()
        cursor.setPosition(rng[0]+rng[1], QTextCursor.KeepAnchor)
        cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
        end = cursor.position()
        length = end - start
        return cursor.selectedText(), (location, length)

    def jumpToLine_(self):
        from nodebox.gui.qt.AskString import AskString
        AskString("Jump to line number:", self._jumpToLineCallback,
                  parentWindow=self.window())

    def _jumpToLineCallback(self, value):
        if value is None:
            return  # user cancelled
        try:
            lineNo = int(value.strip())
        except ValueError:
            pass
        else:
            self.jumpToLine(lineNo)

    def jumpToLine(self, lineNo):
        try:
            lines = self.string().splitlines()
            lineNo = min(max(0, lineNo - 1), len(lines))
            length_of_prevs = sum([len(line)+1 for line in lines[:lineNo]])
            curlen = len(lines[lineNo])
            rng = (length_of_prevs, curlen)
            self.setSelectedRange_(rng)
        except Exception, e:
            from nodebox.gui.qt.util import errorAlert
            etype, value, tb = sys.exc_info()
            errorAlert(None, "(%s: %s)" % (etype, e))

    def getIndent(self):
        if self.usesTabs:
            return "\t"
        else:
            return self.indentSize * " "        

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Backspace:
            self._delete(event, False)
        elif event.key() == Qt.Key_Delete:
            self._delete(event, True)
        elif event.key() == Qt.Key_Tab:
            if not self.tabChangesFocus():
                self.insertTab_(event)
        elif event.key() in (Qt.Key_Enter, Qt.Key_Return):
            self.insertNewline_(event)
        elif event.key() == Qt.Key_V and int(QApplication.keyboardModifiers()) & Qt.ControlModifier > 0:
            self.paste()
        else:
            QTextEdit.keyPressEvent(self, event)

    def keyReleaseEvent(self, event):
        QTextEdit.keyReleaseEvent(self, event)
        self.repaint()

    def _iterLinesBackwards(self, end, maxChars=8192):
        begin = max(0, end - maxChars)
        if end > 0:
            prevChar = self.string()[end - 1]
            if prevChar == "\n":
                end += 1
        lines, linesRng = self.getLinesForRange((begin, end - begin))
        lines = lines[:end - linesRng[0]]
        lines = unicode(lines)
        linesRng = (linesRng[0], len(lines))
        lines = lines.splitlines(True)
        lines.reverse()
        for line in lines:
            nChars = len(line)
            yield line, (end - nChars, nChars)
            end -= nChars
        assert end == linesRng[0]

    def _findMatchingParen(self, index, paren):
        openToCloseMap = {"(": ")", "[": "]", "{": "}"}
        if paren:
            stack = [paren]
        else:
            stack = []
        line, lineRng, pos = None, None, None
        for line, lineRng in self._iterLinesBackwards(index):
            line = removeStringsAndComments(line)
            pos = None
            for i in range(len(line)-1, -1, -1):
                c = line[i]
                if c in ")]}":
                    stack.append(c)
                elif c in "([{":
                    if not stack:
                        if not paren:
                            pos = i
                        break
                    elif stack[-1] != openToCloseMap[c]:
                        # mismatch
                        stack = []
                        break
                    else:
                        stack.pop()
                        if paren and not stack:
                            pos = i
                            break
            if not stack:
                break
        return line, lineRng, pos

    def insertNewline_(self, event):
        selRng = self.selectedRange()
        QTextEdit.keyPressEvent(self, event)
        line, lineRng, pos = self._findMatchingParen(selRng[0], None)
        if line is None:
            return
        leadingSpace = ""
        if pos is None:
            m = whiteRE.match(line)
            if m is not None:
                leadingSpace = m.group()
        else:
            leadingSpace = re.sub(r"[^\t]", " ", line[:pos + 1])
        line, lineRng = self.getLinesForRange((selRng[0], 0))
        line = removeStringsAndComments(line).strip()
        if line and line[-1] == ":":
            leadingSpace += self.getIndent()

        if leadingSpace:
            self.insertText_(leadingSpace)

    def insertTab_(self, event):
        if self.usesTabs:
            QTextEdit.keyPressEvent(self, event)
            return
        self.insertText_("")
        selRng = self.selectedRange()
        assert selRng[1] == 0
        lines, linesRng = self.getLinesForRange(selRng)
        lines = unicode(lines)
        sel = selRng[0] - linesRng[0]
        whiteEnd = findWhitespace(lines, sel)
        nSpaces = self.indentSize - (whiteEnd % self.indentSize)
        self.insertText_(nSpaces * " ")
        sel += nSpaces
        whiteEnd += nSpaces
        sel = min(whiteEnd, sel + (sel % self.indentSize))
        self.setSelectedRange_((sel + linesRng[0], 0))

    def paste(self):
        app = QApplication.instance()
        self.insertText_(app.clipboard().mimeData().text())
        
    def delete(self):
        cursor = self.textCursor()
        cursor.removeSelectedText()

    def _delete(self, event, isForward):
        selRng = self.selectedRange()
        if self.usesTabs or selRng[1]:
            QTextEdit.keyPressEvent(self, event)
            return
        lines, linesRng = self.getLinesForRange(selRng)
        lines = unicode(lines)
        sel = selRng[0] - linesRng[0]
        whiteEnd = findWhitespace(lines, sel)
        whiteBegin = sel
        while whiteBegin and lines[whiteBegin-1] == " ":
            whiteBegin -= 1
        if not isForward:
            white = whiteBegin
        else:
            white = whiteEnd
        if white == sel or (whiteEnd - whiteBegin) <= 1:
            QTextEdit.keyPressEvent(self, event)
            return
        nSpaces = whiteEnd % self.indentSize
        if nSpaces == 0:
            nSpaces = self.indentSize
        offset = sel % self.indentSize
        if not isForward and offset == 0:
            offset = nSpaces
        delBegin = sel - offset
        delEnd = delBegin + nSpaces
        delBegin = max(delBegin, whiteBegin)
        delEnd = min(delEnd, whiteEnd)
        self.setSelectedRange_((linesRng[0] + delBegin, delEnd - delBegin))
        self.insertText_("")

    def indent(self):
        def indentFilter(lines):
            indent = self.getIndent()
            indentedLines = []
            for line in lines:
                if line.strip():
                    indentedLines.append(indent + line)
                else:
                    indentedLines.append(line)
            [indent + line for line in lines[:-1]]
            return indentedLines
        self._filterLines(indentFilter)

    def dedent(self):
        def dedentFilter(lines):
            indent = self.getIndent()
            dedentedLines = []
            indentSize = len(indent)
            for line in lines:
                if line.startswith(indent):
                    line = line[indentSize:]
                dedentedLines.append(line)
            return dedentedLines
        self._filterLines(dedentFilter)

    def comment(self):
        def commentFilter(lines):
            commentedLines = []
            indent = self.getIndent()
            pos = 100
            for line in lines:
                if not line.strip():
                    continue
                pos = min(pos, findWhitespace(line))
            for line in lines:
                if line.strip():
                    commentedLines.append(line[:pos] + "#" + line[pos:])
                else:
                    commentedLines.append(line)
            return commentedLines
        self._filterLines(commentFilter)

    def uncomment(self):
        def uncommentFilter(lines):
            commentedLines = []
            commentMatch = commentRE.match
            for line in lines:
                m = commentMatch(line)
                if m is not None:
                    pos = m.start(1)
                    line = line[:pos] + line[pos+1:]
                commentedLines.append(line)
            return commentedLines
        self._filterLines(uncommentFilter)

    def _filterLines(self, filterFunc):
        selRng = self.selectedRange()
        lines, linesRng = self.getLinesForRange(selRng)
        lines = unicode(lines)
        filteredLines = filterFunc(lines.splitlines(True))

        filteredLines = "".join(filteredLines)
        if lines == filteredLines:
            return
        self.setSelectedRange_(linesRng)
        self.insertText_(filteredLines)
        cursor = self.textCursor()
        newSelRng = (linesRng[0], len(filteredLines))
        self.setSelectedRange_(newSelRng)

    def performFindPanelAction(self):
        sender = self.sender()
        if sender is not None:
            tag = str(sender.property("tag").toString())
            from nodebox.gui.qt.findreplace import FindReplaceController
            self.emit(SIGNAL("performFindPanelAction(int)"), getattr(FindReplaceController, tag))    

    def jumpToSelection(self):
        pass
