from PyObjCTools import NibClassBuilder
import objc
from Foundation import *
from AppKit import *

class ProgressBarController(NibClassBuilder.AutoBaseClass):
    # the actual base class is NSWindowController
    # The following outlets are added to the class:
    # messageField
    # progressBar
    
    def init(self):
        pool = NSAutoreleasePool.alloc().init()
        NSBundle.loadNibNamed_owner_("ProgressBarSheet", self)
        self.retain()
        pool.release()
        return self

    def begin(self, message, maxval):
        self.value = 0
        self.message = message
        self.maxval = maxval
        self.progressBar.setMaxValue_(self.maxval)
        self.messageField.cell().setTitle_(self.message)
        parentWindow = NSApp().keyWindow()
        NSApp().beginSheet_modalForWindow_modalDelegate_didEndSelector_contextInfo_(self.window(), parentWindow, self, None, 0)
        
    def inc(self):
        self.value += 1
        self.progressBar.setDoubleValue_(self.value)
        date = NSDate.dateWithTimeIntervalSinceNow_(0.01)
        NSRunLoop.currentRunLoop().acceptInputForMode_beforeDate_(NSDefaultRunLoopMode, date)
        
    def end(self):
        NSApp().endSheet_(self.window())
        self.window().orderOut_(self)
        