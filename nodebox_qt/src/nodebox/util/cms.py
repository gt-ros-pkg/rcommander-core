try:
    from lcms.lcms import *
except:
    from lcms import *

from PyQt4.QtGui import QColor
			
class CMS:
    def __init__(self, rgbFile, cmykFile):
        deviceRGB = cmsOpenProfileFromFile(rgbFile, "r")
        deviceCMYK = cmsOpenProfileFromFile(cmykFile, "r")
	
        self.cmykTransform = cmsCreateTransform(deviceCMYK, 				                        TYPE_CMYK_8, 
                                                deviceRGB, 
                                                TYPE_RGB_8, 
                                                INTENT_RELATIVE_COLORIMETRIC,
                                                cmsFLAGS_NOTPRECALC)		

        self.rgbTransform  = cmsCreateTransform(deviceRGB, 
                                                TYPE_RGB_8,
                                                deviceCMYK, 
                                                TYPE_CMYK_8,  
                                                INTENT_PERCEPTUAL, 
                                                cmsFLAGS_NOTPRECALC)		
    def convertCMYK(self, clr):
        rgb = COLORB()
        rgb[0] = clr.red()
        rgb[1] = clr.green()
        rgb[2] = clr.blue()
		
        cmyk = COLORB()
        cmyk[0] = 0
        cmyk[1] = 0
        cmyk[2] = 0
        cmyk[3] = 0
        cmsDoTransform(self.rgbTransform, rgb, cmyk, 1)

        return QColor.fromCmyk(cmyk[0], cmyk[1], cmyk[2], cmyk[3], clr.alpha())

    def convertRGB(self, clr, forced=False):
        if clr.spec() == QColor.Cmyk:
            cmyk = COLORB()
            cmyk[0] = clr.cyan()
            cmyk[1] = clr.magenta()
            cmyk[2] = clr.yellow()
            cmyk[3] = clr.black()
		
            rgb = COLORB()
            rgb[0] = 0
            rgb[1] = 0
            rgb[2] = 0		
            cmsDoTransform(self.cmykTransform, cmyk, rgb, 1)

            return QColor.fromRgb(rgb[0], rgb[1], rgb[2], clr.alpha())
        elif forced:
            clr=self.convertCMYK(clr)
            return self.convertRGB(clr)
        elif clr.spec() == QColor.Rgb:
            return clr
        return clr.toRgb()
