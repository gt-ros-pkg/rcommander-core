#!/usr/bin/env python

import roslib
roslib.load_manifest('tts_server')
import rospy
from tts_server.srv import *
import glob
import os
import os.path as pt

class TTSServer:
    def __init__(self):
        self.avs = rospy.Service('available_voices', AvailableVoices, self.available_voices_cb)
        self.ss = rospy.Service('say', Say, self.say_cb)
        self.rcommander_home = pt.join(os.getenv("HOME"), '.rcommander')
        if not pt.exists(self.rcommander_home):
            os.mkdir(self.rcommander_home)

    def available_voices_cb(self, req):
        voice_list = []
        voices = glob.glob('/usr/share/festival/voices/us/*')
        voices += glob.glob('/usr/share/festival/voices/english/*')
        for vname in voices:
            voice_list.append(pt.split(vname)[1])
        return AvailableVoicesResponse(voice_list)

    def say_cb(self, req):
        txt_file_name = pt.join(self.rcommander_home, 'tts_server_temp.txt')
        wav_file_name = pt.join(self.rcommander_home, 'tts_server_temp.wav')
        try:
            txt_file = open(txt_file_name, 'w')
            txt_file.write(req.text)
            txt_file.flush()
        finally:
            txt_file.close()

        os.system("text2wave -eval '(%s)' %s -o %s" % ('voice_' + req.voice, txt_file_name, wav_file_name))
        os.system("aplay %s" % (wav_file_name))
        return SayResponse()

if __name__ == '__main__':
    rospy.init_node('tts_server')
    server = TTSServer()
    rospy.loginfo('Text-To-Speech Server Up!')
    rospy.spin()



