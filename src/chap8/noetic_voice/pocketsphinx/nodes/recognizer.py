#!/usr/bin/env python3

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

Gst.init(None)


class Recognizer:
    def __init__(self):
        rospy.init_node("recognizer")

        self.started = False
        self.pipeline = None
        self.asr = None
        self.bus = None
        self.bus_id = None
        self.loop = GLib.MainLoop()

        self.pub = rospy.Publisher("~output", String, queue_size=10)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)
        rospy.on_shutdown(self.shutdown)

        source = rospy.get_param("~source", None)
        mic_name = rospy.get_param("~mic_name", None)

        if mic_name:
            source = "pulsesrc device={}".format(mic_name)
        elif not source:
            source = "autoaudiosrc"

        self.launch_config = (
            source
            + " ! audioconvert"
            + " ! audioresample"
            + " ! audio/x-raw,format=S16LE,channels=1,rate=16000"
            + " ! pocketsphinx name=asr"
            + " ! fakesink"
        )

        rospy.loginfo("GStreamer pipeline: %s", self.launch_config)
        self.start_recognizer()

    def start_recognizer(self):
        if self.started:
            return

        self.pipeline = Gst.parse_launch(self.launch_config)
        self.asr = self.pipeline.get_by_name("asr")

        lm = rospy.get_param("~lm", None)
        dic = rospy.get_param("~dict", None)
        hmm = rospy.get_param("~hmm", None)

        if lm:
            self.asr.set_property("lm", lm)
        if dic:
            self.asr.set_property("dict", dic)
        if hmm:
            self.asr.set_property("hmm", hmm)

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus_id = self.bus.connect("message", self.on_bus_message)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.started = True
        rospy.loginfo("recognizer started")

    def stop_recognizer(self):
        if not self.started:
            return

        self.pipeline.set_state(Gst.State.NULL)

        if self.bus and self.bus_id:
            self.bus.disconnect(self.bus_id)
            self.bus.remove_signal_watch()

        self.started = False
        self.pipeline = None
        self.asr = None
        self.bus = None
        self.bus_id = None

    def start(self, req):
        self.start_recognizer()
        return EmptyResponse()

    def stop(self, req):
        self.stop_recognizer()
        return EmptyResponse()

    def shutdown(self):
        self.stop_recognizer()
        if self.loop.is_running():
            self.loop.quit()

    def on_bus_message(self, bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            rospy.logerr("GStreamer error: %s", err)
            if debug:
                rospy.logerr("GStreamer debug: %s", debug)
            return

        if msg.type != Gst.MessageType.ELEMENT:
            return

        struct = msg.get_structure()
        if not struct or struct.get_name() != "pocketsphinx":
            return

        hyp = struct.get_value("hypothesis")
        final = struct.get_value("final")

        if final and hyp:
            out = String()
            out.data = str(hyp).lower()
            rospy.loginfo(out.data)
            self.pub.publish(out)
        elif hyp:
            rospy.logdebug("Partial: %s", hyp)

    def run(self):
        self.loop.run()


if __name__ == "__main__":
    Recognizer().run()
