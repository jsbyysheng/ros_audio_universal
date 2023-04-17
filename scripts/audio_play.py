#!/usr/bin/env python3
import rospy
import sounddevice as sd
import numpy as np
import queue
import sys
import sounddevice as sd
from audio_universal.msg import AudioData

'''
~output_device: use `python3 -m sounddevice` to get device list, numerical device ID or case-insensitive substrings is ok.
~channels: 1
~refresh_rate: 30
~latency: 'high'
~blocksize: 512
~dtype: 'float32'
~samplerate: 44100 48000 88200 96000 192000
'''


class audio_play:
    def __init__(self):
        self.initROS()
        self.q = queue.Queue()
        self.q_connects = queue.Queue()
        self.q_channels = queue.Queue()
        self.stream = sd.OutputStream(device=self.output_device,
                                      samplerate=self.samplerate,
                                      blocksize=self.blocksize,
                                      dtype=self.dtype,
                                      latency=self.latency,
                                      channels=self.channels,
                                      callback=self.audio_callback)
        with self.stream:
            rospy.spin()

    def initROS(self):
        rospy.init_node('audio_record', anonymous=True)
        self.output_device = rospy.get_param("~output_device", default=None)
        self.channels = rospy.get_param("~channels", default=1)
        self.refresh_rate = rospy.get_param("~refresh_rate", default=30)
        self.latency = rospy.get_param("~latency", default='high')
        self.blocksize = rospy.get_param("~blocksize", default=512)
        self.dtype = rospy.get_param("~dtype", default='float32')
        self.samplerate = rospy.get_param("~samplerate", default=48000)
        rospy.Subscriber('/audio_record_data', AudioData, self.AudioData_callback)

    def audio_callback(self, outdata, frames, time, status):
        if status:
            rospy.logwarn(status)
        try:
            data = self.q.get_nowait()
            connects = self.q_connects.get_nowait()
            in_channels = self.q_channels.get_nowait()
            data = data.reshape(self.blocksize, in_channels)
            if len(connects) / 2 != len(connects) // 2:
                raise Exception
            for idx in range(0, len(connects) // 2):
                if (connects[idx * 2] in range(0, self.channels)) and (connects[idx * 2 + 1] in range(0, self.channels)):
                    outdata[:, connects[idx * 2 + 1]] = data[:, connects[idx * 2]]
        except queue.Empty as e:
            # rospy.logwarn('Buffer is empty: increase buffersize?')
            outdata[:] = np.zeros_like(outdata)

    def AudioData_callback(self, AudioData):
        self.q.put(np.frombuffer(AudioData.data, dtype=self.dtype))
        self.q_connects.put(np.frombuffer(AudioData.connects, dtype='uint32'))
        self.q_channels.put(AudioData.channels)


if __name__ == '__main__':
    audio_play()
