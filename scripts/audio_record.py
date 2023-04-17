#!/usr/bin/env python3
import rospy
import sounddevice as sd
import numpy as np
import queue
import sounddevice as sd
from audio_universal.msg import AudioData, AudioDisplayData
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse


'''
~input_device: use `python3 -m sounddevice` to get device list, numerical device ID or case-insensitive substrings is ok.
~channels: 1
~refresh_rate: 30
~latency: 'high'
~blocksize: 512
~dtype: 'float32'
~samplerate: 44100 48000 88200 96000 192000
~downsample: 10 for display
~connects: '0:0'
'''


class audio_record:
    def __init__(self):
        self.initROS()
        self.initAudioDevice()
        self.ctl_signal_old = not self.ctl_signal
        with self.stream:
            while not rospy.is_shutdown():
                if self.ctl_signal != self.ctl_signal_old:
                    self.ctl_signal_old = self.ctl_signal
                    if self.ctl_signal:
                        self.stream.start()
                        rospy.loginfo('Audio Device {} Open'.format(self.input_device))
                    else:
                        self.stream.stop()
                        rospy.loginfo('Audio Device {} Close'.format(self.input_device))
                if self.ctl_signal_old:
                    try:
                        data = self.vis_q.get_nowait()
                        (rows, cols) = data.shape
                        seq_start = self.audiodispdata.header.seq
                        stamp = rospy.Time.now()
                        for c in range(0, cols):
                            self.audiodispdata.header.seq = seq_start
                            for r in range(0, rows):
                                self.audiodispdata.header.seq = self.audiodispdata.header.seq + 1
                                self.publish_disp(data[r, c], c, stamp, self.audiodispdata.header.seq)
                    except queue.Empty:
                        # rospy.logwarn('Visualization Queue is empty!')
                        pass
                self.vis_interval.sleep()

    def initROS(self):
        rospy.init_node('audio_record', anonymous=True)
        self.input_device = rospy.get_param("~input_device", default=None)
        self.channels = rospy.get_param("~channels", default=1)
        self.refresh_rate = rospy.get_param("~refresh_rate", default=30)
        self.latency = rospy.get_param("~latency", default='high')
        self.blocksize = rospy.get_param("~blocksize", default=512)
        self.dtype = rospy.get_param("~dtype", default='float32')
        self.samplerate = rospy.get_param("~samplerate", default=48000)
        self.downsample = rospy.get_param("~downsample", default=10)
        self.connects = rospy.get_param("~connects", default='0:0')
        self.pub_data_visualization = []
        for ch in range(0, self.channels):
            self.pub_data_visualization.append(rospy.Publisher('audio_record_data_visualization_ch' + str(ch), AudioDisplayData, queue_size=10))
        self.pub_data = rospy.Publisher('audio_record_data', AudioData, queue_size=10)
        self.audio_control_service = rospy.Service('audio_control_service', SetBool, self.audio_control_service_call)

        self.ctl_signal = True

        self.audiodata = AudioData()
        self.connects = self.connects.replace(' ', '')
        connects_tmp = []
        for res_tmp in self.connects.split(','):
            connects_tmp.append(res_tmp[0])
            connects_tmp.append(res_tmp[2])
        self.audiodata.channels = self.channels
        self.audiodata.connects = np.array(connects_tmp, dtype='uint32').tobytes()
        self.audiodata.header.stamp = rospy.Time.now()
        self.audiodata.header.frame_id = ''
        self.audiodata.header.seq = -1

        self.audiodispdata = AudioDisplayData()
        self.audiodispdata.value = 0.0
        self.audiodispdata.header.stamp = self.audiodata.header.stamp
        self.audiodispdata.header.frame_id = ''
        self.audiodispdata.header.seq = -1

        self.vis_interval = rospy.Rate(self.refresh_rate)

    def initAudioDevice(self):
        self.vis_q = queue.Queue()
        self.stream = sd.InputStream(device=self.input_device,
                                     samplerate=self.samplerate,
                                     blocksize=self.blocksize,
                                     dtype=self.dtype,
                                     latency=self.latency,
                                     channels=self.channels,
                                     callback=self.audio_callback)

    def audio_callback(self, indata, frames, time, status):
        if status:
            rospy.logerr(status)
        data2send = bytes()
        for ch_num in range(0, self.channels):
            data2send = data2send + indata[:, ch_num].tobytes()
        self.vis_q.put(indata[:: self.downsample, :])
        self.publish(data2send)

    def publish(self, bytes_data):
        self.audiodata.header.stamp = rospy.Time.now()
        self.audiodata.header.seq = self.audiodata.header.seq + 1
        self.audiodata.data = bytes_data
        self.pub_data.publish(self.audiodata)

    def publish_disp(self, val, ch, stamp, seq):
        self.audiodispdata.header.stamp = stamp
        self.audiodispdata.header.seq = seq
        self.audiodispdata.value = val
        self.pub_data_visualization[ch].publish(self.audiodispdata)

    def audio_control_service_call(self, req):
        self.ctl_signal = req.data
        return SetBoolResponse(self.ctl_signal, '0')


if __name__ == '__main__':
    audio_record()
