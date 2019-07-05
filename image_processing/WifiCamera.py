import cv2
import numpy as np


class camera(object):
    def __init__():
        self.cam_url = 'rtsp://192.168.0.11/stream1'
        self.cap = cv2.VideoCapture(cam_url)
    def get_feed():
        # cam_url = 'rtsp://192.168.0.11/stream1'
        # cap = cv2.VideoCapture(cam_url)
        # check if cam is open
        if self.cap.isOpened():
            rval, frame = self.cap.read()
        else:
            self.cap.open(cam_url)
            rval = False
        # convert into numpy array
        shape = frame.shape
        shared_array = np.Array(
            ctypes.c_uint16, shape[0] * shape[1] * shape[2], lock=False)
        return frame
        # while rval:
        #     frame = cv2.resize(frame, (1920, 1080))  # resize the frame
        #     rval, frame = cap.read()
        #     key = cv2.waitKey(1)
        #     if key == 27:  # exit on ESC
        #         break

    def start_recording(self, output, format=None, resize=None, splitter_port=1, **options):
        frame = self.get_feed()
        output = frame

        # output = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
        # output.write(frame)

        # if 'quantization' in options:
        #     warnings.warn(
        #         PiCameraDeprecated(
        #             'The quantization option is deprecated; please use '
        #             'quality instead (same value)'))
        # with self._encoders_lock:
        #     camera_port, output_port = self._get_ports(True, splitter_port)
        #     format = self._get_video_format(output, format)
        #     encoder = self._get_video_encoder(
        #             camera_port, output_port, format, resize, **options)
        #     self._encoders[splitter_port] = encoder
        # try:
        #     encoder.start(output, options.get('motion_output'))
        # except Exception as e:
        #     encoder.close()
        #     with self._encoders_lock:
        #         del self._encoders[splitter_port]
        #     raise
    def stop_recording():
        # try:
        #     with self._encoders_lock:
        #         encoder = self._encoders[splitter_port]
        # except KeyError:
        #     raise PiCameraNotRecording(
        #             'There is no recording in progress on '
        #             'port %d' % splitter_port)
        # else:
        #     try:
        #         self.wait_recording(0, splitter_port)
        #     finally:
        #         encoder.close()
        #         with self._encoders_lock:
        #             del self._encoders[splitter_port]
