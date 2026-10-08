"""ROS adapter: ~read triggers one serial read, ~cancel interrupts it."""
import math

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from barcode_reader_serial.msg import BarcodeResult
from barcode_reader_serial.srv import ReadBarcode, ReadBarcodeResponse

from .reader import BarcodeReader, config_from_params

PARAM_KEYS = ('port', 'baud', 'data_bits', 'parity', 'stop_bits', 'trigger_hex',
              'trigger_command', 'release_hex', 'reply_start_hex', 'reply_end_hex',
              'read_timeout_ms', 'no_read_reply')
INVALID_TIMEOUT = 'invalid_timeout'
MAX_TIMEOUT_S = 30.0


class RosLog:
    @staticmethod
    def info(message, *args):
        rospy.loginfo(message, *args)

    @staticmethod
    def warning(message, *args):
        rospy.logwarn(message, *args)

    @staticmethod
    def debug(message, *args):
        rospy.logdebug(message, *args)


class BarcodeReaderNode:
    def __init__(self):
        params = {key: rospy.get_param('~' + key) for key in PARAM_KEYS
                  if rospy.has_param('~' + key)}
        self.config = config_from_params(params)
        self.reader = BarcodeReader(self.config, log=RosLog())
        self.publisher = rospy.Publisher('~result', BarcodeResult, queue_size=10)
        rospy.on_shutdown(self.reader.close)
        self.reader.start()
        self.read_service = rospy.Service('~read', ReadBarcode, self.on_read)
        self.cancel_service = rospy.Service('~cancel', Trigger, self.on_cancel)
        rospy.loginfo('Barcode reader %s ready: trigger=%s release=%s framing=%s timeout=%.2fs',
                      self.config.port, self.config.trigger.hex(' ') or '-',
                      self.config.release.hex(' ') or '-',
                      'text CR/LF' if not self.config.reply_start else 'delimited',
                      self.config.read_timeout_s)

    def on_read(self, request):
        timeout_s = None
        if request.timeout_s > 0:
            if not math.isfinite(request.timeout_s) or request.timeout_s > MAX_TIMEOUT_S:
                return ReadBarcodeResponse(success=False, error=INVALID_TIMEOUT)
            timeout_s = request.timeout_s
        result = self.reader.read(timeout_s)
        message = BarcodeResult()
        message.header.stamp = rospy.Time.now()
        message.request_seq = result.request_seq
        message.success = result.success
        message.code = result.code
        message.error = result.error
        message.elapsed_s = result.elapsed_s
        message.raw_hex = result.raw.hex(' ')
        self.publisher.publish(message)
        return ReadBarcodeResponse(success=result.success, code=result.code, error=result.error,
                                   elapsed_s=result.elapsed_s, request_seq=result.request_seq,
                                   raw_hex=message.raw_hex)

    def on_cancel(self, _request):
        if self.reader.cancel():
            return TriggerResponse(success=True, message='cancelled pending read')
        return TriggerResponse(success=False, message='no pending read')


def main():
    rospy.init_node('barcode_reader_serial')
    BarcodeReaderNode()
    rospy.spin()
