import Adafruit_BluefruitLE
import logging
import signal
import time
import itertools

from collections import namedtuple
from Adafruit_BluefruitLE.services import UART, DeviceInformation
from Adafruit_BluefruitLE import platform
from uuid import UUID
from construct import BitStruct, BitsInteger, Flag, Bit, ConstructError
from datetime import datetime


DATA_SERVICE_UUID = UUID('49535343-fe7d-4ae5-8fa9-9fafd205e455')
RECEIVE_CHARACTERISTIC = UUID('49535343-1E4D-4BD9-BA61-23C647249616')
RENAME_CHARACTERISTIC_UUID = UUID('00005343-0000-1000-8000-00805F9B34FB')

DATA_READ_TIMEOUT_MS = 20

# Bit number to its structure
PARSING_SCHEMA = {
    0: BitStruct(signal_strength=BitsInteger(4), has_signal=Flag, probe_unplugged=Flag, pulse_beep=Flag, sync_bit=Flag),
    1: BitStruct(pleth=BitsInteger(7), sync_bit=Flag),
    2: BitStruct(bargraph=BitsInteger(4), no_finger=Flag, pulse_research=Flag, pr_last_bit=Bit, sync_bit=Flag),
    3: BitStruct(pr_bits=BitsInteger(7), sync_bit=Flag),
    4: BitStruct(spo2=BitsInteger(7), sync_bit=Flag)
}

HeartRateData = namedtuple('HeartRateData', ['signal_strength', 'has_signal', 'probe_unplugged', 'pulse_beep', 'pleth',
                                             'bargraph', 'no_finger', 'pulse_research', 'pulse_rate', 'spo2',
                                             'timestamp'])

# TODO: add logging configuration
config_args = {
    'level': logging.INFO,
    'format': '[%(asctime)s] %(levelname).1s %(message)s',
    'datefmt': '%Y.%m.%d %H:%M:%S',
    'filename': '/tmp/bt_debug.log'
}
logging.basicConfig(**config_args)


def starting_bit_lookup_condition(x):
    try:
        return PARSING_SCHEMA[0].parse(x.to_bytes(1, 'big')).sync_bit
    except ConstructError:
        return False


def chunks(iterable, condition):
    """Yields split chunks for iterable containing sequences based on a passed condition for a head element.

    e.g.:
    [2, 1, 3, 2, 1, 3, 2, 1, 3] split over (lambda x: x == 3) ===> [2, 1], [3, 2, 1], [3, 2, 1], [3]
    [3, 2, 1, 3, 2] split over (lambda x: x == 3) ===> [3, 2, 1], [3, 2]
    [3, 2] split over (lambda x: x == 3) ===> [3, 2]
    [2, 1] split over (lambda x: x == 3) ===> [2, 1]

    :param iterable: any iterable
    :param condition: callable with 1 argument returning boolean
    :return: generator yielding tuples of sequence parts
    """
    it1, it2 = itertools.tee(iter(iterable))
    sequence_started = False
    while True:
        sequence_ended = False
        split_index = 1 if sequence_started else 0
        try:
            while not sequence_ended:
                item = next(it2)
                condition_result = condition(item)
                if condition_result and split_index == 0:
                    sequence_started = True
                elif condition_result and split_index != 0:
                    sequence_ended = True
                    break
                split_index += 1
        except StopIteration:
            pass

        if split_index and sequence_ended:
            sequence_started = True
            # got the sequence tail, going on the next iteration anyway
        chunk = bytearray(itertools.islice(it1, split_index))
        if not chunk:
            return
        yield chunk


class BluetoothPulseSensorReader(object):

    def __init__(self, ble_provider, handle_data_callback):
        logging.info('Initializing...')
        signal.signal(signal.SIGINT, self._signal_handler)
        self._handle_data_callback = handle_data_callback
        self.ble_provider = ble_provider
        self._keep_reading = True
        self._device = None

    def _signal_handler(self, _, unused_frame):
        logging.info('Terminating heartrate reader.')
        self._keep_reading = False

    def _connect(self, adapter):
        adapter.power_on()
        logging.info('Disconnecting any connected UART devices...')
        UART.disconnect_devices()
        logging.info('Searching for UART device...')
        try:
            adapter.start_scan(timeout_sec=100)
            self._device = platform.get_provider().find_device(name='BerryMed')
        finally:
            # Make sure scanning is stopped before exiting.
            adapter.stop_scan()
        if not self._device:
            raise RuntimeError("Couldn't find BerryMed pulse meter.")
        logging.info('Connecting to device...')
        logging.info('Device id: %s', str(self._device.id))
        self._device.connect()  # with default timeout 60 seconds.
        logging.info('Successfully connected.')


    def _find_pulse_data_characteristic(self):
        DeviceInformation.discover(self._device)
        heartbeat_service = None
        for s in self._device.list_services():
            logging.info('Discovered service %s: %s', str(s), s.uuid)
            if s.uuid == DATA_SERVICE_UUID:
                logging.info('Found heartbeat GATT service.')
                heartbeat_service = s
        if not heartbeat_service:
            raise RuntimeError("Couldn't find heartbeat service with uuid %s", DATA_SERVICE_UUID)
        receive_data_characteristic = None
        for c in heartbeat_service.list_characteristics():
            if c.uuid == RECEIVE_CHARACTERISTIC:
                logging.info('Found "receive" characteristic of heartbeat service.')
                receive_data_characteristic = c
        if not receive_data_characteristic:
            raise RuntimeError("Couldn't find receive data characteristic of heartbeat service (uuid %s)",
                               RECEIVE_CHARACTERISTIC)
        return receive_data_characteristic

    def start_reading(self):
        try:
            self.ble_provider.clear_cached_data()
            self._connect(ble_provider.get_default_adapter())
            pulse_characteristic = self._find_pulse_data_characteristic()
            pulse_characteristic.start_notify(lambda x: None)
            logging.info('Subscribing to heartbeat data...')
            while self._keep_reading:
                raw_data = bytearray(pulse_characteristic.read_value())
                if not raw_data:
                    continue

                for packet in chunks(raw_data, starting_bit_lookup_condition):
                    if len(packet) != len(PARSING_SCHEMA):
                        # discarding broken packet
                        continue
                    packet_dict = {}
                    try:
                        for i, schema in PARSING_SCHEMA.items():
                            byte_data_container = schema.parse(packet[i].to_bytes(1, 'big'))
                            packet_dict.update({k: v for k, v in byte_data_container.items() if not k.startswith('_')})
                        pr_start = packet_dict.pop('pr_bits')
                        pr_end = packet_dict.pop('pr_last_bit')
                        packet_dict.pop('sync_bit')
                        packet_dict['pulse_rate'] = (pr_start << 1) + pr_end
                        packet_dict['timestamp'] = str(datetime.now())
                        self._handle_data_callback(HeartRateData(**packet_dict))
                    except Exception as e:
                        logging.exception(e)
                        logging.debug('Error on parsing packet data: %s', e)
                        continue

                time.sleep(DATA_READ_TIMEOUT_MS/1000)
        finally:
            if self._device:
                self._device.disconnect()


def handle_data(heart_rate):
    logging.info(heart_rate)


ble_provider = Adafruit_BluefruitLE.get_provider()
ble_provider.initialize()
reader = BluetoothPulseSensorReader(ble_provider, handle_data)
ble_provider.run_mainloop_with(reader.start_reading)
