#!/usr/bin/env python3
import os
import re
import struct
from dataclasses import dataclass
from multiprocessing import resource_tracker, shared_memory

import numpy as np
from ros2_efficiency_test_shm_py.msg import ShmImage

_SLOT_HEADER = struct.Struct("<I4xQQI4x")  # state, seq, timestamp_ns, size
_SLOT_HEADER_SIZE = 64
_STATE_WRITING = 0
_STATE_READY = 1


def dtype_for_encoding(encoding):
    if encoding in ("mono8", "8UC1"):
        return np.uint8, 1
    if encoding == "16UC1":
        return np.uint16, 1
    if encoding == "32FC1":
        return np.float32, 1
    if encoding in ("rgb8", "bgr8", "8UC3"):
        return np.uint8, 3
    raise ValueError(f"unsupported image encoding: {encoding}")


def shape_for_image(height, width, channels):
    if channels == 1:
        return (height, width)
    return (height, width, channels)


def stamp_key(stamp):
    return (stamp.sec, stamp.nanosec)


def _safe_shm_name(name):
    name = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return name.strip("_") or "shm_image"


def _unregister_from_resource_tracker(shm):
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


@dataclass
class SHMImageSample:
    header: object
    image: np.ndarray
    msg: ShmImage


class SHMImagePublisher:
    def __init__(self, node, topic, *, width, height, encoding="mono8", slots=8, shm_name=None, qos_profile=10):
        self.node = node
        self.topic = topic
        self.width = int(width)
        self.height = int(height)
        self.encoding = encoding
        self.dtype, self.channels = dtype_for_encoding(encoding)
        self.itemsize = np.dtype(self.dtype).itemsize
        self.step = self.width * self.channels * self.itemsize
        self.payload_size = self.step * self.height
        self.slots = int(slots)
        self.slot_stride = _SLOT_HEADER_SIZE + self.payload_size
        self.shm_size = self.slot_stride * self.slots
        base_name = shm_name or f"{node.get_name()}_{_safe_shm_name(topic)}_{os.getpid()}"
        self.shm_name = _safe_shm_name(base_name)
        self.publisher = node.create_publisher(ShmImage, topic, qos_profile)
        self.seq = 0

        try:
            old = shared_memory.SharedMemory(name=self.shm_name, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        self.shm = shared_memory.SharedMemory(name=self.shm_name, create=True, size=self.shm_size)
        self.buffer = self.shm.buf

    def close(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass

    def publish(self, image, header=None):
        arr = np.asarray(image, dtype=self.dtype)
        expected_shape = shape_for_image(self.height, self.width, self.channels)
        if arr.shape != expected_shape:
            raise ValueError(f"expected image shape {expected_shape}, got {arr.shape}")
        if not arr.flags.c_contiguous:
            arr = np.ascontiguousarray(arr)

        slot = self.seq % self.slots
        slot_base = slot * self.slot_stride
        payload_offset = slot_base + _SLOT_HEADER_SIZE
        timestamp_ns = self.node.get_clock().now().nanoseconds

        _SLOT_HEADER.pack_into(self.buffer, slot_base, _STATE_WRITING, self.seq, timestamp_ns, self.payload_size)
        self.buffer[payload_offset:payload_offset + self.payload_size] = arr.view(np.uint8).reshape(-1)
        _SLOT_HEADER.pack_into(self.buffer, slot_base, _STATE_READY, self.seq, timestamp_ns, self.payload_size)

        msg = ShmImage()
        if header is not None:
            msg.header = header
        msg.shm_name = self.shm_name
        msg.offset = payload_offset
        msg.size = self.payload_size
        msg.width = self.width
        msg.height = self.height
        msg.step = self.step
        msg.encoding = self.encoding
        msg.slot_index = slot
        msg.seq = self.seq
        self.publisher.publish(msg)
        self.seq += 1


class SHMImageSubscriber:
    def __init__(self, node, topic, callback, *, qos_profile=10):
        self.node = node
        self.callback = callback
        self._shm_by_name = {}
        self.subscription = node.create_subscription(ShmImage, topic, self._on_msg, qos_profile)

    def close(self):
        for shm in self._shm_by_name.values():
            shm.close()
        self._shm_by_name.clear()

    def _get_shm(self, name):
        shm = self._shm_by_name.get(name)
        if shm is None:
            shm = shared_memory.SharedMemory(name=name, create=False)
            _unregister_from_resource_tracker(shm)
            self._shm_by_name[name] = shm
        return shm

    def _on_msg(self, msg):
        shm = self._get_shm(msg.shm_name)
        dtype, channels = dtype_for_encoding(msg.encoding)
        shape = shape_for_image(msg.height, msg.width, channels)
        slot_base = msg.slot_index * (_SLOT_HEADER_SIZE + msg.size)
        state, seq, _timestamp_ns, size = _SLOT_HEADER.unpack_from(shm.buf, slot_base)
        if state != _STATE_READY or seq != msg.seq or size != msg.size:
            return
        image = np.ndarray(shape, dtype=dtype, buffer=shm.buf, offset=msg.offset)
        self.callback(SHMImageSample(header=msg.header, image=image, msg=msg))
