#!/usr/bin/env python3

import ctypes
import os
import sys
from collections.abc import Iterator
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from typing import Any, cast

COLORS = ["red", "blue"]
SHAPES = ["cube", "cylinder"]
MIC_DEVICE_INDEX = 5

_ALSA_HANDLER = None


def suppress_alsa_errors() -> None:
    global _ALSA_HANDLER

    if os.name != "posix":
        return

    asound = None
    for lib_name in ("libasound.so.2", "libasound.so"):
        try:
            asound = ctypes.cdll.LoadLibrary(lib_name)
            break
        except OSError:
            continue
    if asound is None:
        return

    error_handler_type = ctypes.CFUNCTYPE(
        None,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
    )

    def py_error_handler(_filename, _line, _function, _err, _fmt):
        return

    _ALSA_HANDLER = error_handler_type(py_error_handler)
    asound.snd_lib_error_set_handler(_ALSA_HANDLER)


@contextmanager
def suppress_stderr_fd() -> Iterator[None]:
    if os.name != "posix":
        yield
        return

    stderr_fd = sys.stderr.fileno()
    saved_stderr = os.dup(stderr_fd)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, stderr_fd)
        yield
    finally:
        os.dup2(saved_stderr, stderr_fd)
        os.close(saved_stderr)
        os.close(devnull)

class SpeechBrainNode(Node):

    def __init__(self):
        super().__init__('speech_brain_node')

        self.detected_data = ""

        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.detected_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/pick_coordinates',
            10
        )

        suppress_alsa_errors()
        self.recognizer = sr.Recognizer()
        try:
            with suppress_stderr_fd():
                with sr.Microphone(device_index=MIC_DEVICE_INDEX) as source:
                    if source.stream is None:
                        raise RuntimeError(
                            f"Microphone index {MIC_DEVICE_INDEX} has no usable input stream"
                        )
            self.mic = sr.Microphone(device_index=MIC_DEVICE_INDEX)
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(f"Failed to open microphone index {MIC_DEVICE_INDEX}: {exc}") from exc

        self.get_logger().info(f"Speech Brain Node Started (mic device index: {MIC_DEVICE_INDEX})")

        self.listen_loop()

    def detected_callback(self, msg):
        self.detected_data = msg.data

    def parse_detected_objects(self):
        objects = {}

        if not self.detected_data:
            return objects

        entries = self.detected_data.split("|")

        for entry in entries:
            entry = entry.strip()
            if ":" in entry:
                name, coords = entry.split(":")
                objects[name.strip()] = coords.strip()

        return objects

    def extract_command(self, text):
        text = text.lower()

        found_color = None
        found_shape = None

        for color in COLORS:
            if color in text:
                found_color = color
                break
        else:
            found_color = "unknown"

        for shape in SHAPES:
            if shape in text:
                found_shape = shape
                break
        else:
            found_shape = "unknown"

        if found_color or found_shape:
            return f"{found_color} {found_shape}"
        return None

    def listen_loop(self):
        while rclpy.ok():
            try:
                with suppress_stderr_fd():
                    with self.mic as source:
                        if source.stream is None:
                            raise RuntimeError(
                                f"Microphone index {MIC_DEVICE_INDEX} has no usable input stream"
                            )
                        self.recognizer.adjust_for_ambient_noise(source, duration=1)
                        self.get_logger().info("Listening...")
                        audio = self.recognizer.listen(source, phrase_time_limit=5)

                text = cast(Any, self.recognizer).recognize_google(
                    audio,
                    language="en-IN"
                )

                self.get_logger().info(f"You said: {text}")

                command = self.extract_command(text)

                if not command:
                    self.get_logger().warn("No valid object detected in sentence")
                    continue

                objects = self.parse_detected_objects()

                if command in objects:
                    coords = objects[command]

                    msg = String()
                    msg.data = coords
                    self.publisher.publish(msg)

                    self.get_logger().info(
                        f"{command} found. Sending coordinates: {coords}"
                    )
                else:
                    self.get_logger().warn("Object not found")

            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")

            except sr.RequestError as e:
                self.get_logger().error(f"Speech API error: {e}")

            except OSError as exc:
                self.get_logger().error(f"Microphone read error: {exc}")

            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                break

            except (AssertionError, AttributeError) as exc:
                self.get_logger().error(f"Microphone stream error: {exc}")

            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    node = SpeechBrainNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()