#!/usr/bin/env python3

import ctypes
import importlib
import os
import sys
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
import speech_recognition as sr
from playsound import playsound
import tempfile
from typing import Any, cast


def _load_external_gtts_class():
    """Load gTTS from the installed gtts package, not local gtts.py."""
    module_dir = Path(__file__).resolve().parent
    original_sys_path = list(sys.path)
    try:
        filtered = []
        for path_entry in sys.path:
            try:
                if Path(path_entry).resolve() == module_dir:
                    continue
            except Exception:
                pass
            filtered.append(path_entry)

        sys.path = filtered
        sys.modules.pop("gtts", None)
        gtts_module = importlib.import_module("gtts")
        return gtts_module.gTTS
    finally:
        sys.path = original_sys_path


try:
    from gtts import gTTS
except Exception:
    gTTS = _load_external_gtts_class()

COLORS = ["red", "read", "blue"]
SHAPES = ["cube", "cylinder"]
DEFAULT_MIC_DEVICE_INDEX = None

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
            'detected_objects',
            self.detected_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseArray,
            'pick_coordinates',
            10
        )

        env_index = os.environ.get("RTK_MIC_DEVICE_INDEX", "").strip()
        if env_index:
            try:
                self.mic_device_index = int(env_index)
            except ValueError:
                self.get_logger().warn(
                    f"Invalid RTK_MIC_DEVICE_INDEX='{env_index}', using system default microphone"
                )
                self.mic_device_index = DEFAULT_MIC_DEVICE_INDEX
        else:
            self.mic_device_index = DEFAULT_MIC_DEVICE_INDEX

        suppress_alsa_errors()
        self.recognizer = sr.Recognizer()
        with suppress_stderr_fd():
            self.mic = sr.Microphone(device_index=self.mic_device_index)

        mic_desc = "system default" if self.mic_device_index is None else str(self.mic_device_index)
        self.get_logger().info(f"Speech Brain Node Started (mic device index: {mic_desc})")
        self.speak("Voice control system ready. Please give a command.")

        self.listen_loop()

    # ---------------- TTS ----------------
    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                filename = fp.name
                tts.save(filename)

            playsound(filename)
            os.remove(filename)

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

    # ---------------- ROS Callback ----------------
    def detected_callback(self, msg):
        self.detected_data = msg.data
        self.get_logger().debug(f"detected_objects updated: {self.detected_data}")

    # ---------------- Parsing ----------------
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

    def build_pick_pose_array(self, coords: str) -> PoseArray | None:
        try:
            x_str, y_str = [value.strip() for value in coords.split(",", 1)]
            x = float(x_str)
            y = float(y_str)
        except (ValueError, IndexError):
            self.get_logger().warn(f"Invalid coordinate format: {coords}")
            return None

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "workspace"
        msg.poses = [pose]
        return msg

    # ---------------- Keyword Extraction ----------------
    def extract_command(self, text):
        text = text.lower()

        found_color = None
        found_shape = None

        for color in COLORS:
            if color in text:
                if color == "read":
                    color = "red"
                found_color = color
                break

        for shape in SHAPES:
            if shape in text:
                found_shape = shape
                break

        if found_color and found_shape:
            return f"{found_color} {found_shape}"

        return None

    # ---------------- Main Loop ----------------
    def listen_loop(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.01)

                with self.mic as source:
                    with suppress_stderr_fd():
                        self.recognizer.adjust_for_ambient_noise(source, duration=1)
                    self.get_logger().info("Listening...")
                    self.speak("Listening")
                    with suppress_stderr_fd():
                        audio = self.recognizer.listen(source, phrase_time_limit=5)

                text = cast(Any, self.recognizer).recognize_google(
                    audio,
                    language="en-IN"
                )

                self.get_logger().info(f"You said: {text}")

                command = self.extract_command(text)
                if not command:
                    self.get_logger().warn("No valid object detected")
                    # self.speak("Invalid command. Please say red or blue cube or cylinder.")
                    continue

                objects = self.parse_detected_objects()

                if command in objects:
                    coords = objects[command]

                    msg = self.build_pick_pose_array(coords)
                    if msg is None:
                        continue
                    self.publisher.publish(msg)

                    self.get_logger().info(
                        f"{command} found. Sending coordinates: {coords}"
                    )

                    self.speak(f"{command} found. Picking now.")

                    time.sleep(8)  # Wait for the pick action to complete before speaking
                    self.speak("Ready for next action.")
                else:
                    self.get_logger().warn("Object not found")
                    self.get_logger().info(f"Available objects: {', '.join(objects.keys())}")
                    self.speak("Object not found in workspace.")

            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")
                self.speak("I didn't understand. Please repeat.")

            except sr.RequestError as e:
                self.get_logger().error(f"Speech API error: {e}")

            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)
    node = SpeechBrainNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()