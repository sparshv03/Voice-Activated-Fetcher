#!/usr/bin/env python3
"""
propeller_tracker_node.py

ROS 2 node that subscribes to object detection signals and drives a
Propeller mecanum bot via serial to:
    1. Minimise lateral offset (angular PID -> yaw rate w)
    2. Move forward at constant forward_speed (vx)
         but STOP when ultrasonic_data indicates obstacle distance
         <= stop_distance_cm.
    3. If object leaves frame, rotate in the last known turn direction
         at search_turn_speed until detections resume.

Subscriptions:
    /object_offset   (std_msgs/Float32)        - normalised lateral error [-1, 1]
    /ultrasonic_data (std_msgs/Int32MultiArray) - ultrasonic distances in cm

Serial protocol (auto_car.c):
    M <vx> <vy> <w>\n   - motion command (integers, range +/-1000)
    S\n                  - full stop
    P\n                  - ping

Usage
-----
  ros2 run <your_package> propeller_tracker_node \
      --ros-args \
      -p port:=/dev/ttyUSB0 \
      -p baud:=9600 \
            -p forward_speed:=220 \
      -p stop_distance_cm:=30.0
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

import serial


# ─────────────────────────── helpers ────────────────────────────────────────

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ─────────────────────────── PID ────────────────────────────────────────────

class PID:
    """Simple discrete PID with anti-windup clamp."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float, output_max: float,
                 name: str = "PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.name = name

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def compute(self, error: float) -> float:
        now = time.monotonic()
        dt = (now - self._last_time) if self._last_time is not None else 0.0
        self._last_time = now

        if dt <= 0.0:
            dt = 1e-3

        # Proportional
        p = self.kp * error

        # Integral with clamp anti-windup
        self._integral += error * dt
        i_raw = self.ki * self._integral
        i_clamped = clamp(i_raw, self.output_min, self.output_max)
        # Back-calculate to prevent windup accumulation
        if self.ki != 0:
            self._integral = i_clamped / self.ki

        # Derivative (on measurement change, not error, to avoid kick)
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = clamp(p + i_clamped + d, self.output_min, self.output_max)
        return output


# ─────────────────────────── serial helpers ─────────────────────────────────

def serial_send(ser: serial.Serial, msg: bytes) -> None:
    try:
        ser.write(msg)
        ser.flush()
    except serial.SerialException:
        pass  # will be logged by the node


def send_motion(ser: serial.Serial, vx: int, vy: int, w: int) -> None:
    serial_send(ser, f"M {vx} {vy} {w}\n".encode("ascii"))


def send_stop(ser: serial.Serial) -> None:
    serial_send(ser, b"S\n")


def send_ping(ser: serial.Serial) -> None:
    serial_send(ser, b"P\n")


def drain_rx(ser: serial.Serial) -> None:
    """Read and discard any pending RX bytes."""
    try:
        waiting = ser.in_waiting
        if waiting:
            ser.read(waiting)
    except Exception:
        pass


# ─────────────────────────── node ───────────────────────────────────────────

class PropellerTrackerNode(Node):

    # ── tuneable defaults ──────────────────────────────────────────────────
    # Angular PID  (error = offset [-1..1], output = w [-1000..1000])
    ANG_KP = 500.0
    ANG_KI = 50.0
    ANG_KD = 18.0

    # Deadbands – ignore tiny errors to prevent motor chatter
    OFFSET_DEADBAND  = 0.04   # normalised units

    # Maximum time (s) without a fresh detection before search-spin.
    WATCHDOG_S = 0.05

    def __init__(self):
        super().__init__("propeller_tracker")

        # ── declare ROS parameters ─────────────────────────────────────────
        self.declare_parameter("port",              "/dev/ttyUSB1")
        self.declare_parameter("baud",              9600)
        self.declare_parameter("stop_distance_cm",  30.0)
        self.declare_parameter("max_speed",         800)    # vx ceiling
        self.declare_parameter("max_turn",          800)    # w  ceiling
        self.declare_parameter("forward_speed",     800)
        self.declare_parameter("search_turn_speed", 260)
        self.declare_parameter("control_hz",        20.0)
        self.declare_parameter("ultrasonic_topic",  "ultrasonic_data")
        self.declare_parameter("boot_wait_s",       2.4)
        self.declare_parameter("ang_kp",            self.ANG_KP)
        self.declare_parameter("ang_ki",            self.ANG_KI)
        self.declare_parameter("ang_kd",            self.ANG_KD)

        # ── read parameters ────────────────────────────────────────────────
        port           = self.get_parameter("port").value
        baud           = self.get_parameter("baud").value
        self.stop_dist = float(self.get_parameter("stop_distance_cm").value)
        self.max_speed = int(self.get_parameter("max_speed").value)
        self.max_turn  = int(self.get_parameter("max_turn").value)
        self.forward_speed = int(self.get_parameter("forward_speed").value)
        self.search_turn_speed = int(self.get_parameter("search_turn_speed").value)
        self.forward_speed = int(clamp(self.forward_speed, 0, self.max_speed))
        self.search_turn_speed = int(clamp(self.search_turn_speed, 0, self.max_turn))
        control_hz     = float(self.get_parameter("control_hz").value)
        ultrasonic_topic = str(self.get_parameter("ultrasonic_topic").value)
        boot_wait      = float(self.get_parameter("boot_wait_s").value)

        ang_kp = float(self.get_parameter("ang_kp").value)
        ang_ki = float(self.get_parameter("ang_ki").value)
        ang_kd = float(self.get_parameter("ang_kd").value)

        # ── PID controllers ────────────────────────────────────────────────
        self.ang_pid = PID(ang_kp, ang_ki, ang_kd,
                           -self.max_turn, self.max_turn, "angular")

        # ── state ──────────────────────────────────────────────────────────
        self.latest_offset: float | None = None
        self.last_detection_time: float = 0.0
        self._us_stop_flag = False
        self._us_min_dist: float | None = None
        self._search_turn_dir = 1
        self._prev_vx = 0
        self._prev_w  = 0

        # ── serial ─────────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.2)
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:
                pass
            time.sleep(0.08)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as exc:
            self.get_logger().fatal(f"Cannot open serial port {port}: {exc}")
            raise

        self._wait_for_boot(boot_wait)

        # ── subscriptions ──────────────────────────────────────────────────
        self.create_subscription(Float32, "object_offset",
                                 self._cb_offset,   10)
        self.create_subscription(Int32MultiArray, ultrasonic_topic,
                                 self._cb_ultrasonic, 10)
        
        # ── control loop timer ─────────────────────────────────────────────
        period = 1.0 / max(control_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"PropellerTracker ready | port={port} baud={baud} "
            f"forward_speed={self.forward_speed} search_turn_speed={self.search_turn_speed} "
            f"stop_dist={self.stop_dist} cm max_turn={self.max_turn} "
            f"ultrasonic_topic={ultrasonic_topic}  "
            f"rate={control_hz:.1f} Hz"
        )

    # ── boot handshake ─────────────────────────────────────────────────────

    def _wait_for_boot(self, boot_wait: float) -> None:
        self.get_logger().info("Waiting for Propeller boot banner …")
        t_end = time.monotonic() + boot_wait
        found = False

        while time.monotonic() < t_end and not found:
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    raw = self.ser.read(waiting).decode("ascii", errors="replace")
                    if "READY" in raw:
                        found = True
                        break
            except Exception:
                pass
            time.sleep(0.03)

        if not found:
            send_ping(self.ser)
            t_end = time.monotonic() + 0.8
            while time.monotonic() < t_end and not found:
                try:
                    waiting = self.ser.in_waiting
                    if waiting:
                        raw = self.ser.read(waiting).decode("ascii", errors="replace")
                        if "OK P" in raw or "READY" in raw:
                            found = True
                except Exception:
                    pass
                time.sleep(0.02)

        if found:
            self.get_logger().info("Propeller link OK.")
        else:
            self.get_logger().warn(
                "No READY/OK-P from Propeller. Continuing anyway – "
                "check port/baud and ensure only one process owns the port."
            )

    # ── subscription callbacks ─────────────────────────────────────────────

    def _cb_offset(self, msg: Float32) -> None:
        self.latest_offset = float(msg.data)
        self.last_detection_time = time.monotonic()

    def _cb_ultrasonic(self, msg: Int32MultiArray) -> None:
        if not msg.data:
            return

        valid = [float(value) for value in msg.data if value >= 0]
        if not valid:
            return

        min_dist = min(valid)
        stop_now = min_dist <= self.stop_dist

        if stop_now != self._us_stop_flag:
            self._us_stop_flag = stop_now
            if stop_now:
                self.get_logger().info(
                    f"Ultrasonic STOP flag ON ({min_dist:.1f}cm <= {self.stop_dist:.1f}cm)"
                )
            else:
                self.get_logger().info(
                    f"Ultrasonic STOP flag OFF ({min_dist:.1f}cm > {self.stop_dist:.1f}cm)"
                )

        self._us_min_dist = min_dist

    # ── control loop ──────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        drain_rx(self.ser)  # keep RX buffer tidy

        now = time.monotonic()
        age = now - self.last_detection_time

        if self._us_stop_flag:
            if self._prev_vx != 0 or self._prev_w != 0:
                send_stop(self.ser)
                self._prev_vx = 0
                self._prev_w  = 0
                self.ang_pid.reset()
            return

        # ── watchdog: no detection recently -> search spin ──────────────────
        if age > self.WATCHDOG_S or self.latest_offset is None:
            self.ang_pid.reset()

            vx = 0
            w = self._search_turn_dir * self.search_turn_speed

            if vx != self._prev_vx or w != self._prev_w:
                if w == 0:
                    send_stop(self.ser)
                else:
                    send_motion(self.ser, vx, 0, w)
                self._prev_vx = vx
                self._prev_w = w
                self.get_logger().info(
                    f"Target out of frame -> searching with w={w}"
                )
            return

        offset = self.latest_offset

        # ── angular PID (yaw) ──────────────────────────────────────────────
        ang_error = offset  # goal: drive offset → 0
        if abs(ang_error) < self.OFFSET_DEADBAND:
            ang_error = 0.0
            self.ang_pid._integral *= 0.85  # bleed windup gently

        w_raw = self.ang_pid.compute(ang_error)
        w = int(clamp(w_raw, -self.max_turn, self.max_turn))
        if w > 0:
            self._search_turn_dir = 1
        elif w < 0:
            self._search_turn_dir = -1

        vx = self.forward_speed

        # ── send only if changed ───────────────────────────────────────────
        if vx != self._prev_vx or w != self._prev_w:
            if vx == 0 and w == 0:
                send_stop(self.ser)
            else:
                send_motion(self.ser, vx, 0, w)

            self._prev_vx = vx
            self._prev_w  = w

            self.get_logger().debug(
                f"offset={offset:+.3f} "
                f"→ vx={vx} w={w}"
            )

    # ── cleanup ────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        try:
            send_stop(self.ser)
            send_stop(self.ser)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────── entry point ────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PropellerTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"[FATAL] Serial error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()