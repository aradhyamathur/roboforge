"""MAVLink as a link, in three modes.

`vehicle`  The app is the vehicle. It sends telemetry and answers arm,
           takeoff, land and position targets, so a ground station or a
           MAVSDK / pymavlink program connects to it with no autopilot.
`client`   The app talks to an autopilot (PX4, ArduPilot, or another app in
           vehicle mode): telemetry becomes state on the bus, setpoints on
           the bus become position targets.
`hil`      The app is PX4 SITL's simulator: it sends sensor readings and
           steps only when PX4 answers with motor commands (lockstep).
           Experimental: exercised against a stand-in for PX4, not PX4 itself.

MAVLink uses NED world and FRD body axes; everything on the bus is ENU / FLU.
The conversion happens here and nowhere else. `link.raw` is the list of
pymavlink connections.
"""
from __future__ import annotations

import math
import select
import socket
import threading
import time

import numpy as np

from .. import msgs
from ..component import Link
from ..frames import (attitude_enu_to_ned, attitude_ned_to_enu, enu_to_ned, euler_to_mat, flu_to_frd, frd_to_flu, mat_to_euler, ned_to_enu,
                      yaw_of)

EARTH_R = 6378137.0
MAG_NED = np.array([0.21, 0.0, 0.43])       # a plausible field, gauss


class MavlinkLink(Link):
    name = "mavlink"

    def __init__(self, mode: str = "vehicle", url: str = None, urls: list = None, system_id: int = 1, component_id: int = 1,
                 rate: float = 50.0, home=(47.397742, 8.545594, 488.0), takeoff_altitude: float = 2.5, land_speed: float = 0.7,
                 autopilot: str = "generic", motor_map: list = None, lockstep_timeout: float = 1.0, connect_timeout: float = 30.0,
                 sensor_rate: float = 250.0, sensor_noise: float = 1.0, target_timeout: float = 1.0):
        from ..errors import ConfigError, positive, suggest
        if mode not in ("vehicle", "client", "hil"):
            raise ConfigError(f"mavlink mode must be vehicle, client or hil, got {mode!r}.{suggest(mode, ('vehicle', 'client', 'hil'))}")
        for what, v in (("rate", rate), ("sensor_rate", sensor_rate), ("lockstep_timeout", lockstep_timeout), ("connect_timeout", connect_timeout)):
            positive(v, f"mavlink {what}")
        if urls is not None and (isinstance(urls, str) or not all(isinstance(u, str) for u in urls)):
            raise ConfigError(f"mavlink urls must be a list of strings, got {urls!r}. For one endpoint use `url:`.")
        self.mode = mode
        default = {"vehicle": ["udpout:127.0.0.1:14550", "udpout:127.0.0.1:14540"], "client": ["udpin:0.0.0.0:14540"], "hil": ["tcpin:0.0.0.0:4560"]}[mode]
        self.urls = list(urls) if urls else ([url] if url else default)
        self.system_id, self.component_id, self.rate = system_id, component_id, rate
        self.home, self.takeoff_altitude, self.land_speed = tuple(home), takeoff_altitude, land_speed
        self.autopilot, self.motor_map = autopilot, motor_map
        self.lockstep_timeout, self.connect_timeout = lockstep_timeout, connect_timeout
        self.sensor_rate, self.late_answers, self._hil_connected = sensor_rate, 0, False
        self.sensor_noise, self._rng = sensor_noise, np.random.default_rng(0)
        self._late_in_a_row = 0
        self.target_timeout, self._target_t, self._target_fresh = target_timeout, 0.0, False
        self._peer_px4, self._peer_mode, self._amsl, self._offboard_from = False, 0, None, None
        self.conns: list = []
        self._lock = threading.Lock()
        self._stop = False
        self._threads: list = []
        self._next = {}
        # vehicle state
        self.armed, self.flight_mode, self.landed = False, "HOLD", True
        self._target = None
        self._ground = 0.0
        # client state
        self._acks: dict = {}
        self._ack_event = threading.Event()
        self._att = None
        self.connected = False
        self._stream_from = 0          # setpoints published after this count are streamed to the autopilot
        # hil state
        self._controls = None
        self._controls_event = threading.Event()
        self.inputs, self.outputs = {
            "vehicle": ([msgs.POSE, msgs.TWIST, msgs.CMD_VEHICLE], [msgs.STATUS, msgs.CMD_SETPOINT]),
            "client": ([msgs.CMD_SETPOINT, msgs.CMD_VEHICLE], [msgs.POSE, msgs.TWIST, msgs.STATUS]),
            "hil": ([msgs.IMU, msgs.POSE, msgs.TWIST], [msgs.CMD_ACTUATORS]),
        }[mode]

    @property
    def raw(self):
        return self.conns

    # --- setup ---------------------------------------------------------------
    def setup(self, app):
        from pymavlink import mavutil
        self._mu, self.mav = mavutil, mavutil.mavlink
        self.bus, self.app = app.bus, app
        sysid = self.system_id if self.mode != "client" else 255
        comp = self.component_id if self.mode != "client" else self.mav.MAV_COMP_ID_MISSIONPLANNER
        from ..errors import ConfigError
        if self.mode in ("vehicle", "hil") and (app.robot is None or app.robot.profile != "multirotor"):
            what = f"{app.robot.name} is {'an' if app.robot.profile[0] in 'aeiou' else 'a'} {app.robot.profile} robot" if app.robot else "this app has no robot"
            raise ConfigError(f"mavlink {self.mode} mode flies a multirotor, and {what}. Use the quadrotor, give your model "
                              f"`profile: multirotor`, or use `mode: client` to talk to an existing autopilot.")
        for url in self.urls:
            kind, _, rest = str(url).partition(":")
            host, _, port = rest.rpartition(":")
            if kind not in ("udpin", "udpout", "udp", "udpbcast", "tcp", "tcpin") and not kind.startswith("/dev/"):
                raise ConfigError(f"mavlink url {url!r} is not understood. Use udpout:host:port (send to), udpin:host:port (listen), "
                                  f"tcp:host:port, tcpin:host:port, or a serial device such as /dev/ttyUSB0.")
            if not kind.startswith("/dev/") and (not host or not port.isdigit() or not 0 < int(port) < 65536):
                raise ConfigError(f"mavlink url {url!r} needs a host and a port from 1 to 65535, as in {kind}:127.0.0.1:14540")
            try:
                if kind in ("udpin", "tcpin"):
                    # pymavlink binds with SO_REUSEADDR, so a second listener on the port would
                    # start without complaint and then steal or lose packets. Check first.
                    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM if kind == "udpin" else socket.SOCK_STREAM)
                    try:
                        probe.bind((host, int(port)))
                    finally:
                        probe.close()
                conn = mavutil.mavlink_connection(url, source_system=sysid, source_component=comp)
                conn.robust_parsing = True      # resynchronise after bad bytes instead of raising
                self.conns.append(conn)
            except OSError as e:
                why = "another program (or another copy of this app) is already using that port" if e.errno == 98 else str(e)
                raise ConnectionError(f"mavlink could not open {url}: {why}") from None
        if self.mode == "vehicle":
            pose = self.bus.latest(msgs.POSE)
            self._ground = float(pose.position[2]) if pose is not None else 0.0
            self.bus.subscribe(msgs.CMD_VEHICLE, lambda _n, c: self.command(c.name, **c.params))
            self._publish_status()
        elif self.mode == "client":
            self.bus.subscribe(msgs.CMD_VEHICLE, lambda _n, c: self.command(c.name, **c.params))
        for c in self.conns:
            th = threading.Thread(target=self._receive, args=(c,), name=f"mavlink-{self.mode}", daemon=True)
            th.start()
            self._threads.append(th)

    def _due(self, key: str, t: float, hz: float) -> bool:
        nxt = self._next.get(key, t)                 # the first call starts the schedule at t
        if t + 1e-9 < nxt:
            return False
        self._next[key] = max(nxt + 1.0 / hz, t + 0.5 / hz)
        return True

    def _send(self, fn):
        """Run `fn(conn)` on every connection; a peer that is not there yet is not an error."""
        with self._lock:
            for c in self.conns:
                try:
                    fn(c)
                except Exception:
                    pass

    def _receive(self, conn):
        udp = isinstance(conn, self._mu.mavudp)
        while not self._stop:
            try:
                if udp:
                    # Over UDP every datagram carries whole messages. pymavlink parses a byte
                    # stream, so after junk it can sit waiting for the rest of a frame that never
                    # existed and swallow the real packets that follow. Parse per datagram
                    # instead: drop a half-read frame left over from the previous datagram, read
                    # one datagram, then empty the parser.
                    if not select.select([conn.port], [], [], 0.2)[0]:
                        continue
                    p = conn.mav
                    if len(p.buf) > p.buf_index:
                        p.buf, p.buf_index, p.expected_length = bytearray(), 0, 8
                        self.dropped += 1
                    self._handle(conn, conn.recv_msg())
                    while not self._stop:
                        more = conn.mav.parse_char(b"")
                        if more is None:
                            break
                        conn.post_message(more)
                        self._handle(conn, more)
                else:
                    self._handle(conn, conn.recv_match(blocking=True, timeout=0.2))
            except OSError:                 # the socket is not usable right now (peer gone, not yet connected)
                if self._stop:
                    break
                time.sleep(0.05)
            except Exception:               # bytes that are not MAVLink
                self.dropped += 1

    def _handle(self, conn, m):
        if m is None:
            return
        kind = m.get_type()
        if kind == "BAD_DATA" or kind.startswith("UNKNOWN_"):
            self.dropped += 1
            return
        try:
            getattr(self, f"_rx_{self.mode}")(conn, m)
        except Exception as e:
            print(f"roboforge: mavlink {self.mode} could not handle {kind}: {type(e).__name__}: {e}", flush=True)

    # =====================================================================
    # vehicle
    # =====================================================================
    def _publish_status(self):
        self.bus.publish(msgs.STATUS, msgs.VehicleStatus(armed=self.armed, mode=self.flight_mode, landed=self.landed), "mavlink")

    def command(self, name: str, **params):
        """`arm`, `disarm`, `takeoff` (altitude=), `land`. In vehicle mode it acts on
        this vehicle; in client mode it is sent to the autopilot and the result
        (True if accepted) is returned."""
        if self.mode == "client":
            return self._client_command(name, **params)
        return self._vehicle_command(name, **params)

    def arm(self):
        return self.command("arm")

    def disarm(self):
        return self.command("disarm")

    def takeoff(self, altitude: float = None):
        return self.command("takeoff", **({} if altitude is None else {"altitude": altitude}))

    def land(self):
        return self.command("land")

    def _vehicle_command(self, name, **params) -> bool:
        pose = self.bus.latest(msgs.POSE)
        if name == "arm":
            self.armed = True
            if pose is not None and self._target is None:
                self._target = msgs.Setpoint(position=pose.position.copy(), yaw=yaw_of(pose.orientation))
                self.bus.publish(msgs.CMD_SETPOINT, self._target, "mavlink")
        elif name == "disarm":
            if not self.landed and not params.get("force"):
                return False
            self.armed, self.flight_mode = False, "HOLD"
        elif name == "takeoff":
            if not self.armed or pose is None:
                return False
            alt = float(params.get("altitude") or self.takeoff_altitude)
            if not math.isfinite(alt) or alt <= 0:
                return False            # a takeoff goes up
            self.flight_mode, self.landed = "TAKEOFF", False
            self._target = msgs.Setpoint(position=np.array([pose.position[0], pose.position[1], self._ground + alt]), yaw=yaw_of(pose.orientation))
            self.bus.publish(msgs.CMD_SETPOINT, self._target, "mavlink")
        elif name == "land":
            if pose is None:
                return False
            self.flight_mode = "LAND"
            self._target = msgs.Setpoint(position=pose.position.copy(), yaw=yaw_of(pose.orientation))
        else:
            return False
        self._publish_status()
        return True

    def _rx_vehicle(self, conn, m):
        mav, t = self.mav, m.get_type()
        if t == "COMMAND_LONG":
            ok, cmd = False, m.command
            if cmd == mav.MAV_CMD_COMPONENT_ARM_DISARM:
                ok = self._vehicle_command("arm") if m.param1 > 0.5 else self._vehicle_command("disarm", force=m.param2 > 21000)
            elif cmd == mav.MAV_CMD_NAV_TAKEOFF:
                alt = m.param7
                # Autopilots take an absolute (AMSL) altitude here; small values are meant as "above ground".
                alt = None if (alt is None or math.isnan(alt) or alt == 0) else (alt - self.home[2] if alt > self.home[2] / 2 else alt)
                ok = self._vehicle_command("takeoff", altitude=alt)
            elif cmd == mav.MAV_CMD_NAV_LAND:
                ok = self._vehicle_command("land")
            elif cmd in (mav.MAV_CMD_DO_SET_MODE, mav.MAV_CMD_SET_MESSAGE_INTERVAL):
                ok = True
            elif cmd == mav.MAV_CMD_REQUEST_MESSAGE and int(m.param1) == mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                ok = True
                with self._lock:
                    conn.mav.autopilot_version_send(mav.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED | mav.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
                                                    1, 0, 0, 0, bytes(8), bytes(8), bytes(8), 0, 0, 0, bytes(18))
            result = mav.MAV_RESULT_ACCEPTED if ok else (mav.MAV_RESULT_UNSUPPORTED if cmd not in (
                mav.MAV_CMD_COMPONENT_ARM_DISARM, mav.MAV_CMD_NAV_TAKEOFF, mav.MAV_CMD_NAV_LAND) else mav.MAV_RESULT_DENIED)
            with self._lock:
                conn.mav.command_ack_send(cmd, result)
        elif t == "SET_POSITION_TARGET_LOCAL_NED" and self.armed and self.flight_mode != "LAND":   # landing is not interrupted by a stale target stream
            ignore_pos = (m.type_mask & 0b111) == 0b111
            ignore_vel = (m.type_mask & 0b111000) == 0b111000
            yaw = 0.0 if (m.type_mask & 0x400) else math.pi / 2 - m.yaw
            used = ([] if ignore_pos else [m.x, m.y, m.z]) + ([] if ignore_vel else [m.vx, m.vy, m.vz]) + ([] if m.type_mask & 0x400 else [m.yaw])
            if not self.finite(used):
                return self.reject("a position target", "it contains nan or inf")
            if ignore_pos and ignore_vel:
                return self.reject("a position target", "its type_mask ignores both position and velocity (acceleration targets are not supported)")
            pos = None if ignore_pos else ned_to_enu([m.x, m.y, m.z]) + np.array([0.0, 0.0, self._ground])
            vel = None if ignore_vel else ned_to_enu([m.vx, m.vy, m.vz])
            self.flight_mode, self.landed = "OFFBOARD", False
            self._target = msgs.Setpoint(position=pos, velocity=vel, yaw=yaw)
            self._target_fresh = True
            self.bus.publish(msgs.CMD_SETPOINT, self._target, "mavlink")
            self._publish_status()

    def _vehicle_step(self, t: float):
        pose, twist = self.bus.latest(msgs.POSE), self.bus.latest(msgs.TWIST)
        if pose is None or twist is None:
            return
        height = float(pose.position[2]) - self._ground
        # --- mode logic ---
        if self._target_fresh:
            self._target_t, self._target_fresh = t, False
        if (self.flight_mode == "OFFBOARD" and self._target is not None and self._target.position is None
                and t - self._target_t > self.target_timeout):
            # A velocity target that is no longer being refreshed: the sender has gone. Stop and hold here.
            self.flight_mode = "HOLD"
            self._target = msgs.Setpoint(position=pose.position.copy(), yaw=yaw_of(pose.orientation))
            self.bus.publish(msgs.CMD_SETPOINT, self._target, "mavlink")
            self._publish_status()
        if self.flight_mode == "TAKEOFF" and self._target is not None and abs(pose.position[2] - self._target.position[2]) < 0.2:
            self.flight_mode = "HOLD"
            self._publish_status()
        elif self.flight_mode == "LAND":
            if height < 0.12 and abs(twist.linear[2]) < 0.25:
                self.armed, self.landed, self.flight_mode, self._target = False, True, "HOLD", None
                self._publish_status()
            elif self._due("land", t, 20.0):
                tgt = self._target.position.copy()
                tgt[2] = max(self._ground - 0.2, pose.position[2] - self.land_speed * 0.5)
                self._target = msgs.Setpoint(position=tgt, yaw=self._target.yaw)
                self.bus.publish(msgs.CMD_SETPOINT, self._target, "mavlink")
        # --- telemetry ---
        mav, ms = self.mav, int(t * 1000) & 0xFFFFFFFF
        if self._due("hb", t, 1.0):
            base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav.MAV_MODE_FLAG_GUIDED_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            ap = mav.MAV_AUTOPILOT_PX4 if self.autopilot == "px4" else mav.MAV_AUTOPILOT_GENERIC
            state = mav.MAV_STATE_ACTIVE if self.armed else mav.MAV_STATE_STANDBY
            landed = mav.MAV_LANDED_STATE_ON_GROUND if self.landed else mav.MAV_LANDED_STATE_IN_AIR
            lat, lon = self._latlon(pose.position)
            self._send(lambda c: (c.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, ap, base, 0, state),
                                  c.mav.sys_status_send(0, 0, 0, 0, 16000, -1, 100, 0, 0, 0, 0, 0, 0),
                                  c.mav.extended_sys_state_send(mav.MAV_VTOL_STATE_UNDEFINED, landed),
                                  c.mav.home_position_send(int(self.home[0] * 1e7), int(self.home[1] * 1e7), int(self.home[2] * 1000), 0, 0, 0, [1, 0, 0, 0], 0, 0, 0),
                                  c.mav.gps_raw_int_send(int(t * 1e6), 3, int(lat * 1e7), int(lon * 1e7), int((self.home[2] + height) * 1000), 80, 80, 0, 0, 12)))
        if self._due("tm", t, self.rate):
            R = attitude_enu_to_ned(pose.orientation)
            roll, pitch, yaw = mat_to_euler(R)
            rates = flu_to_frd(twist.angular)
            p = enu_to_ned(pose.position - np.array([0.0, 0.0, self._ground]))
            v = enu_to_ned(twist.linear)
            lat, lon = self._latlon(pose.position)
            hdg = int(math.degrees(yaw) % 360 * 100)
            self._send(lambda c: (c.mav.attitude_send(ms, roll, pitch, yaw, rates[0], rates[1], rates[2]),
                                  c.mav.local_position_ned_send(ms, p[0], p[1], p[2], v[0], v[1], v[2]),
                                  c.mav.global_position_int_send(ms, int(lat * 1e7), int(lon * 1e7), int((self.home[2] + height) * 1000), int(height * 1000),
                                                                 int(v[0] * 100), int(v[1] * 100), int(v[2] * 100), hdg)))

    def _latlon(self, pos):
        lat = self.home[0] + math.degrees(pos[1] / EARTH_R)
        lon = self.home[1] + math.degrees(pos[0] / (EARTH_R * math.cos(math.radians(self.home[0]))))
        return lat, lon

    # =====================================================================
    # client
    # =====================================================================
    def _rx_client(self, conn, m):
        t = m.get_type()
        if t == "HEARTBEAT" and m.type != self.mav.MAV_TYPE_GCS:
            self.connected = True
            conn.target_system, conn.target_component = m.get_srcSystem(), m.get_srcComponent()
            self.armed = bool(m.base_mode & self.mav.MAV_MODE_FLAG_SAFETY_ARMED)
            self._peer_px4, self._peer_mode = m.autopilot == self.mav.MAV_AUTOPILOT_PX4, m.custom_mode
            self.bus.publish(msgs.STATUS, msgs.VehicleStatus(armed=self.armed, mode=self.flight_mode, landed=self.landed), "mavlink")
        elif t == "EXTENDED_SYS_STATE":
            self.landed = m.landed_state == self.mav.MAV_LANDED_STATE_ON_GROUND
        elif t == "GLOBAL_POSITION_INT":
            self._amsl = m.alt / 1000.0
        elif t == "ATTITUDE":
            if not self.finite([m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed]):
                return self.reject("an ATTITUDE message", "it contains nan or inf")
            self._att = (attitude_ned_to_enu(euler_to_mat(m.roll, m.pitch, m.yaw)), frd_to_flu([m.rollspeed, m.pitchspeed, m.yawspeed]))
        elif t == "LOCAL_POSITION_NED":
            if not self.finite([m.x, m.y, m.z, m.vx, m.vy, m.vz]):
                return self.reject("a LOCAL_POSITION_NED message", "it contains nan or inf")
            q, rates = self._att if self._att is not None else (np.array([1.0, 0, 0, 0]), np.zeros(3))
            stamp = m.time_boot_ms / 1000.0
            self.bus.publish(msgs.POSE, msgs.Pose(position=ned_to_enu([m.x, m.y, m.z]), orientation=q, t=stamp), "mavlink")
            self.bus.publish(msgs.TWIST, msgs.Twist(linear=ned_to_enu([m.vx, m.vy, m.vz]), angular=rates, t=stamp), "mavlink")
        elif t == "COMMAND_ACK":
            self._acks[m.command] = m.result
            self._ack_event.set()

    def wait_connected(self, timeout: float = 10.0) -> bool:
        """Block until the autopilot's heartbeat has been seen."""
        end = time.monotonic() + timeout
        while not self.connected and time.monotonic() < end:
            self._send(lambda c: c.mav.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            time.sleep(0.1)
        return self.connected

    def _client_command(self, name, timeout: float = 3.0, **params) -> bool:
        mav = self.mav
        table = {
            "arm": (mav.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0]),
            "disarm": (mav.MAV_CMD_COMPONENT_ARM_DISARM, [0, 21196 if params.get("force") else 0, 0, 0, 0, 0, 0]),
            "takeoff": (mav.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, float("nan"), float("nan"), float("nan"), float(params.get("altitude") or 0)]),
            "land": (mav.MAV_CMD_NAV_LAND, [0, 0, 0, float("nan"), float("nan"), float("nan"), 0]),
        }
        if name not in table:
            raise ValueError(f"unknown vehicle command {name!r}; use arm, disarm, takeoff or land")
        cmd, p = table[name]
        if name == "takeoff" and self._peer_px4 and params.get("altitude") and self._amsl is not None:
            p[6] = self._amsl + float(params["altitude"])      # PX4 reads the takeoff altitude as above sea level
        if name in ("land", "disarm"):
            # Stop repeating the last position target, or it would fight the landing.
            self._stream_from = self.bus.count(msgs.CMD_SETPOINT) + 1
        self._acks.pop(cmd, None)
        self._ack_event.clear()
        end, tries = time.monotonic() + timeout, 0
        while cmd not in self._acks and time.monotonic() < end:
            # UDP loses packets: resend (with the MAVLink confirmation counter) until acknowledged.
            n = tries
            self._send(lambda c: c.mav.command_long_send(c.target_system or 1, c.target_component or 1, cmd, min(n, 255), *p))
            tries += 1
            resend = min(time.monotonic() + 1.0, end)
            while cmd not in self._acks and time.monotonic() < resend:
                self._ack_event.wait(0.05)
                self._ack_event.clear()
        return self._acks.get(cmd) == mav.MAV_RESULT_ACCEPTED

    def _client_step(self, t: float):
        if self._due("hb", t, 1.0):
            self._send(lambda c: c.mav.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID, 0, 0, 0))
        sp = self.bus.latest(msgs.CMD_SETPOINT)
        if sp is None or self.bus.count(msgs.CMD_SETPOINT) < self._stream_from or not self._due("sp", t, min(self.rate, 20.0)):
            return
        ms = int(t * 1000) & 0xFFFFFFFF
        frame = self.mav.MAV_FRAME_LOCAL_NED
        yaw = math.pi / 2 - sp.yaw
        if self._peer_px4 and self.armed and (self._peer_mode >> 16) & 0xFF != 6:
            # PX4 follows position targets only in OFFBOARD mode, and only accepts that
            # mode once targets are already arriving: ask for it a moment after they start.
            if self._offboard_from is None:
                self._offboard_from = t + 0.5
            elif t >= self._offboard_from and self._due("offboard", t, 1.0):
                self._send(lambda c: c.mav.command_long_send(c.target_system or 1, c.target_component or 1, self.mav.MAV_CMD_DO_SET_MODE, 0,
                                                             self.mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0))
        else:
            self._offboard_from = None
        if sp.position is not None:
            p = enu_to_ned(sp.position)
            mask = 0b0000_1001_1111_1000       # position + yaw
            self._send(lambda c: c.mav.set_position_target_local_ned_send(ms, c.target_system or 1, c.target_component or 1, frame, mask,
                                                                          p[0], p[1], p[2], 0, 0, 0, 0, 0, 0, yaw, 0))
        elif sp.velocity is not None:
            v = enu_to_ned(sp.velocity)
            mask = 0b0000_1001_1100_0111       # velocity + yaw
            self._send(lambda c: c.mav.set_position_target_local_ned_send(ms, c.target_system or 1, c.target_component or 1, frame, mask,
                                                                          0, 0, 0, v[0], v[1], v[2], 0, 0, 0, yaw, 0))

    # =====================================================================
    # hil (PX4 SITL lockstep)
    # =====================================================================
    def _rx_hil(self, conn, m):
        if m.get_type() == "HIL_ACTUATOR_CONTROLS":
            self._controls = np.array(m.controls, float)
            self._controls_event.set()

    def _px4_motor_map(self):
        """Which PX4 output drives each of the model's rotors.

        PX4 numbers a quadrotor X as front-right, rear-left, front-left, rear-right,
        with the first two spinning counter-clockwise. The model's order and layout
        are read from the physics backend, so any X quadrotor works unmapped."""
        alloc = getattr(self.app.physics, "allocation", None)
        n = len(getattr(self.app.physics, "actuator_names", []))
        if not alloc or n != 4:
            return list(range(n or 16))
        M = np.asarray(alloc["matrix"])                 # rows: Fz, roll, pitch, yaw torque (body FLU)
        px4 = {(True, False): 0, (False, True): 1, (True, True): 2, (False, False): 3}     # (front, left)
        order = [px4[(bool(M[2, i] < 0), bool(M[1, i] > 0))] for i in range(4)]
        if sorted(order) != [0, 1, 2, 3]:
            return list(range(4))                       # not an X layout: leave the order alone
        # A counter-clockwise rotor twists the body clockwise: negative yaw torque with z up.
        if any((M[3, i] < 0) != (order[i] in (0, 1)) for i in range(4)):
            print("roboforge: this model's rotors spin the other way round from PX4's quadrotor X; yaw will be reversed. Flip the yaw gear signs or set motor_map.", flush=True)
        return order

    def _hil_step(self, t: float):
        imu, pose, twist = self.bus.latest(msgs.IMU), self.bus.latest(msgs.POSE), self.bus.latest(msgs.TWIST)
        if imu is None or pose is None or twist is None:
            return
        if not self._hil_connected:
            # The autopilot connects to us. Nothing can be sent until it has, so wait for it.
            end = time.monotonic() + self.connect_timeout
            while not any(getattr(c, "port", None) is not None for c in self.conns):
                if time.monotonic() > end:
                    raise TimeoutError(f"no autopilot connected on {self.urls[0]} within {self.connect_timeout:g}s. Start PX4 SITL so it connects to this simulator.")
                time.sleep(0.02)
            self._hil_connected = True
        if not self._due("sensor", t, self.sensor_rate):
            return          # between sensor samples the last motor command stays in force
        # PX4 takes its clock from these timestamps and rejects a sample stamped 0.
        usec = int(t * 1e6) + 1_000_000
        R = attitude_enu_to_ned(pose.orientation)
        acc, gyro, mag = flu_to_frd(imu.accel), flu_to_frd(imu.gyro), R.T @ MAG_NED
        alt = self.home[2] + float(pose.position[2])
        pressure = 1013.25 * (1.0 - 2.25577e-5 * alt) ** 5.25588
        if self.sensor_noise:
            # An autopilot treats a reading that never changes as a stuck sensor.
            k, rng = self.sensor_noise, self._rng
            acc, gyro, mag = acc + rng.normal(0, 0.02 * k, 3), gyro + rng.normal(0, 0.002 * k, 3), mag + rng.normal(0, 0.002 * k, 3)
            pressure += rng.normal(0, 0.01 * k)
            alt_baro = alt + rng.normal(0, 0.08 * k)
        else:
            alt_baro = alt
        v = enu_to_ned(twist.linear)
        lat, lon = self._latlon(pose.position)
        self._controls_event.clear()
        self._send(lambda c: c.mav.hil_sensor_send(usec, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2],
                                                   pressure, 0.0, alt_baro, 20.0, 0x1FFF, 0))
        if self._due("gps", t, 10.0):
            speed = float(np.hypot(v[0], v[1]))
            cog = int(math.degrees(math.atan2(v[1], v[0])) % 360 * 100)
            self._send(lambda c: c.mav.hil_gps_send(usec, 3, int(lat * 1e7), int(lon * 1e7), int(alt * 1000), 30, 40, int(speed * 100),
                                                    int(v[0] * 100), int(v[1] * 100), int(v[2] * 100), cog, 12))
        if self._controls is None:
            # PX4 is still booting: it needs a stream of samples to move its clock before
            # it sends the first motor command, so do not block on an answer yet.
            return
        # Lockstep: do not advance until the autopilot has answered this sample.
        if not self._controls_event.wait(self.lockstep_timeout):
            self.late_answers += 1
            self._late_in_a_row += 1
            if self._late_in_a_row >= 5:
                raise ConnectionError(f"the autopilot stopped answering on {self.urls[0]} (no motor command for {5 * self.lockstep_timeout:.0f}s). Did PX4 exit?")
            return          # a late answer: keep the previous motor command for this step
        self._late_in_a_row = 0
        u = self._controls
        if self.motor_map is None:
            self.motor_map = self._px4_motor_map()
        u = u[self.motor_map]
        n = len(getattr(self.app.physics, "actuator_names", [])) or len(u)
        self.bus.publish(msgs.CMD_ACTUATORS, msgs.ActuatorCommand(values=np.clip(u[:n], 0.0, 1.0)), "mavlink")

    # --- loop hooks ----------------------------------------------------------
    def before_step(self, t: float):
        if self.mode == "hil":
            self._hil_step(t)

    def after_step(self, t: float):
        if self.mode == "vehicle":
            self._vehicle_step(t)
        elif self.mode == "client":
            self._client_step(t)

    def close(self):
        self._stop = True
        for th in self._threads:
            th.join(timeout=1.0)
        for c in self.conns:
            try:
                c.close()
            except Exception:
                pass
        self.conns = []

    def describe(self):
        return {**super().describe(), "mode": self.mode, "urls": list(self.urls)}
