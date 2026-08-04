#!/usr/bin/env python3
"""RACECAR Neo wall-follow service: /scan -> PD steering -> /drive + web dashboard.

Same pattern as the driver's dashboard.py (stdlib HTTP + rclpy) on port 8081.
CAUTION: the neoracer mux forwards /drive with no ROS deadman; the physical
gate is the SWC switch on the Flysky transmitter. The shipped yaml has
speed 0.0 so the car cannot drive until the slider is raised.

Two forms, selected by `mode` in wallfollow.yaml (file edit only, no UI switch):
  static:  mirrored side windows at look_angle. Error = right - left meters.
           Setpoint 0 keeps the car centered between the walls.
  dynamic: steer toward the most open heading in a front arc, with obstacles
           inflated by the car's half-width. side_weight biases one side.

Angles use the student convention (0 = nose, positive = right). On the
neoracer the student angle is minus the raw scan angle (physically verified).
/drive steering is negated on publish: positive on the wire turns this car
left (physically verified; the car steered mirrored without the negation).
Speed feedback comes from /odom.
"""

from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import re
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import yaml

PORT = 8081
BASE = Path(__file__).resolve().parent
YAML_PATH = BASE / 'wallfollow.yaml'
LOG_DIR = BASE / 'logs'

MAX_SPEED = 1.0      # hard throttle cap, same idea as drive_real.set_max_speed
SEARCH_STEP = 2      # degrees between candidate headings (dynamic)
HALF_CLEAR = 0.21    # meters, car half-width plus margin, inflates obstacles
CENTER_BIAS = 0.006  # score penalty per degree off-center, stops ping-pong

_lock = threading.Lock()
_params: dict = {}
_state: dict = {'scan': [], 'hist': [], 'overlay': {}, 'enc_speed': 0.0}
_hist: deque = deque(maxlen=600)
# Full rows behind the live chart. POST /logs/save snapshots this to a csv,
# so a saved log is exactly what was on the screen.
_rows: deque = deque(maxlen=600)
# Parameter-change markers [t, label] shown on the charts, so a run can be
# judged before vs after a tuning change. Saved into logs as "# mark" lines.
_marks: deque = deque(maxlen=40)
_T0 = time.monotonic()


def _now():
    return round(time.monotonic() - _T0, 2)


def _add_mark(label):
    # Coalesce rapid updates (the speed slider fires while dragging).
    if _marks and _now() - _marks[-1][0] < 1.0 and _marks[-1][1].split(' ')[0] == label.split(' ')[0]:
        _marks[-1] = [_now(), label]
    else:
        _marks.append([_now(), label])


def set_max_speed(speed):
    """Clamp throttle to [0, MAX_SPEED]. Every speed that leaves this program
    passes through here, so /drive can never carry more than 1.0."""
    return max(0.0, min(MAX_SPEED, float(speed)))


def load_params():
    with _lock:
        before = dict(_params)
        _params.update(yaml.safe_load(YAML_PATH.read_text()))
        _params['speed'] = set_max_speed(_params['speed'])
        _params.setdefault('lookahead', 3.0)  # older yaml files lack this key
        if before and before != _params:
            _add_mark('yaml load')


def save_params():
    """Rewrite the numeric values in place so the yaml comments survive."""
    text = YAML_PATH.read_text()
    with _lock:
        for k, v in _params.items():
            if k == 'mode':
                continue
            if re.search(rf'(?m)^{k}:', text):
                text = re.sub(rf'(?m)^{k}:\s*[-\d.eE+]+', f'{k}: {v}', text)
            else:
                text += f'{k}: {v}\n'
    YAML_PATH.write_text(text)


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wallfollow')
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self._last_error = 0.0
        self._enc = 0.0

    def _odom_cb(self, msg):
        self._enc = msg.twist.twist.linear.x

    def _mean_at(self, msg, deg, window):
        """Average range (m) in a `window`-deg cone at `deg` (0 = nose, + = right)."""
        n = len(msg.ranges)
        center = round((math.radians(-deg) - msg.angle_min) / msg.angle_increment)
        half = max(1, round(math.radians(window / 2) / msg.angle_increment))
        vals = [msg.ranges[(center + k) % n] for k in range(-half, half + 1)]
        vals = [r for r in vals if msg.range_min < r < msg.range_max]
        # No return means open space, not zero. Zero would pull toward gaps.
        return sum(vals) / len(vals) if vals else msg.range_max

    def _scan_cb(self, msg):
        with _lock:
            p = dict(_params)

        la = p['lookahead']
        if p['mode'] == 'dynamic':
            degs = list(range(-int(p['width']), int(p['width']) + 1, SEARCH_STEP))
            dists = [min(self._mean_at(msg, d, p['window']), la) for d in degs]
            # An obstacle at d blocks every heading within atan(HALF_CLEAR/d)
            # of it. The car body would sideswipe it there.
            corridor = list(dists)
            for j, d in enumerate(dists):
                if d >= la:
                    continue
                block = math.degrees(math.atan2(HALF_CLEAR, d))
                span = int(block // SEARCH_STEP) + 1
                for i in range(max(0, j - span), min(len(degs), j + span + 1)):
                    if abs(degs[i] - degs[j]) <= block and d < corridor[i]:
                        corridor[i] = d
            best = max(range(len(degs)), key=lambda i: corridor[i]
                       - CENTER_BIAS * abs(degs[i]) + p['side_weight'] * degs[i])
            error = degs[best] / p['width']
            overlay = {'mode': 'dynamic', 'width': p['width'], 'target_deg': degs[best], 'la': la}
        else:
            left = min(self._mean_at(msg, -p['look_angle'], p['window']), la)
            right = min(self._mean_at(msg, p['look_angle'], p['window']), la)
            error = right - left  # >0: drifted left -> steer right
            overlay = {'mode': 'static', 'look_angle': p['look_angle'], 'window': p['window'],
                       'left': round(left, 2), 'right': round(right, 2), 'la': la}

        cmd = p['kp'] * error + p['kd'] * (error - self._last_error)
        self._last_error = error
        cmd = max(-1.0, min(1.0, cmd))

        out = AckermannDriveStamped()
        out.drive.speed = set_max_speed(p['speed'])
        out.drive.steering_angle = float(-cmd)  # wire positive = left on this car
        self._pub.publish(out)

        t = _now()
        _rows.append(f'{t},{error:.4f},{cmd:.4f},{p["speed"]},{self._enc:.3f}\n')
        _hist.append([t, round(error, 4)])
        overlay.update({'error': round(error, 4), 'steer': round(cmd, 3)})
        scan = [[round(-math.degrees(msg.angle_min + i * msg.angle_increment), 1),
                 round(msg.ranges[i], 3)]
                for i in range(0, len(msg.ranges), 3)
                if msg.range_min < msg.ranges[i] < msg.range_max]
        with _lock:
            _state.update({'scan': scan, 'overlay': overlay, 'hist': list(_hist),
                           'marks': list(_marks), 'enc_speed': round(self._enc, 3)})


def _log_number(name):
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = re.match(r'Log(\d+)\.csv$', name)
    return int(m.group(1)) if m else 0


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self._send((BASE / 'wallfollow.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path == '/state':
            with _lock:
                body = json.dumps({**_state, 'params': _params})
            self._send(body.encode(), 'application/json')
        elif self.path == '/logs':
            names = sorted((f.name for f in LOG_DIR.glob('*.csv')), key=_log_number)
            self._send(json.dumps(names).encode(), 'application/json')
        elif self.path.startswith('/logs/'):
            f = LOG_DIR / Path(self.path).name
            if f.is_file():
                self._send(f.read_bytes(), 'text/csv')
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/params':
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            with _lock:
                changed = []
                for k in ('speed', 'kp', 'kd', 'window', 'lookahead',
                          'look_angle', 'width', 'side_weight'):
                    if k in data:
                        v = float(data[k])
                        if k == 'speed':
                            v = set_max_speed(v)
                        if _params.get(k) != v:
                            _params[k] = v
                            changed.append(f'{k} {v:g}')
                if changed:
                    _add_mark(', '.join(changed))
        elif self.path == '/logs/save':
            LOG_DIR.mkdir(exist_ok=True)
            latest = max((_log_number(f.name) for f in LOG_DIR.glob('Log*.csv')), default=0)
            name = f'Log{latest + 1}.csv'
            rows = list(_rows)
            tmin = float(rows[0].split(',')[0]) if rows else 0.0
            with open(LOG_DIR / name, 'w') as f:
                f.write('t,error,steer,speed_cmd,speed_ms\n')
                f.writelines(f'# mark,{t},{label}\n' for t, label in list(_marks) if t >= tmin)
                f.writelines(rows)
            self._send(name.encode(), 'text/plain')
            return
        elif self.path == '/save':
            save_params()
        elif self.path == '/load':
            load_params()
        else:
            self.send_error(404)
            return
        self._send(b'ok', 'text/plain')

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def main():
    load_params()
    rclpy.init()
    node = WallFollowNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f'Wall-follow dashboard on http://0.0.0.0:{PORT} (mode={_params["mode"]})')
    HTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
