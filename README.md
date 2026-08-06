# Wall Follow Dashboard

Wall following as a service for the Neoracer, with a live tuning dashboard on port 8081.

One process subscribes to /scan, computes PD steering, publishes AckermannDriveStamped on /drive, and serves the dashboard. Runs as a systemd service that starts on boot, alongside the stock neoracer services.

## Contents

- [Install](#install)
- [Service control](#service-control)
- [Dashboard](#dashboard)
- [Modes](#modes)
- [Speed](#speed)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Install

On the car:

```
git clone https://github.com/Neobotics-Foundation-Inc/wallfollow_dashboard.git
bash wallfollow_dashboard/setup.sh
```

setup.sh points neoracer-wallfollow.service at this checkout wherever it sits and copies nothing, so the repository can live anywhere the racecar user can read. A first install leaves the service stopped and disabled; start it with `bash setup.sh enable`. Dashboard: `http://<car-ip>:8081`.

Re-running setup.sh updates the unit, keeps the car's tuned wallfollow.yaml, and leaves the enable state alone: a running service restarts on the new code, a stopped one stays stopped.

## Service control

Run on the car, from the checkout:

| Command | Effect |
| --- | --- |
| `bash setup.sh` | install or update the unit; a first install does not start it |
| `bash setup.sh enable` | start now and at every boot |
| `bash setup.sh disable` | stop now and keep off across boots |
| `bash setup.sh restart` | restart; takes port 8081 back first |
| `bash setup.sh remove` | stop, disable, and uninstall the unit; keeps wallfollow.yaml |

Enable, restart, and an update of a running service clear port 8081 first. A dashboard left over from an earlier install under a different unit name or directory, or any other service on 8081, is stopped through systemd; a `wallfollow.py` started by hand is signalled directly. Without this the new instance would fail to bind and loop on `Restart=on-failure`.

After `remove`, the checkout and its tuned wallfollow.yaml stay in place; `bash setup.sh` reinstalls.

## Dashboard

- Lidar view: scan points, active windows or search arc, measured rays, scroll to zoom
- Live error vs setpoint chart with values in cm and yellow markers at every parameter change
- Tune panel: speed is a 0 to 1 drag slider that applies while dragging, other parameters apply on Apply
- Save and Load write and read wallfollow.yaml on the car. Reset (top bar) re-reads the yaml
- Save log snapshots what is on the live chart to LogN.csv. Load log defaults to the latest save, markers included

## Modes

`mode` in wallfollow.yaml selects the controller: 0 = static, 1 = dynamic. It is switched only by editing the file, then pressing Reset or Load. There is no UI toggle.

- `static`: mirrored side windows at look_angle. Error = right minus left in meters, setpoint 0 keeps the car centered.
- `dynamic`: steers toward the most open heading in a front arc, obstacles inflated by the car's half width. width opens and closes the arc, side_weight biases one side.

All parameters are documented in wallfollow.yaml.

## Speed

The speed slider is a throttle cap, not a constant command. The target speed drops when the wall ahead is inside lookahead and when steering hard, and speed_kp / speed_kd ramp the throttle toward that target. The live throttle shows in the top bar.

## Safety

The neoracer mux forwards /drive with no software deadman. The transmitter's SWB switch is the physical autonomy gate. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. The speed command is hard capped at 1.0 in code.

## Car specifics

This package is calibrated for the Neoracer: LakiBeam lidar angle mapping, steering sign, speed feedback from /odom, ROS Humble paths. Both signs were verified physically on the car.
