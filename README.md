# Wall Follow Dashboard

Wall following as a service for the Neoracer, with a live tuning dashboard on port 8081.

One process subscribes to /scan, computes PD steering, publishes AckermannDriveStamped on /drive, and serves the dashboard. Runs as a systemd service that starts on boot, alongside the stock neoracer services.

## Install

From a laptop on the car's network:

```
scp -r neoracer_wallfollow racecar@<car-ip>:~/wallfollow
ssh racecar@<car-ip> 'bash ~/wallfollow/setup.sh'
```

Dashboard: `http://<car-ip>:8081`. Re-running setup.sh updates the code and keeps the car's tuned wallfollow.yaml.

## Dashboard

- Lidar view: scan points, active windows or search arc, measured rays, scroll to zoom
- Live error vs setpoint chart with values in cm and yellow markers at every parameter change
- Tune panel: speed is a 0 to 1 drag slider that applies while dragging, other parameters apply on Apply
- Save and Load write and read wallfollow.yaml on the car. Reset (top bar) re-reads the yaml
- Save log snapshots what is on the live chart to LogN.csv. Load log defaults to the latest save, markers included

## Modes

`mode` in wallfollow.yaml selects the controller. It is switched only by editing the file, then pressing Reset or Load. There is no UI toggle.

- `static`: mirrored side windows at look_angle. Error = right minus left in meters, setpoint 0 keeps the car centered.
- `dynamic`: steers toward the most open heading in a front arc, obstacles inflated by the car's half width. width opens and closes the arc, side_weight biases one side.

All parameters are documented in wallfollow.yaml.

## Safety

The neoracer mux forwards /drive with no software deadman. The transmitter's SWB switch is the physical autonomy gate. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. The speed command is hard capped at 1.0 in code.

## Car specifics

This package is calibrated for the Neoracer: LakiBeam lidar angle mapping, steering sign, speed feedback from /odom, ROS Humble paths. Both signs were verified physically on the car.
