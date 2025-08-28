# wall_e-Mini

Raspberry Pi + Arduino RC project.

## PID Debug Mode

The control application can print detailed telemetry from the steering PID
controller.  Enable it either by passing `--pid-debug` when starting the
application or by setting `log_steering_corrections = True` in `config.py`.

```
python3 -m app.main --pid-debug
```

When active, each control loop prints an additional line showing:

```
PID err=<heading_error> P=<proportional> I=<integral> D=<derivative> \
    yaw=<yaw_rate> int=<integral_state>
```

- `err` – current heading error in degrees
- `P`, `I`, `D` – contributions from each PID term
- `yaw` – measured yaw rate in degrees per second
- `int` – accumulated integral error

These values help tune the steering controller and diagnose IMU behaviour.
