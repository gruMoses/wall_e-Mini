# Hardware and Channel Mapping

## RC Channel Semantics (Runtime)

Controller/safety expects 5 RC channels:

- `ch1`: left track throttle
- `ch2`: right track throttle
- `ch3`: arm/disarm switch
- `ch4`: Follow Me mode switch
- `ch5`: emergency trigger (latched)

Safety thresholds (default):

- Arm high: `>= 1800us`
- Disarm low: `<= 1200us`
- Follow Me enter: `ch4 >= 1800us` (while armed)
- Follow Me exit: `ch4 <= 1200us`
- Emergency: `ch5 >= 1800us`

## Arduino RC Firmware Mapping

Sketch: `arduino_rc_reader/arduino_rc_reader.ino` (and mirrored top-level copy).

### RC input pins

- CH1 -> `D2`
- CH2 -> `D3`
- CH3 -> `D4`
- CH4 -> `D5`
- CH5 -> `D6`

### Motor driver pins (Model X bridge)

- Left PWM `ENB_L` -> `D10`
- Left dir `IN3_L` -> `D7`
- Left dir `IN4_L` -> `D8`
- Right PWM `ENA_R` -> `D9`
- Right dir `IN2_R` -> `D11`
- Right dir `IN1_R` -> `D12`

### Serial protocol

- Arduino outputs CSV at ~50 Hz:
  - `ch1,ch2,ch3,ch4,ch5`
- Main app sends motor commands:
  - `M,<left_byte>,<right_byte>`

## Motor Byte Conventions

From `pi_app/control/mapping.py`:

- Output range: `0..254`
- Neutral: `126`
- RC deadband around 1500us maps to neutral.
