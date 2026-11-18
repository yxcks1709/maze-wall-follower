

# Maze Wall-Follower Robot 🤖

An autonomous robot that navigates a maze using the **left-hand rule**, built with a ROBOTIS OpenCM 9.04 board, three IR sensors and two Dynamixel motors.

---

## Hardware

| Component | Details |
|---|---|
| Controller | ROBOTIS OpenCM 9.04 |
| Motors | 2× Dynamixel in wheel mode (IDs 14 and 15) |
| Sensors | 3× OLLO IR sensors — front (port 1), left (port 2), right (port 3) |

## How it works

On every loop the robot reads the three IR sensors and takes one of three decisions:

```
            ┌────────────────────┐
            │  Read F / L / R IR  │
            └─────────┬──────────┘
                      │
      ┌───────────────┼──────────────────────┐
      ▼               ▼                      ▼
 Wall in front?   Left wall ended?       Otherwise
      │           (confirmed 5 reads)        │
      ▼               ▼                      ▼
 Both sides busy  Drive past corner,    Drive forward,
 → U-turn         turn 90° left,        correcting distance
 Else → turn to   enter new corridor    to the walls
 the freer side
```

1. **Wall in front** → stop and compare how much space there is on each side (using each sensor's baseline value). Turn 90° toward the freer side, or turn 180° if both sides are blocked.
2. **Gap on the left** → if the robot was following a left wall and it disappears for 5 consecutive readings, it drives past the corner, turns 90° left and enters the new corridor.
3. **Clear path** → drive forward with small speed corrections:
   - too close to the left wall → steer right
   - too close to the right wall → steer left
   - drifting away from the left wall → steer gently back

## Challenges & what I learned

- **Sensor calibration:** each IR sensor has a different baseline reading with nothing in front (left ≈ 162, right ≈ 29), so thresholds had to be tuned per sensor and decisions compare *space relative to the baseline*, not raw values.
- **Motor drift:** the robot naturally veered to one side, so the right wheel gets a small speed offset (`AJUSTE_DER`).
- **Noisy readings:** a single low reading could trigger a false turn; requiring 5 consecutive "no wall" readings (`CONFIRMAR`) fixed it.
- **Hysteresis zones:** separate "too close" and "too far" thresholds keep the robot centered without zig-zagging.

## Limitations & future improvements

- Turns are **time-based** (`TIEMPO_90`, `TIEMPO_GIRO`), so they depend on battery level and floor friction → use wheel encoders or a gyroscope.
- Turns use blocking `delay()` calls, so the robot does not read sensors while turning → refactor into a non-blocking state machine.
- Wall following uses fixed corrections → replace with a **PID controller** for smoother tracking.

## How to run

1. Install the [ROBOTIS OpenCM IDE](https://emanual.robotis.com/docs/en/software/opencm_ide/getting_started/).
2. Open `wall_follower_robot/wall_follower_robot.ino`.
3. Upload it to the OpenCM 9.04 and open the serial monitor to see live sensor values and decisions.
4. Calibrate `TIEMPO_90`, `TIEMPO_GIRO` and the sensor thresholds for your robot and surface.

## Project structure

```
maze-wall-follower/
├── wall_follower_robot/
│   └── wall_follower_robot.ino   # Robot firmware
├── docs/                         # Photos / videos of the robot
└── README.md
```

---

_Variable names and some identifiers are in Spanish (e.g. `izq` = left, `der` = right, `frente` = front, `pared` = wall)._
