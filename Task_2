#!/usr/bin/env pybricks-micropython
# Lab 2: Line follower (EV3 + Pybricks MicroPython) - v5
# Gentle PD edge-following. At sharp corners, if the sensor stays fully on the
# tape for a moment, the robot stops driving forward and pivots in place
# (same direction the controller already wants) so it can't cross the tape.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
sensor = ColorSensor(Port.S4)          # pointing down, ~4 mm above the floor

# Measure your own robot: wheel diameter and distance between wheels (mm)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# ---------- Tuning knobs ----------
BASE_SPEED = 100    # mm/s on straights. Raise once it's stable.
KP = 0.8            # gentle steering: lower = calmer, higher = sharper
KD = 0.5            # small damping only; set to 0 if it twitches
SLOWDOWN = 0.3      # 0 = never slow down, 1 = stop completely at max error
MAX_TURN = 90       # deg/s cap for normal driving
EDGE = -1            # 1 = follow one edge, -1 = the other. Flip if it spins away.

CORNER_TURN = 120   # deg/s pivot speed at a sharp corner
CORNER_LOOPS = 3    # loops fully on tape (3 x 25 ms = 75 ms) before pivoting
# -----------------------------------


def sample_on_button(message):
    """Show a message, wait for centre button, return an averaged reading."""
    ev3.screen.clear()
    ev3.screen.print(message)
    ev3.screen.print("then press CENTRE")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)
    total = 0
    for _ in range(20):                 # average only during calibration
        total += sensor.reflection()
        wait(5)
    while Button.CENTER in ev3.buttons.pressed():
        wait(10)
    return total / 20


# ---------- Two-point calibration (do it on the real track, in the real room) ----------
tape = sample_on_button("Sensor on TAPE")
floor = sample_on_button("Sensor on FLOOR")
target = (tape + floor) / 2             # the edge = half tape, half floor
span = abs(floor - tape) or 1           # avoid divide-by-zero

ev3.screen.clear()
ev3.screen.print("tape  {:.0f}".format(tape))
ev3.screen.print("floor {:.0f}".format(floor))
ev3.screen.print("Place on edge,")
ev3.screen.print("press CENTRE")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
while Button.CENTER in ev3.buttons.pressed():
    wait(10)

# ---------- Main loop ----------
timer = StopWatch()
last_error = 0
dark_loops = 0          # how many loops in a row the sensor has been fully on tape

while Button.CENTER not in ev3.buttons.pressed():       # press CENTRE to stop
    # Error normalised to roughly -50..+50, so gains transfer between rooms
    error = (sensor.reflection() - target) / span * 100

    derivative = error - last_error
    last_error = error

    turn = EDGE * (KP * error + KD * derivative)        # deg/s
    turn = max(-MAX_TURN, min(MAX_TURN, turn))

    # Slow down a little when off the edge, full speed when centred
    speed = BASE_SPEED * (1 - SLOWDOWN * min(abs(error), 50) / 50)

    # Sharp corner: sensor has been fully on the tape for a while.
    # Stop moving forward and pivot the way the controller is already turning,
    # until the sensor reaches the edge again. Never reverses direction.
    if error < -35:
        dark_loops += 1
    else:
        dark_loops = 0

    if dark_loops >= CORNER_LOOPS:
        speed = 0
        turn = CORNER_TURN if turn >= 0 else -CORNER_TURN

    robot.drive(speed, turn)
    wait(25)                                            # ~40 Hz, the sensor's update rate

robot.stop()
ev3.screen.clear()
ev3.screen.print("Time: {:.2f} s".format(timer.time() / 1000))
wait(5000)
