#!/usr/bin/env pybricks-micropython
# -*- coding: utf-8 -*-
 
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
 
# -----------------------------------------------------
try:
    import urandom as _rnd
except ImportError:
    import random as _rnd
 
 
def rand01():
    return _rnd.getrandbits(24) / 16777216.0
 
 
def uniform(a, b):
    return a + (b - a) * rand01()
 
 
def choice(seq):
    return seq[_rnd.getrandbits(16) % len(seq)]
 
 
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C
COLOR_PORT = Port.S3
ULTRA_PORT = Port.S4
 
WHEEL_DIAMETER = 56
AXLE_TRACK = 114
 
LOOP_MS = 50           
RUN_S = 120             
WAIT_FOR_BUTTON = True 
CALIBRATE = True       
CAL_POINTS = 3          
CAL_SAMPLES = 5   
 
TAPE = 25
SPEED = 150          
REVERSE_MM = 200        
                       
TURN_DEG = (90, 180)  
TURN_RATE = 120        
ESCAPE_LEFT_PROB = 0.75 
                        
ESCAPE_TIMEOUT_S = 4.0  
 
# --- avoid a robot (rank 2) ---
ROBOT_MM = 250          
                        
                        
CONFIRM = 2             
AVOID_MIN_S = 0.6    
 
# --- wander (rank 3) ---
WANDER_PERIOD_S = (3, 6)    
WANDER_TURN_DEG = (30, 90)  
WANDER_CURVE = 0            
                           
 
USE_PRESENCE = False
PRESENCE_MM = 400
SLOW = 50
 
ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
eye = ColorSensor(COLOR_PORT)          
eyes = UltrasonicSensor(ULTRA_PORT)   
 
 
def turn_cmd(sign):
    return (0, sign * TURN_RATE)
 
 
# ===========================================================================
# ===========================================================================
def escape(s, st):
    state = st.get('escape', 'WANDER')
 
    if state == 'WANDER':
        if s['reflection'] < TAPE:                
            st['escape'] = 'REVERSE'
            st['mark'] = s['odo_mm']
            st['esc_t0'] = s['t']
            return (-SPEED, 0)
        return None                             
 
    if state == 'REVERSE':
        done = st['mark'] - s['odo_mm'] >= REVERSE_MM        
        timeout = s['t'] - st['esc_t0'] > ESCAPE_TIMEOUT_S
        if done or timeout:
            st['escape'] = 'TURN'
            st['mark'] = s['odo_deg']
            st['esc_t0'] = s['t']
            side = -1 if rand01() < ESCAPE_LEFT_PROB else 1
            st['turn'] = side * uniform(*TURN_DEG)
            return turn_cmd(side)
        return (-SPEED, 0)
 
    if state == 'TURN':
        side = 1 if st['turn'] > 0 else -1
        done = abs(s['odo_deg'] - st['mark']) >= abs(st['turn'])
        timeout = s['t'] - st['esc_t0'] > ESCAPE_TIMEOUT_S
        if done or timeout:                      
            st['escape'] = 'WANDER'
            st['last_turn'] = side              
            st['w'] = 'STRAIGHT'
            st['next_change'] = s['t'] + uniform(*WANDER_PERIOD_S)
            return None
        return turn_cmd(side)
 
    st['escape'] = 'WANDER'                       
    return None
 
def avoid(s, st):
    near = s['distance'] < ROBOT_MM
    st['near_n'] = st.get('near_n', 0) + 1 if near else 0
    active = st.get('avoid_until', -1) > s['t']
 
    if st['near_n'] >= CONFIRM and not active:
        st['side'] = st.get('last_turn') or choice((-1, 1))
        st['avoid_until'] = s['t'] + AVOID_MIN_S
        active = True
 
    if active or st['near_n'] >= CONFIRM:
        return turn_cmd(st['side'])
    return None
 
 
def wander(s, st):
    t = s['t']
    if 'next_change' not in st:
        st['w'] = 'STRAIGHT'
        st['next_change'] = t + uniform(*WANDER_PERIOD_S)
 
    if st['w'] == 'TURN':
        side = 1 if st['w_turn'] > 0 else -1
        if abs(s['odo_deg'] - st['w_mark']) >= abs(st['w_turn']):
            st['w'] = 'STRAIGHT'
            st['next_change'] = t + uniform(*WANDER_PERIOD_S)
        else:
            return turn_cmd(side)
 
    if t >= st['next_change']:                
        side = choice((-1, 1))
        st['w'] = 'TURN'
        st['w_mark'] = s['odo_deg']
        st['w_turn'] = side * uniform(*WANDER_TURN_DEG)
        return turn_cmd(side)
 
    speed = SPEED
    if USE_PRESENCE and s['presence'] and s['distance'] < PRESENCE_MM:
        speed = SLOW                              
    return (speed, WANDER_CURVE)
 
 
def referee(behaviours, s, st):
    for b in behaviours:               
        cmd = b(s, st)
        if cmd is not None:
            return cmd, b.__name__
    return (0, 0), 'none'              
 
 
def show(*lines):
    ev3.screen.clear()
    for ln in lines:
        ev3.screen.print(ln)
 
 
def wait_release():
    while ev3.buttons.pressed():
        wait(10)
 
 
def ask(lines_fn):
    wait_release()
    while True:
        show(*lines_fn())
        p = ev3.buttons.pressed()
        if Button.CENTER in p:
            wait_release()
            return Button.CENTER
        if Button.LEFT in p:
            wait_release()
            return Button.LEFT
        wait(100)
 
 
def sample(reader):
    vals = []
    for _ in range(CAL_SAMPLES):
        vals.append(reader())
        wait(30)
    vals.sort()
    return vals[len(vals) // 2]         
 
 
def measure_points(name, hint):
    """CAL_POINTS замеров датчика цвета в разных местах."""
    got = []
    while len(got) < CAL_POINTS:
        n = len(got) + 1
        ask(lambda: ["%s %d/%d" % (name, n, CAL_POINTS),
                     hint,
                     "now: %d" % eye.reflection(),
                     "got: " + str(got),
                     "CENTER = take"])
        show("%s %d/%d" % (name, n, CAL_POINTS), "measuring...")
        got.append(sample(eye.reflection))
        ev3.speaker.beep(800, 80)
    return got
 
 
def calibrate():
    global TAPE, ROBOT_MM
    while True:
        # 1. Пол
        floor = measure_points("FLOOR", "sensor over floor")
        # 2. Лента
        tape = measure_points("TAPE", "sensor over tape")
        b = ask(lambda: ["DARK FLOOR MARK",
                         "seam, NOT tape",
                         "now: %d" % eye.reflection(),
                         "CENTER = take",
                         "LEFT = skip"])
        if b == Button.CENTER:
            show("DARK MARK", "measuring...")
            floor.append(sample(eye.reflection))
            ev3.speaker.beep(800, 80)
 
        tape_max = max(tape)
        floor_min = min(floor)
        thr = (tape_max + floor_min) // 2          
        gap = floor_min - tape_max
        lines = ["floor min: %d" % floor_min,
                 "tape max: %d" % tape_max,
                 "TAPE = %d" % thr]
        if gap < 10:
            lines.append("WARN: gap small!")  
        lines += ["CENTER = accept", "LEFT = redo"]
        if ask(lambda: lines) == Button.CENTER:
            TAPE = thr
            break
 
    ask(lambda: ["SONAR: clear ahead",
                 "dist: %d mm" % eyes.distance(),
                 "should be big",
                 "(<1000 = sees floor)",
                 "CENTER = ok"])
 
    b = ask(lambda: ["SONAR: robot/box",
                     "touching our nose",
                     "dist: %d mm" % eyes.distance(),
                     "CENTER = take",
                     "LEFT = skip"])
    if b == Button.CENTER:
        show("SONAR", "measuring...")
        nose = sample(eyes.distance)
        if nose < 300:
            ROBOT_MM = max(ROBOT_MM, nose + 170)
            ev3.speaker.beep(800, 80)
        else:
            ask(lambda: ["SONAR: %d mm" % nose, "nothing close seen",
                         "ROBOT_MM kept %d" % ROBOT_MM, "CENTER = ok"])
 
    ask(lambda: ["TAPE = %d" % TAPE,
                 "ROBOT_MM = %d" % ROBOT_MM,
                 "",
                 "put robot at start",
                 "CENTER = GO"])
 
 
# ===========================================================================
# Main
# ===========================================================================
def main():
    behaviours = [escape, avoid, wander]  
    st = {}                              
 
    seed_watch = StopWatch()
    if CALIBRATE:
        calibrate()          
    elif WAIT_FOR_BUTTON:
        ask(lambda: ["ARENA", "TAPE = %d" % TAPE, "CENTER = GO"])
    try:
        _rnd.seed(seed_watch.time() * 7919 + eye.reflection() * 31
                  + eyes.distance())
    except Exception:
        pass
 
    log = DataLog('t', 'reflection', 'distance', 'escape_state',
                  'winner', 'speed', 'turn_rate', name='arena_log')
 
    ev3.speaker.beep()
    robot.reset()
    watch = StopWatch()
 
    try:
        while True:
            tick_start = watch.time()
            t = tick_start / 1000
 
            if RUN_S and t >= RUN_S:
                break
 
            s = {
                't': t,
                'reflection': eye.reflection(),
                'distance': eyes.distance(),
                'presence': eyes.presence() if USE_PRESENCE else False,
                'odo_mm': robot.distance(),
                'odo_deg': robot.angle(),
            }
 
            cmd, who = referee(behaviours, s, st)
            robot.drive(cmd[0], cmd[1])
 
            log.log(round(t, 2), s['reflection'], s['distance'],
                    st.get('escape', 'WANDER'), who, cmd[0], cmd[1])
 
            spent = watch.time() - tick_start
            if spent < LOOP_MS:
                wait(LOOP_MS - spent)
    finally:
        robot.stop()
        left_motor.brake()
        right_motor.brake()
        ev3.speaker.beep(200, 300)
 
 
main()
