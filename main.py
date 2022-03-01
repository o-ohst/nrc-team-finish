#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
import math
import time
import pid as pidfile
from graphs import *

### initialisation
brick.sound.beep()

color_sensor_left = ColorSensor(Port.S1)
color_sensor_right = ColorSensor(Port.S2)
color_sensor_side_left = ColorSensor(Port.S3)
color_sensor_side_right = ColorSensor(Port.S4)
motor_left = Motor(Port.A,Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.B,Direction.COUNTERCLOCKWISE)
motor_vertical = Motor(Port.C,Direction.COUNTERCLOCKWISE)
motor_rotate = Motor(Port.D,Direction.COUNTERCLOCKWISE)

pid = pidfile.PID()
robot = DriveBase(motor_left,motor_right,80,142.5)

###variables
nodes_stored = 0
current_location = ''
current_direction = 'n'
claw_state = 'u' #u, d
claw_orientation = 'f' #f, b, l, r
orientations = {'r':'e','g':'e','b':'e','y':'s'}
nodes = {'n1':'b','n2':'b','n3':'w','n4':'w','n5':'b','n6':'b'}
cables = {'c1':'','c2':'c'}
boxes = {'r':'','b':'c','g':'','y':''}

###constants for tuning

left_min = 100
right_min = 100
left_max = 0
right_max = 0
ang_turn = 0
ang_around = 0
ref_black_thres = 25
ref_white_thres = 25
forward_collection = True
wait_time = 0.1
coefficients = {}
r_thres, g_thres, b_thres, y_thres = 60.5, 57.5, 60, 63


### basic calculation functions
def distToAngle(dist_in_mm):
    circumf = math.pi * 75
    angle = (dist_in_mm / circumf) * 360 * 0.8915 #TUNING
    return angle

def determineColor(rgb_tuple):

    r = rgb_tuple[0]
    g = rgb_tuple[1] 
    b = rgb_tuple[2]

    # blue (11.1, 29.7, 55.9)
    # yellow (75.8, 65.8, 19.8)
    # red (48.70000000000001, 16.8, 7.3)
    # green (8.199999999999999, 37.3, 10.4)

    if r<2 and g<2 and b<2:
        return None
    elif r > g and r > b: #either r or y
        if r > 3 * g:
            return 'r'
        else:
            return 'y'
    elif b > 3 * g:
        return 'b'
    else:
        return 'g'
        
def dijkstra(graph,src,dest,visited=[],dist={},predecessors={}):
    global dijkstra_runs
    
    if src not in graph:
        raise TypeError('The root of the shortest path tree cannot be found')
    if dest not in graph:
        raise TypeError('The target of the shortest path cannot be found')    
    if src == dest:
        path=[]
        pred=dest
        while pred != None:
            path.append(pred)
            pred=predecessors.get(pred,None)
        path.reverse()
        del path[0]
        return path
    else :     
        if not visited: 
            dist[src]=0
        for neighbor in graph[src] :
            if neighbor not in visited:
                new_distance = dist[src] + graph[src][neighbor]
                if new_distance < dist.get(neighbor,float('inf')):
                    dist[neighbor] = new_distance
                    predecessors[neighbor] = src
        visited.append(src)
        unvisited={}
        for k in graph:
            if k not in visited:
                unvisited[k] = dist.get(k,float('inf'))        
        x=min(unvisited, key=unvisited.get)
        return(dijkstra(graph,x,dest,visited,dist,predecessors))

def turningCode(direction):
    for dire,num in turning_directions[current_direction].items():
        if dire == direction:
            return num

def updateDirection(turning_code):
    global current_direction
    for dire,num in turning_directions[current_direction].items():
            if num == turning_code:
                current_direction = dire
                # print('turned to : '+current_direction)

def updateClawOrientation(turning_code):
    global claw_orientation
    # print(claw_orientation)
    for dire,num in claw_rotating_orientations[claw_orientation].items():
            if num == turning_code:
                claw_orientation = dire
                # print('turned to : '+claw_orientation)

def removeNode(node_removed):
    global nodes
    if node_removed == 'n1':
        nodes['n1'] = 'c'
    elif node_removed == 'n2':
        nodes['n2'] = 'c'
    elif node_removed == 'n3':
        nodes['n3'] = 'c'
    elif node_removed == 'n4':
        nodes['n4'] = 'c'
        distances['1b']['2b'] = 380
        distances['2b']['1b'] = 380
        directions['1b']['2b'] = 'e'
        directions['2b']['1b'] = 'w'
    elif node_removed == 'n5':
        nodes['n5'] = 'c'
        distances['1c']['2c'] = 380
        distances['2c']['1c'] = 380
        directions['1c']['2c'] = 'e'
        directions['2c']['1c'] = 'w'
    elif node_removed == 'n6':
        nodes['n6'] = 'c'
        distances['1d']['2d'] = 380
        distances['2d']['1d'] = 380
        directions['1d']['2d'] = 'e'
        directions['2d']['1d'] = 'w'
    # print('node removed: '+node_removed)

def removeCable(cable_removed):
    global cables
    if cable_removed == 'c1':
        cables['c1'] = 'c'
    elif cable_removed == 'c2':
        cables['c2'] = 'c'
    else:
        pass
        # print('removeCable: error')


### basic robot functions
def ref():
    global left_max, right_max, left_min, right_min
    left = max(min(color_sensor_left.reflection(),left_max),left_min)
    right = max(min(color_sensor_right.reflection(),right_max),right_min)
    calibrated_left_ref = ((left-left_min)/(left_max-left_min))*100
    calibrated_right_ref = ((right-right_min)/(right_max-right_min))*100
    ref = tuple((calibrated_left_ref,calibrated_right_ref))
    return ref

def moveDist(speed,distance):
    ang = distToAngle(distance)
    motor_left.run_angle(speed,ang,Stop.HOLD,False)
    motor_right.run_angle(speed,ang,Stop.HOLD,True)

def moveAngle(speed,angle):
    motor_left.run_angle(speed,angle,Stop.HOLD,False)
    motor_right.run_angle(speed,angle,Stop.HOLD,True)

def stop(stop_type):
    robot.stop(stop_type)

def deccelerate(speed):
    last_speed = speed
    increment = 0
    if abs(speed) <400:
        increment = 20
    else:
        increment = 35
    if speed < 0:
        while True:
            if last_speed - increment >= 0:
                break
            robot.drive(last_speed+increment,0)
            last_speed += increment
    else:
        while True:
            if last_speed - increment <= 0:
                break
            robot.drive(last_speed-increment,0)
            last_speed -= increment

def turn(direction):
    global current_direction
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)
    speed_turn = 300
    ang = ang_around/2

    if not direction in [1,2,0,-1]:
        direction = turningCode(direction)

    if direction == -1:
        # print('left')
        motor_left.run_angle(-speed_turn,ang,Stop.HOLD,False)
        motor_right.run_angle(speed_turn,ang,Stop.HOLD,True)
        time.sleep(wait_time)
        updateDirection(-1)

    elif direction == 1:
        # print('right')
        motor_left.run_angle(speed_turn,ang,Stop.HOLD,False)
        motor_right.run_angle(-speed_turn,ang,Stop.HOLD,True)
        updateDirection(1)

    elif direction == 2:
        motor_left.run_angle(-speed_turn,ang_around,Stop.HOLD,False)
        motor_right.run_angle(speed_turn,ang_around,Stop.HOLD,True)
        updateDirection(2)

    elif direction == 0:
        stop(Stop.HOLD)
    else:
        # print("turn: direction invalid: " + direction) 
        return

def turnAngle(speed, angle, direction):
    speed_turn = speed
    if direction == 'left' or direction == -1:
        speed_turn = -speed_turn
    motor_left.run_angle(speed_turn,angle,Stop.HOLD,False)
    motor_right.run_angle(-speed_turn,angle,Stop.HOLD,True)
    # stop(Stop.HOLD)
    # time.sleep(wait_time)

def junctionTurn(next_direction_or_turning_code,reverse=False): #turn from junction
    global current_direction, turning_directions
    speed_turn = 500
    
    turning_code = 0
    if next_direction_or_turning_code in [1,2,0,-1]:
        turning_code = next_direction_or_turning_code
    else:
        turning_code = turningCode(next_direction_or_turning_code)

    # turn(turning_code)
    angle = 10 #TUNING
    speed = 400

    # moveAngle(300,91)
    # turn(turning_code)
    # return #temp

    if reverse == True:
        if turning_code == -1:
            # moveAngle(-speed,angle) #adjust backwards
            motor_left.run_angle(-speed_turn,ang_turn,Stop.HOLD,True)
            updateDirection(-1)
            # stop(Stop.HOLD)
            # time.sleep(wait_time)

        elif turning_code == 1:
            # moveAngle(speed,angle) #adjust forward
            motor_right.run_angle(-speed_turn,ang_turn,Stop.HOLD,True)
            updateDirection(1)
            # stop(Stop.HOLD)
            # time.sleep(wait_time)

        elif turning_code == 2: 
            motor_left.run_angle(speed_turn,ang_around,Stop.HOLD,False)
            motor_right.run_angle(-speed_turn,ang_around,Stop.HOLD,True)
            updateDirection(2)
            # stop(Stop.HOLD)
            # time.sleep(wait_time)

        else:
            pass
            # print('turning code ineffective')

    else:
        pass

        if turning_code == -1:
            motor_right.run_angle(speed_turn,ang_turn,Stop.BRAKE,True)
            updateDirection(-1)
            stop(Stop.HOLD)
            time.sleep(wait_time)

        elif turning_code == 1:
            motor_left.run_angle(speed_turn,ang_turn,Stop.BRAKE,True)
            updateDirection(1)
            stop(Stop.HOLD)
            time.sleep(wait_time)

        elif turning_code == 2:
            motor_left.run_angle(speed_turn,ang_around,Stop.BRAKE,False)
            motor_right.run_angle(-speed_turn,ang_around,Stop.BRAKE,True)
            updateDirection(2)
            stop(Stop.HOLD)
            time.sleep(wait_time)

        else:
            pass
            # print('turning code ineffective')

    return

def shiftClaw(state,speed_shift=800):
    global claw_state
    m_angle_from_u = 90 #TUNING
    m_angle_from_d = 105 #TUNING
    if state == claw_state:
        return
        # print('no shifting claw needed')
    elif state == 'u':
        motor_vertical.run_until_stalled(speed_shift,Stop.BRAKE,60)
        claw_state = 'u'
        # print('shifted claw to u')
        return
    elif state == 'd':
        motor_vertical.run_until_stalled(-speed_shift,Stop.BRAKE,60)
        claw_state = 'd'
        # print('shifted claw to d')
        return
    elif state == 'm':
        if claw_state == 'u':
            motor_vertical.run_angle(-speed_shift,m_angle_from_u)
            claw_state = 'm'
            # print('shifted claw to m')
        elif claw_state == 'd':
            motor_vertical.run_angle(speed_shift,m_angle_from_d)
            claw_state = 'm'
            # print('shifted claw to m')
    else:
        print('shiftClaw: error')
        return

def rotateClaw(orientation_or_code):
    speed_rotate = 300
    global claw_orientation
    turning_code = 0
    if orientation_or_code in [0,1,-1,2]:
        turning_code = orientation_or_code
    else:    
        for dire,num in claw_rotating_orientations[claw_orientation].items():
                if dire == orientation_or_code:
                    turning_code = num

    # print('turning code is '+str(turning_code))
                
    if turning_code == 0:
        pass
        # print('rotateClaw: no action needed')

    elif turning_code == 1:
        motor_rotate.run_angle(-speed_rotate,90,Stop.HOLD)
        # print('rotateClaw: turned right')

    elif turning_code == 2:
        motor_rotate.run_angle(speed_rotate,180,Stop.HOLD)
        # print('rotateClaw: turned back')
    
    elif turning_code == -1:
        motor_rotate.run_angle(speed_rotate,90,Stop.HOLD)
        # print('rotateClaw: turned left')

    else:
        pass
        # print('rotateClaw: error')

    updateClawOrientation(turning_code)

def moveJunction(speed,no_junctions,sensor='side'): #move for junctions

    junctions_passed = 0
    junction_state = ''
    if color_sensor_side_left.reflection() <= ref_black_thres:
        junctions_passed -= 1
        
    if sensor == 'side':
        while True:
            side_ref = color_sensor_side_left.reflection()
            if side_ref <= ref_black_thres:
                stop(Stop.HOLD)
                time.sleep(wait_time)
                break
            robot.drive(speed,0)
    elif sensor == 'right':
        while True:
            side_ref = color_sensor_right.reflection()
            if side_ref <= ref_black_thres:
                stop(Stop.HOLD)
                time.sleep(wait_time)
                break
            robot.drive(speed,0)
    else:
        while True:
            side_ref = color_sensor_left.reflection()
            if side_ref <= ref_black_thres:
                stop(Stop.HOLD)
                time.sleep(wait_time)
                break
            robot.drive(speed,0)
    return

def moveTime(speed,timez):
    robot.drive_time(speed,0,timez)
    stop(Stop.BRAKE)
    time.sleep(0.1)

def grabNode(node,dest='junction',store=False):
    global nodes_stored
    speed = 400
    speed2 = 300
    wiggle = False

    if nodes_stored == 1:
        return
        wiggle = True
    
    rotateClaw('b')

    shiftClaw('u')
    junctionTurn('w')

    if store == False:
        if wiggle == True:
            shiftClaw('d')
            turnAngle(speed,40,1)
        moveAngle(-speed,30)
        shiftClaw('d')
        moveAngle(-speed,110)
        shiftClaw('m')
        moveAngle(speed,80)
        if wiggle == True:
            turnAngle(speed,30,-1)
        
    else:
        moveAngle(speed2,100)
        shiftClaw('d')
        moveAngle(-speed2,70)

    junctionTurn('s',True)

    # if store == False:
    #     turnAngle(speed,130,'right')
    #     moveAngle(200,90)
    #     shiftClaw('m')
    #     moveAngle(-speed,70)
    #     turnAngle(200,130,'left')
    #     moveAngle(-speed,15)
    # else:
    #     turnAngle(speed,110,'right')
    #     moveAngle(speed,250)
    #     shiftClaw('d')
    #     moveAngle(-speed,250)
    #     turnAngle(speed,110,'left')

    nodes_stored += 1
    removeNode(node)

    lineTrack(200,1,True,False)
    return

def depositNode(color,stay=False,side='x'): #depositNode from junction from x, xlong or y, then and move back to junction (or stay)cv
    global boxes, nodes_stored, claw_state
    
    forward = forward_collection
    desired_orientation = orientations[color] #assuming robot picks up nodes with claw backwards
    current_node_orientation = current_direction
    turning_code = 0
    desired_value = 0 #desired turning code from claw to node orientation
    if forward == False:
        if claw_orientation == 'b':
            desired_value = 2
        elif claw_orientation == 'r':
            desired_value = 1
        elif claw_orientation == 'l':
            desired_value = -1
        else:
            desired_value = 0
    else:
        if claw_orientation == 'f':
            desired_value = 2
        elif claw_orientation == 'l':
            desired_value = 1
        elif claw_orientation == 'r':
            desired_value = -1
        else:
            desired_value = 0
    for key,value in turning_directions[current_direction].items(): #getting current node orientation
        if value == desired_value:
            current_node_orientation = key
    for dire,num in turning_directions[current_node_orientation].items(): #getting turning code
                if dire == desired_orientation:
                    turning_code = num
    thres = 0
    if color == 'r':
        thres = r_thres
    elif color == 'g':
        thres = g_thres
    elif color == 'b':
        thres = b_thres
    else:
        thres = y_thres

    shiftClaw('m')
    rotateClaw(turning_code)

    node_orientation = claw_orientation
    if forward == True:
        for key,value in claw_rotating_orientations[claw_orientation].items():
            if value == 2:
                node_orientation = key

    #linetrack for a distance (alternatively track until box: ang_thres = 0 and 0) #TUNING
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)

    if side == 'x':
        ang_thres = 100 #TUNING -150
    elif side == 'y':
        ang_thres = 0 #TUNING
 
    speed = 100
    setLineTrack(speed,[0.4,0,0.2])
    while (motor_left.angle() + motor_right.angle())/2 <= ang_thres:
        lineTrackUnit(speed)
    stop(Stop.BRAKE)
    time.sleep(wait_time)

    #linetrack to color
    speed2 = 50
    setLineTrack(speed2,[0.15,0,0.3])
    while sum(color_sensor_side_left.rgb()) >= thres:
        lineTrackUnit(speed2)
    stop(Stop.HOLD)
    time.sleep(0.1)

    #putting node
    last_angle = 10000
    move_angle = 20
    move_speed = 50
    wiggle_speed = 200
    wiggle_angle = 0

    if node_orientation not in ['b','f']:
        wiggle_angle = 25

    #adjusting
    # print('node orientation', node_orientation)
    larger = ''

    if node_orientation == 'r':
        moveAngle(200,50)
        motor_vertical.run_angle(600,50)
        # turnAngle(wiggle_speed,wiggle_angle,'right')
        motor_left.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        # motor_right.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
    elif node_orientation == 'l':
        moveAngle(200,50)
        motor_vertical.run_angle(600,50)
        # turnAngle(wiggle_speed,wiggle_angle,'left')
        motor_right.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        # motor_left.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
    else:
        if node_orientation == 'f':
            moveAngle(200,70)
            motor_vertical.run_angle(600,20)
            moveAngle(-wiggle_speed,wiggle_angle)    
        else:
            moveAngle(200,70)
            motor_vertical.run_angle(600,20)
        left, right = ref()
        if right >= left:
            wiggle_angle = (right - left)/3
            turnAngle(wiggle_speed,wiggle_angle,'left')
            # motor_left.run_angle(-wiggle_speed,wiggle_angle,Stop.BRAKE,True)
            larger = 'left'
        else:
            wiggle_angle = (left - right)/3
            turnAngle(wiggle_speed,wiggle_angle,'right')
            # motor_right.run_angle(-wiggle_speed,wiggle_angle,Stop.BRAKE,True)
            larger = 'right'
        
    shiftClaw('d')

    #straightening

    if node_orientation == 'r':
        motor_left.run_angle(-wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        rotateClaw('f')
    elif node_orientation == 'l':
        motor_right.run_angle(-wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        rotateClaw('f')
    else:
        if larger == 'left':
            motor_left.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        elif larger == 'right':
            motor_right.run_angle(wiggle_speed,wiggle_angle,Stop.BRAKE,True)
        else:
            pass

    boxes[color] = 'd'
    nodes_stored -= 1

    # print('claw orientation ',claw_orientation)

    move_speed = 400

    #moving out
    if claw_orientation == 'b':
        moveAngle(move_speed,15)
        shiftClaw('u')
        moveAngle(-move_speed,200)
    elif claw_orientation == 'l': 
        rotateClaw('f')
        moveAngle(-100,150)
    elif claw_orientation == 'r':
        rotateClaw('f')
        moveAngle(-100,150)
    else:#f
        moveAngle(-move_speed,120)
        
    #moving back
    if claw_orientation == 'b':
        if nodes_stored == 1:
            rotateClaw('f')
            shiftClaw('d')
            moveAngle(move_speed,80)
            shiftClaw('m')
            moveAngle(move_speed,-80)
    elif claw_orientation == 'l': 
        rotateClaw('f')
        if nodes_stored == 1:
            moveAngle(move_speed,50)
    elif claw_orientation == 'r':
        rotateClaw('f')
        if nodes_stored == 1:
            moveAngle(move_speed,50)
    else:#f
        if nodes_stored == 1:
            moveAngle(move_speed,40)
            shiftClaw('u')
            moveAngle(-move_speed,130)
            shiftClaw('d')
            moveAngle(move_speed,30)
            shiftClaw('m')

    if stay == True:
        return

    junctionTurn(2)

    rotateClaw('f')
    lineTrack(200,1)

    return

def collectCable(): #move and collect cable from junction and move back to junction
    global claw_state
    speed = 300
    ang = 210
    junctionTurn('e')

    if nodes_stored == 2:
        shiftClaw('d')
        moveAngle(-speed,50)
        shiftClaw('u')
        moveAngle(-speed,ang-50)
        shiftClaw('d')
        shiftClaw('u',200)
        moveAngle(speed,ang-50)
        shiftClaw('d')
        moveAngle(speed,70)
        shiftClaw('m')
        moveAngle(-speed,ang+20)

    else:
        shiftClaw('u')
        moveAngle(-speed,ang)
        if nodes_stored == 0:
            shiftClaw('d')
        else:
            shiftClaw('u')

    moveJunction(100,1)
    return

def depositCable(color,stay=False): #from junction: move towards and turn to deposit cable and go back without turning

    #linetrack for a distance
    original_claw_state = claw_state
    speed = 300
    ang = 0
    if color_sensor_side_left.reflection() <= ref_black_thres:
        # print('on junction')
        ang = 340
    else:
        ang = 140
    setLineTrack(speed,coefficients[speed])
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)
    while (motor_left.angle() + motor_right.angle())/2 <= ang:
        lineTrackUnit(speed)
    stop(Stop.BRAKE)
    time.sleep(wait_time)

    #linetrack to color
    thres = 0
    if color == 'r':
        thres = r_thres
    else:
        thres = y_thres

    speed2 = 50
    setLineTrack(speed2,coefficients[speed2])
    while sum(color_sensor_side_left.rgb()) >= thres:
        lineTrackUnit(speed2)
    stop(Stop.HOLD)
    time.sleep(wait_time)

    #turn
    junctionTurn(2)
    adjustToLine(150)

    #move backward to box
    speed3 = 100
    ang3 = 180
    moveAngle(-speed3,ang3)
    stop(Stop.HOLD)
    time.sleep(wait_time)

    #deposit
    shiftClaw('u',200)
    moveAngle(-200,20)

    #move frward and then linetrack back
    # moveAngle(speed3,ang3)
    if stay == False:
        moveJunction(200,1)
    shiftClaw(original_claw_state)

    return

def setLineTrack(speed,coeffs):
    pid.setKp(coeffs[0])
    pid.setKi(coeffs[1])
    pid.setKd(coeffs[2])
    pid.setWindup(min(speed, 100-speed))

def lineTrackUnit(speed,sensor='both'):
    if speed < 0 :
        right_ref,left_ref = ref()
    else:
        left_ref,right_ref = ref()
    if sensor == 'both':
        e = right_ref - left_ref
    elif sensor == 'left':
        e = 50 - left_ref
    else:
        e = right_ref - 50

    pid.update(e)
    o = pid.output

    robot.drive(speed,o)

def lineTrackAngle(speed,angle):
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)
    if speed < 0:
        setLineTrack(speed,coefficients[-speed])
    else:
        setLineTrack(speed,coefficients[speed])
    while True:
        a = abs(motor_left.angle()+motor_right.angle())/2
        if a >= angle:
            break
        lineTrackUnit(speed)
    stop(Stop.HOLD)
    time.sleep(0.1)
    return

def lineTrack(speed,no_junctions,include_start=False,fast_decceleration=False):
    #tracks line until it passes a number of junctions (detected with the sensor specified) and stops at the last
    
    ref_sum_thres = 115 #TUNING
    slow_speed = 100 #TUNING
    coeffs = coefficients[speed]
    stopp = True
    
    junction_state = ''
    junctions_passed = 0
    e = 0

    pid.setKp(coeffs[0])
    pid.setKi(coeffs[1])
    pid.setKd(coeffs[2])
    pid.setWindup(min(speed, 100-speed))

    if color_sensor_side_left.reflection() <= ref_black_thres:
        # print('starting on junction')
        if include_start == False:
            junctions_passed -= 1
        else:
            if no_junctions == 1:
                stop(Stop.HOLD)
                return
            else:
                pass


    increment = speed/10
    if fast_decceleration == True:
        increment = speed/7
    last_speed = 0

    while True:
        left_ref, right_ref = ref()
        side_ref = color_sensor_side_left.reflection()
        # print(side_ref)

        if no_junctions != 0:

            #exit condition

            if side_ref <= ref_black_thres:
                junction_state = 'on'

            elif side_ref > ref_white_thres :
                if junction_state == 'on': #if robot was but no longer on black
                    # print('passed',junctions_passed+1)
                    junctions_passed += 1
                    junction_state = 'over'

                else:
                    pass
            
            if junctions_passed == no_junctions-1:
                break

        else:
            pass

        

        # if junctions_side == 'both': #using right sensor
            # e = right_ref - 50
        # elif junctions_side == 'left': #using right sensor
        #     e = right_ref - 50
        # elif junctions_side == 'right': #using left sensor
        #     e = 50 - left_ref
        # else:
        #     print('wrong input for junctions_side')

        e = right_ref - left_ref
        pid.update(e)
        o = pid.output

        if last_speed <= speed - increment:
            robot.drive(last_speed+increment,o)
            last_speed += increment
        else:
            robot.drive(speed+increment,o)

    #deccelerating before last junction
    speed2 = speed
    setLineTrack(slow_speed,coefficients[slow_speed])
    if fast_decceleration == False:
        increment = speed/10
        # if speed <= 200:
        #     increment = 1
        # elif speed <= 400:
        #     increment = 3
        # else:
        #     increment = 20
    else:
        increment = speed/6
    while True:
        left_ref, right_ref = ref()
        side_ref = color_sensor_side_left.reflection()
        if side_ref <= ref_black_thres:
            stop(Stop.HOLD)
            break
        e = right_ref - left_ref
        pid.update(e)
        o = pid.output
        robot.drive(speed2,o)
        if speed2 >= slow_speed + increment:
            speed2 -= increment
        else:
            speed2 = slow_speed
    time.sleep(wait_time)
    
    return

def goTo(speed,dest):
    global current_location
    global distances
    global directions

    if current_location == dest:
        # print('already there')
        return

    # print('goTo',current_location,dest)
    include_start = False
    if current_location in ['1ab','1de']:
        include_start = True
    fast_decceleration = False

    path = dijkstra(distances,current_location,dest,[],{},{}) 

    # print('goTo',path)
    path_directions = []
    path_junctions = []
    final_path = []
    
    # print(current_location,path[0])
    path_directions.append(directions[current_location][path[0]])
    path_junctions.append(1)
    last_direction = path_directions[0]
    last_node = current_location

    #finding path
    if len(path)>1:
        for i in range(len(path)):
            if i == len(path)-1:
                final_path.append(path[i])
            else:
                next_direction = directions[path[i]][path[i+1]]
                if next_direction == last_direction:
                    path_junctions[-1] += 1
                else:
                    final_path.append(path[i])
                    path_directions.append(next_direction)
                    path_junctions.append(1)
            last_direction = next_direction
            last_node = path[i]

    elif len(path) == 1:
        final_path = path
    
    if dest in ['r','g','b','y']: #remove last node
        # print(dest)
        # print('Dest is r/g/b/y')
        final_path.pop(-1)
        path_junctions.pop(-1)
        path_directions.pop(-1)

    print('Final path: ',final_path)
    for i in range(len(path_directions)):
        print('Turn to '+path_directions[i])
        print('Line track for '+path_junctions[i]+' junctions')

    #movement
    for i in range(len(final_path)):
        # print('moving to '+str(final_path[i]))
        if current_direction != path_directions[i]:
            junctionTurn(path_directions[i])
        # print('lineTrack ' + str(path_junctions[i]))
        if path_junctions[i] == 1:
            if distances[current_location][final_path[i]] <= 250:
                fast_decceleration == True
            else:
                pass
        lineTrack(speed,path_junctions[i],include_start,fast_decceleration)
        current_location = final_path[i]

    # print("path is " + str(final_path))
    # print("path_directions is " + str(path_directions))
    # print("path_junctions is " + str(path_junctions))

def adjustToLine(ang):
    setLineTrack(100,[0.2,0,0.35])
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)
    while abs(motor_left.angle() + motor_right.angle())/2 <= ang:
        lineTrackUnit(100)
    stop(Stop.HOLD)

def goToCable(speed,cable):
    if cable == 'c1':
        goTo(speed,'1ab')
    else:
        ang = 230
        goTo(speed,'1d')
        if current_direction == 'n':
            moveAngle(-speed,ang)
        else:
            moveAngle(speed,ang)
    junctionTurn('e')
    return

            


### main functions
def start(speed):
    on_junction = False
    while True:
        side_ref = color_sensor_side_left.reflection()
        # print(side_ref)
        if on_junction == True:
            if side_ref >= ref_white_thres:
                # print('start line crossed')
                return
        elif side_ref <= ref_black_thres:
            on_junction = True
        robot.drive(speed,0)

def scanBlocks(speed): #moves forward while scanning 4 blocks, then stop after the last 1
    global orientations
    o = ['e','s','w','n']
    slow_speed = 100
    nothing_thres = 10 #TUNING
    setLineTrack(speed,coefficients[speed])

    for i in range(len(o)):
        while sum(color_sensor_side_right.rgb()) <= nothing_thres: #line track until detects something on the right
            lineTrackUnit(speed)

        # print(determineColor(color_sensor_side_right.rgb()),o[i])
        orientations[determineColor(color_sensor_side_right.rgb())] = o[i]
        # stop(Stop.BRAKE)
        # time.sleep(5)

        while sum(color_sensor_side_right.rgb()) > nothing_thres: #moveDist forward until there's nothing on the right
            robot.drive(speed,0)
            
    return
    
def scanNodes(speed,list_of_nodes,run=1):
    global nodes, current_location
    black_white_thres = 15

    if run==1:

        for i in range(3): #for each junction

            if i == 0:
                last_speed = 0
                increment = 20
                setLineTrack(speed,coefficients[speed])
                while color_sensor_side_left.reflection() >= ref_black_thres: #move forward until junction
                    if last_speed + increment >= speed:
                        lineTrackUnit(speed)
                    else:
                        lineTrackUnit(last_speed+increment)
                        last_speed+= increment
            else:
                setLineTrack(speed,coefficients[speed])
                while color_sensor_side_left.reflection() >= ref_black_thres: #move forward until junction
                    lineTrackUnit(speed)

            # print(sum(color_sensor_side_right.rgb()))
            if sum(color_sensor_side_right.rgb()) >= black_white_thres:
                for item in list_of_nodes:
                    if list_of_nodes[i] == item:
                        nodes[item] = 'w'
                        # print(item,'w')
                    else:
                        nodes[item] = 'b'
                        # print(item,'b')
                # current_location = nodes_locations[list_of_nodes[i]]
                # print('current location ' + current_location)

            while color_sensor_side_left.reflection() <= ref_black_thres: #move past junction
                lineTrackUnit(speed)
            
        deccelerate(speed)
        current_location = '1a'
        # print(nodes)
    
    elif run == 2: #speed is negative

        for i in range(3):

            if i == 0:
                last_speed = 0
                increment = 20
                setLineTrack(speed,coefficients[-speed])
                while color_sensor_side_left.reflection() >= ref_black_thres: #move forward until junction
                    if last_speed + increment <= speed:
                        lineTrackUnit(speed)
                    else:
                        lineTrackUnit(last_speed-increment)
                        last_speed-= increment
                

            else:
                setLineTrack(speed,coefficients[-speed])
                while color_sensor_side_left.reflection() >= ref_black_thres: #move until junction
                    lineTrackUnit(speed)

            # print(sum(color_sensor_side_right.rgb()))
            if sum(color_sensor_side_right.rgb()) >= black_white_thres: #if white
                for item in list_of_nodes:
                    if list_of_nodes[i] == item:
                        nodes[item] = 'w'
                        # print(item,'w')
                    else:
                        nodes[item] = 'b'
                        # print(item,'b')
                # current_location = nodes_locations[list_of_nodes[i]]
                # print('current location ' + current_location)

            while color_sensor_side_left.reflection() <= ref_black_thres: #move past junction
                lineTrackUnit(speed)
                
        deccelerate(speed)
        current_location = '2a'
        # print(nodes)

    return

def collect2Nodes(speed,run=1):#collect 2 nodes in a straight line on x axis
    global nodes_stored, current_location
    list_of_nodes = ['','']
    if nodes['n1'] == 'b' and nodes['n4'] == 'b':
        list_of_nodes = ['n4','n1']
    elif nodes['n2'] == 'b' and nodes['n5'] == 'b':
        list_of_nodes = ['n5','n2']
    elif nodes['n3'] == 'b' and nodes['n6'] == 'b':
        list_of_nodes = ['n6','n3']
    else:
        for key,value in nodes.items():
            if value == 'b':
                if key in ['n1','n2','n3']:
                    list_of_nodes[1] = key
                else:
                    list_of_nodes[0] = key

    # print(list_of_nodes)
            
    if list_of_nodes in node_pairs:
        speed2 = 200
        goTo(speed,nodes_locations[list_of_nodes[0]])
        shiftClaw('u')
        junctionTurn('w')
        lineTrack(speed,1)
        removeNode(list_of_nodes[0])
        rotateClaw('f')
        shiftClaw('d')
        setLineTrack(speed2,coefficients[speed2])
        lineTrackAngle(speed2,220)
        shiftClaw('m')
        current_location = nodes_locations[list_of_nodes[1]]
        nodes_stored = 2
    else:
        speed2 = 200
        goTo(speed,nodes_locations[list_of_nodes[0]])
        junctionTurn('w')
        lineTrack(speed,1)
        shiftClaw('d')
        goTo(speed,nodes_locations[list_of_nodes[1]])
        junctionTurn('s')
        grabNode(list_of_nodes[1])
        nodes_stored += 1
        removeNode(list_of_nodes[0])
        
    if run == 1:
        junctionTurn('n',True)
    else:
        junctionTurn('s',True)


    

def tuning():
    mode = 0
    max_mode = 1
    brick.display.clear()
    
    while True:
        if Button.DOWN in brick.buttons():
            return
        elif Button.LEFT in brick.buttons():
            if mode == 0:
                mode = max_mode
            else:
                mode -= 1
        elif Button.RIGHT in brick.buttons():
            if mode == max_mode:
                mode = 0
            else:
                mode += 1

        if mode == 0:
            # brick.display.clear()
            # brick.display.text('color sensor reflection values')
            left = color_sensor_left.reflection()
            right = color_sensor_right.reflection()
            side_left = color_sensor_side_left.reflection()
            side_right = color_sensor_side_right.reflection()
            brick.display.text('ref: ' + str(left) + ',' + str(right) + '  ' + str(side_left) + ',' + str(side_right))
        elif mode == 1:
            # brick.display.clear()
            # brick.display.text('side color sensor RGB values')
            side_left_rgb = color_sensor_side_left.rgb()
            side_right_rgb = color_sensor_side_right.rgb()
            brick.display.text('rgb: ' + str(int(sum(side_left_rgb))) + ' ' + str(int(side_right_rgb[0])) + ',' + str(int(side_right_rgb[1])) + ',' + str(int(side_right_rgb[2]))) 

def run():
    global current_location, current_direction, claw_orientation, claw_state, left_max, right_max, left_min, right_min, wait_time, boxes, nodes, coefficients, ref_black_thres, ref_white_thres, r_thres, g_thres, b_thres, y_thres, ang_turn, ang_around, nodes_stored, forward_collection

    #tuning
    #for bottom sensors
    left_max, right_max, left_min, right_min = 100, 100, 13, 15 #the median white values and median black values
    #for side sensors. b: w:
    ref_black_thres = 20
    ref_white_thres = 20
    thres = 25
    #solid color values: r:21, g:15, b:20 y:42. thresholds: halfway between 100 and solid color
    r_thres, g_thres, b_thres, y_thres = 84, 43, 78, 70
    ang_turn = 329
    ang_around = 328
    coefficients = {
              600:[1,0,0.1],
              500:[0.11,0,0.1],
              400:[0.1,0,0.06],
              300:[0.1,0,0.07],
              250:[0.12,0,0.05],
              200:[0.1,0,0.05],
              150:[0.13,0,0.05],
              100:[0.15,0,0.05],
              50:[0.07,0,0.08]} #TUNING

    #config
    wait_time = 0
    forward_collection = True
    current_location = '1a'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
    current_direction = 'e'
    claw_state = 'u' 
    claw_orientation = 'f'
    nodes_stored = 0
    fast_speed = 400 #500
    normal_speed = 400 #400
    slow_speed = 300 #200
    very_slow_speed = 200

    #########Test
    depositNode('r')
    return

    #########Code
    start(slow_speed)
    scanBlocks(very_slow_speed)
    lineTrack(normal_speed,1)
    current_location = '7a'
    junctionTurn('w')
    adjustToLine(150)
    current_location = '6a'
    goTo(fast_speed,'2a')
    junctionTurn('s')

    #collect and despoit first 2 nodes
    
    for i in range(3): #for every junction
        # print('i='+str(i))
        if nodes['n4'] in ['b','c'] and nodes['n5'] in ['b','c']:
            break
        lineTrack(slow_speed,1,False,True)
        shiftClaw('d')
        # print(sum(color_sensor_side_right.rgb()))
        if sum(color_sensor_side_right.rgb()) <= thres:
            # print('black node detected')
            nodes[['n4','n5','n6'][i]] = 'b'
            if i == 0:
                grabNode(['n4','n5','n6'][i])
                current_location = '2b'
                junctionTurn('e')
                depositNode('r')
                junctionTurn('s')
            elif i == 1:
                grabNode(['n4','n5','n6'][i])
                current_location = '2c'
                if boxes['r'] == '':
                    goTo(slow_speed,'2b')
                    junctionTurn('e')
                    depositNode('r')
                    junctionTurn('s')
                    lineTrack(slow_speed,1)
                else:
                    lineTrack(slow_speed,1)
                    junctionTurn('e')
                    depositNode('b')
                    # junctionTurn('s')
            else:
                grabNode(['n4','n5','n6'][i])
                current_location = '2d'
                junctionTurn('e')
                depositNode('b')
                # junctionTurn('s') 
        else:#white
            # print('white node detected')
            nodes[['n4','n5','n6'][i]] = 'w'
            if i == 0:
                nodes['n5'] = 'b'
                nodes['n6'] = 'b'
                current_location = '2c'
            elif i == 1:
                nodes['n4'] = 'b'
                nodes['n6'] = 'b'
                current_location = '2d'

    current_location = '2d'

    goTo(normal_speed,'1a')
    junctionTurn('s')

    for i in range(3): #for every junction
        # print('i='+str(i))
        if nodes_stored == 1:
            break
        lineTrack(slow_speed,1,False,False)
        # shiftClaw('d')
        # print(sum(color_sensor_side_right.rgb()))
        if sum(color_sensor_side_right.rgb()) <= thres:
            # print('black node detected')
            nodes[['n1','n2','n3'][i]] = 'b'
            # print(['n1','n2','n3'][i],'b')
            if i == 0:
                grabNode(['n1','n2','n3'][i],'junction')
                current_location = '1b'
                lineTrack(slow_speed,3)
            elif i == 1:
                grabNode(['n1','n2','n3'][i],'junction')
                current_location = '1c'
                lineTrack(slow_speed,2)
            else:
                grabNode(['n1','n2','n3'][i],'junction')
                current_location = '1d'
                lineTrack(slow_speed,1)
        else:#white
            # print('white node detected')
            nodes[['n1','n2','n3'][i]] = 'w'
            if i == 0:
                nodes['n2'] = 'b'
                nodes['n3'] = 'b'
                current_location = '1b'
            elif i == 1:
                nodes['n1'] = 'b'
                nodes['n3'] = 'b'
                current_location = '1c'

    # current_location = '1d'

    # goTo(slow_speed,'1e')
    junctionTurn('e')
    adjustToLine(150)
    lineTrack(normal_speed,5)
    junctionTurn('n')
    lineTrack(slow_speed,1)
    junctionTurn('w')
    depositNode('y',True)
    # goTo(slow_speed,'6b')
    # junctionTurn('w')
    # depositNode('g',True)
 
    moveTime(-600,1400)
    moveAngle(400,10)
    turn(1)
    moveTime(-600,1100)


while True:
    if Button.CENTER in brick.buttons():
        run()
        break

