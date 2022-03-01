def calibrate__color__sensors():
    #returns max and min color sensors for manual calibration
    leftmax = 0
    rightmax = 0
    leftmin = 100
    rightmin = 100
    while motor_left.angle() + motor_right.angle() < 1000:
        motor_left.run(300)
        motor_right.run(300)
        leftmin = min(leftmin,color_sensor_left.reflection())
        rightmin = min(rightmin,color_sensor_right.reflection())
        leftmax = max(leftmax,color_sensor_left.reflection())
        rightmax = max(rightmax,color_sensor_right.reflection())

    print("max: "+str(leftmax)+','+str(rightmax)+" ,min: "+str(leftmin)+','+str(rightmin))

def run():
    global current_node, current_direction, claw_orientation, claw_state, left_max, right_max, left_min, right_min, wait_time, boxes, nodes, coefficients, ref_black_thres, ref_white_thres, r_thres, g_thres, b_thres, y_thres, ang_turn, nodes_stored

    #tuning
    #for bottom sensors
    left_max, right_max, left_min, right_min = 100, 100, 9, 10 #the median white values and median black values
    #for side sensors. b: w:
    ref_black_thres = 20
    ref_white_thres = 20
    #solid color values: r:21, g:15, b:20 y:42. thresholds: halfway between 100 and solid color
    r_thres, g_thres, b_thres, y_thres = 60.5, 57.5, 60, 63
    ang_turn = 333
    coefficients = {
              900:[1,0,0.18],
              800:[0.3,0,0.1],
              700:[0.28,0,0.08],
              600:[0.25,0,0.1],
              500:[0.2,0,0.05],
              400:[0.2,0,0.1], #0.2
              300:[0.2,0,0.15],
              200:[0.2,0,0.15],
              50:[0.07,0,0.1]} #TUNING

    #config
    wait_time = 0.05
    current_node = ''
    current_direction = 'e'
    claw_state = 'd'
    claw_orientation = 'f'
    nodes_stored = 0
    fast_speed = 700 #900
    normal_speed = 400 #600
    slow_speed = 200

    motor_left.set_run_settings(1000,800)
    motor_right.set_run_settings(1000,800)

    # #########test
    depositNode('r')
    return


   ###########code
    # start(normal_speed)
    # scanBlocks(slow_speed)
    # lineTrack(normal_speed,1)

    # current_location = '7a'
    # junctionTurn('w')
    # adjustToLine(170)
    # current_location = '6a'
    # goTo(fast_speed,'2a')
    # junctionTurn('s')

    ###collect and despoit first 2 nodes
    
    thres = 15
    for i in range(3): #for every junction
        print(nodes)
        if nodes['n4'] in ['b','c'] and nodes['n5'] in ['b','c']:
            break
        if i == 0:
            lineTrack(normal_speed,1)
        else:
            lineTrack(slow_speed,1)
        shiftClaw('d')
        print(sum(color_sensor_side_right.rgb()))
        if sum(color_sensor_side_right.rgb()) <= thres:
            print('black node detected')
            nodes[['n4','n5','n6'][i]] = 'b'
            if i == 0:
                collectNode(['n4','n5','n6'][i],'box')
                current_node = '2b'
                junctionTurn(2)
                depositNode('r',False,'xlong')
                junctionTurn('s')
            elif i == 1:
                collectNode(['n4','n5','n6'][i])
                current_node = '2c'
                if boxes['r'] == '':
                    junctionTurn(1,True)
                    goTo(slow_speed,'2b')
                    junctionTurn('e')
                    depositNode('r')
                    junctionTurn('s')
                    lineTrack(slow_speed,1)
                else:
                    junctionTurn(-1,True)
                    goTo(slow_speed,'2d')
                    junctionTurn('e')
                    depositNode('b')
                    junctionTurn('s')
            else:
                collectNode(['n4','n5','n6'][i],'box')
                current_node = '2d'
                junctionTurn(2)
                depositNode('b')
                junctionTurn('s') 
        else:
            print('white node detected')
            nodes[['n4','n5','n6'][i]] = 'w'

    return

    ##collect and deposit 1st cable, from 2c/2d
    goTo(normal_speed,'1ab')
    junctionTurn('e')

    collectCable()

    goTo(normal_speed,'2c')
    depositCable('r')

    goTo(normal_speed,'1a')
    junctionTurn('s')

    ###collect last 2 nodes and 2nd cable

    junctions_passed = 0
    for i in range(3):
        if nodes_stored == 2:
            break
        lineTrack(normal_speed,1)
        junctions_passed += 1
        if sum(color_sensor_side_right.rgb()) <= ref_black_thres:
            nodes[['n1','n2','n3'][i]] = 'b'
            if nodes_stored == 0:
                collectNode(['n1','n2','n3'][i],'junction',True)
            elif nodes_stored == 1:
                collectNode(['n1','n2','n3'][i])
            junctionTurn('s',True)

    if junctions_passed == 2:
        current_node = '1c'
    elif junctions_passed == 3:
        current_node = '1d'
    else:
        print('something is seriously wrong')

    goTo(normal_speed,'1e')
    moveAngle(-slow_speed,95)

    junctionTurn('e')
    collectCable() 

    ###deposit cable and nodes
    junctionTurn('s')
    moveAngle(-slow_speed,100)
    junctionTurn('e')
    current_node = '1e'
    goTo(normal_speed,'6c')
    junctionTurn('w')
    depositCable('y')
    goTo(normal_speed,'6b')
    junctionTurn('w')
    depositNode('g')
    goTo(normal_speed,'6d')
    junctionTurn('w')
    depositNode('y',False,'xshort',False)

    moveAngle(-1000,722)
    turn('s')
    moveUntilStalled(1000)

    return

def oldCode():
    moveAngle(normal_speed,30)

    scanNodes(normal_speed,['n1','n2','n3'])
    goTo(normal_speed,'2e')
    junctionTurn('n')
    adjustToLine(150)
    current_location = '2d'
    goTo(normal_speed,'2a')
    junctionTurn('s')
    moveAngle(-normal_speed,120)
    adjustToLine(120)
    scanNodes(slow_speed,['n4','n5','n6'],2)

    collect2Nodes(normal_speed)#
    goToCable(slow_speed,'c1')
    collectCable()
    junctionTurn('s')
    # adjustToLine(100)
    goTo(normal_speed,'2b')
    junctionTurn('e')
    depositNode('r',False,'xlong')
    goTo(slow_speed,'2c')
    junctionTurn('e')
    depositCable('r')
    goTo(slow_speed,'2d')
    junctionTurn('e')
    depositNode('b',False,'xlong')



   def grabNode(node,dest='junction',store=False, wiggle = False): #move and collect node and move back for either junction or box, without turning

    global nodes_stored
    speed = 800 #TUNING
    small_ang = 110 #TUNING
    really_small_ang = 0

    
    if store == False:
        if wiggle == True:
            really_small_ang = 10
            turnAngle(speed,30,1)
        else:
            really_small_ang = 40
        rotateClaw('b')
        moveAngle(-speed,really_small_ang)
        shiftClaw('d')
        moveAngle(-speed,small_ang-really_small_ang)
        shiftClaw('m')
        if wiggle==True:
            turnAngle(speed,30,-1)
    else:
        really_small_ang = 50
        moveAngle(speed,really_small_ang)
        shiftClaw('m')
        moveAngle(-speed,small_ang+really_small_ang)

    
    nodes_stored += 1
    removeNode(node)

    if dest == 'junction':
        moveAngle(speed,small_ang)
    
    return
    

def collectNode(node,dest='junction',store=False, wiggle = False):
    rotateClaw('b')
    shiftClaw('u')
    junctionTurn('w')
    grabNode(node, dest,store)


    # start(normal_speed)
    # scanBlocks(300)
    # lineTrack(normal_speed,1)
    # junctionTurn('w')
    # adjustToLine(140)
    # lineTrack(normal_speed,4)
    # current_location = '1b'
    # junctionTurn('s')

    ###collect and despoit first 2 nodes
    
    # thres = 25
    # for i in range(3): #for every junction
    #     print('i='+str(i))
    #     if nodes['n4'] in ['b','c'] and nodes['n5'] in ['b','c']:
    #         break
    #     lineTrack(very_slow_speed,1,False,True)
    #     shiftClaw('d')
    #     print(sum(color_sensor_side_right.rgb()))
    #     if sum(color_sensor_side_right.rgb()) <= thres:
    #         print('black node detected')
    #         nodes[['n4','n5','n6'][i]] = 'b'
    #         if i == 0:
    #             grabNode(['n4','n5','n6'][i])
    #             current_location = '2b'
    #             junctionTurn('e')
    #             depositNode('r')
    #             junctionTurn('s')
    #         elif i == 1:
    #             grabNode(['n4','n5','n6'][i])
    #             current_location = '2c'
    #             if boxes['r'] == '':
    #                 goTo(very_slow_speed,'2b')
    #                 junctionTurn('e')
    #                 depositNode('r')
    #                 junctionTurn('s')
    #                 lineTrack(slow_speed,1)
    #             else:
    #                 goTo(slow_speed,'2d')
    #                 junctionTurn('e')
    #                 depositNode('b')
    #                 junctionTurn('s')
    #         else:
    #             grabNode(['n4','n5','n6'][i])
    #             current_location = '2d'
    #             junctionTurn('e')
    #             depositNode('b')
    #             # junctionTurn('s') 
    #     else:#white
    #         print('white node detected')
            # nodes[['n4','n5','n6'][i]] = 'w'
            # if i == 0:
            #     nodes['n5'] = 'b'
            #     nodes['n6'] = 'b'
            #     current_location = '2c'
            # elif i == 1:
            #     nodes['n4'] = 'b'
            #     nodes['n6'] = 'b'
            #     current_location = '2d'

    ##collect and deposit 1st cable, from 2c/2d
    # junctionTurn('n')
    # moveTime(-600,1100)
    # moveAngle(300,130)
    # junctionTurn('e')
    # shiftClaw('u')
    # moveAngle(-600,800)
    # shiftClaw('d')
    # lineTrack(very_slow_speed,1,True,True)
    # junctionTurn('n')
    # moveAngle(-slow_speed,60)
    # if nodes['n5'] == 'b' or nodes['n5'] == 'c':
    #     lineTrack(slow_speed,2,True,False)
    #     junctionTurn('e')
    #     lineTrack(slow_speed,1)
    #     depositCable('r')
    # else:
    #     lineTrack(slow_speed,1)
    #     junctionTurn('e')
    #     lineTrack(slow_speed,1)
    #     junctionTurn('n')
    #     lineTrack(very_slow_speed,1)
    #     junctionTurn('e')
    #     depositCable('r')
    # current_location = '2c'

    #collect 2 nodes and 1 cable

    # goTo(slow_speed,'1a')
    # junctionTurn('s')
    # shiftClaw('d')

    # for i in range(3): #for every junction
    #     print('i='+str(i))
    #     if nodes['n4'] in ['b','c'] and nodes['n5'] in ['b','c']:
    #         break
    #     lineTrack(very_slow_speed,1,False,True)
    #     shiftClaw('d')
    #     # print(sum(color_sensor_side_right.rgb()))
    #     if sum(color_sensor_side_right.rgb()) <= thres:
    #         print('black node detected')
    #         nodes[['n1','n2','n3'][i]] = 'b'
    #         if i == 0:
    #             grabNode(['n1','n2','n3'][i],'junction',True)
    #             current_location = '1b'
    #         elif i == 1:
    #             store = False
    #             if nodes_stored == 0:
    #                 store = True
    #             grabNode(['n1','n2','n3'][i],'junction',store)
    #             current_location = '1c'
    #         else:
    #             store = False
    #             if nodes_stored == 0:
    #                 store = True
    #             grabNode(['n1','n2','n3'][i],'junction',store)
    #             current_location = '1d'
    #     else:#white
    #         print('white node detected')
    #         nodes[['n4','n5','n6'][i]] = 'w'
    #         if i == 0:
    #             nodes['n5'] = 'b'
    #             nodes['n6'] = 'b'
    #             current_location = '1b'
    #         elif i == 1:
    #             nodes['n4'] = 'b'
    #             nodes['n6'] = 'b'
    #             current_location = '1c'

    # goTo(slow_speed,'1e')
    # goTo(slow_speed,'6d')
    # junctionTurn('w')
    # depositNode('y')
    # goTo(slow_speed,'6c')
    # junctionTurn('w')
    # depositCable('y')
    # goTo(slow_speed,'6b')
    # junctionTurn('w')
    # depositNode('g',True)

    # moveTime(-600,1600)
    # moveAngle(400,100)
    # turn('n')
    # moveTime(-600,1600)





    # scanNodes(normal_speed,['n1','n2','n3'])
    # goTo(normal_speed,'2e')
    # junctionTurn('s')
    # scanNodes(-slow_speed,['n6','n5','n4'],2)

    # collect2Nodes(slow_speed,1)
    # lineTrack(slow_speed,1,True,False)
    # junctionTurn('e')
    # collectCable()
    # moveAngle(normal_speed,60)
    # moveJunction(normal_speed,1)
    # junctionTurn(1)
    # junctionTurn(-1)
    # lineTrackAngle(-very_slow_speed,300)
    # depositNode('r',True)

    # junctionTurn('s')
    # moveJunction(very_slow_speed,1)
    # junctionTurn('e')
    # lineTrackAngle(-very_slow_speed,300)
    # depositCable('r',True)

    # junctionTurn('s')
    # moveJunction(very_slow_speed,1)
    # junctionTurn('e')
    # lineTrackAngle(-very_slow_speed,300)
    # depositNode('b')

    # current_location = '2d'
    # collect2Nodes()

    #collect and deposit 1st cable, from 2d
    junctionTurn('n')
    moveTime(-600,1100)
    moveAngle(300,130)
    junctionTurn('e')
    shiftClaw('u')
    moveAngle(-600,800)
    shiftClaw('d')
    lineTrack(very_slow_speed,1,True,True)
    junctionTurn('n')
    moveAngle(-slow_speed,60)
    if nodes['n5'] == 'b' or nodes['n5'] == 'c':
        lineTrack(slow_speed,2,True,False)
        junctionTurn('e')
        lineTrack(slow_speed,1)
        depositCable('r')
    else:
        lineTrack(slow_speed,1)
        junctionTurn('e')
        lineTrack(slow_speed,1)
        junctionTurn('n')
        lineTrack(very_slow_speed,1)
        junctionTurn('e')
        depositCable('r')
    current_location = '2c'





#cable 1 code
        # goTo(normal_speed,'1a')
    # junctionTurn('s')
    # turn('e')
    # shiftClaw('u')
    # moveAngle(-slow_speed,120)
    # shiftClaw('d')

    # junctionTurn(1)
    # moveAngle(-slow_speed,60)