#!/usr/bin/env python
"""
@file    Optimal_Parameter_Without_Attacking.py
@author  Jian Lou
version 2
"""

import os, sys
import optparse
import subprocess
import random
import time

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools")) # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), "..", "..", "..")), "tools")) # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
# the port used for communicating with your sumo instance
PORT = 8815

NSGREEN = "Grr" 
NSYELLOW = "yrr"
WEGREEN = "rGG" 
WEYELLOW = "ryy"

NSGREEN1 = "GGrr" 
NSYELLOW1 = "yyrr"
WEGREEN1 = "rrGG" 
WEYELLOW1 = "rryy"

Light_Min_IK=30  
Light_Max_IK=120 #the constrains of the length of traffic lights for intersection IK

Light_Min_LJ=30  
Light_Max_LJ=120 #the constrains of the length of traffic lights for intersection LJ

Light_Min_GD=30  
Light_Max_GD=120 #the constrains of the length of traffic lights for intersection GD

Light_Min_FH=30  
Light_Max_FH=120 #the constrains of the length of traffic lights for intersection FH

Light_Min_AC=30  
Light_Max_AC=120 #the constrains of the length of traffic lights for intersection AC

f = open("Optimal_Defense_Without_Attacking.txt", "a")


def vehicle_number(sensor1, sensor2, id_T1, id_S1, sum_sensor1, sum_sensor2): # return the queue length of a lane, sensor1 and sensor2 
    if traci.inductionloop.getLastStepVehicleNumber(sensor1)==0:        # means the sensor name, id_T1 and id_S1 means the id of vehicles 
        id_T1=''                                                        # sum_sensor1 and sum_senor2 means the number of vehicles accross the sensors
    else:                                                               # the function will compute the queue lenght of a lane. Then it will return
        list_T1 = traci.inductionloop.getLastStepVehicleIDs(sensor1)    # a list consisting of the length of the queue, the id of vehicles, the number of 
        for i in range (0, len(list_T1)):                               # vehicles accrossing the sensors 
            if list_T1[i]!=id_T1:        
                id_T1=list_T1[i]
                sum_sensor1+=1
        
    if traci.inductionloop.getLastStepVehicleNumber(sensor2)==0:
        id_S1=''        
    else:
        list_S1 = traci.inductionloop.getLastStepVehicleIDs(sensor2)            
        for i in range (0, len(list_S1)):        
            if list_S1[i]!=id_S1:        
                id_S1=list_S1[i]
                sum_sensor2 += 1
    queue_length = sum_sensor1-sum_sensor2 + 1 
    tmp=[queue_length, id_T1, id_S1, sum_sensor1, sum_sensor2]  
    return tmp

def clock_value(intersection, clock_WE, clock_NS):
    TS = traci.trafficlights.getRedYellowGreenState(intersection) #the current traffic light state
    if TS == NSGREEN:        
        clock_NS = clock_NS + 1
        clock_WE = 0
    else:            
        clock_WE = clock_WE + 1
        clock_NS = 0   #we have got the clock value of the traffic lights
    return [clock_WE, clock_NS, TS]

def clock_value1(intersection, clock_WE, clock_NS):
    TS = traci.trafficlights.getRedYellowGreenState(intersection) #the current traffic light state
    if TS == NSGREEN1:        
        clock_NS = clock_NS + 1
        clock_WE = 0
    else:            
        clock_WE = clock_WE + 1
        clock_NS = 0   #we have got the clock value of the traffic lights
    return [clock_WE, clock_NS, TS]

def controller(intersection, TS, queue_WE, queue_NS, clock_WE, clock_NS, Light_Min, Light_Max, s_WE, s_NS):
    if (queue_WE < s_WE and queue_NS < s_NS) or (queue_WE >= s_WE and queue_NS >= s_NS):
        if TS == WEGREEN and clock_WE > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN)
            clock_WE = 0
        if TS == NSGREEN and clock_NS > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN)
            clock_NS = 0
    elif (queue_WE >= s_WE and queue_NS <s_NS):
        if TS == NSGREEN and clock_NS > Light_Min:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN) 
            clock_NS = 0
        if TS == WEGREEN and clock_WE > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN)
            clock_WE = 0
    else:
        if TS == WEGREEN and clock_WE > Light_Min:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN)
            clock_WE = 0
        if TS == NSGREEN and clock_NS > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN)
            clock_NS = 0
    return [TS, clock_WE, clock_NS]

def controller1(intersection, TS, queue_WE, queue_NS, clock_WE, clock_NS, Light_Min, Light_Max, s_WE, s_NS):
    if (queue_WE < s_WE and queue_NS < s_NS) or (queue_WE >= s_WE and queue_NS >= s_NS):
        if TS == WEGREEN1 and clock_WE > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN1)
            clock_WE = 0
        if TS == NSGREEN1 and clock_NS > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN1)
            clock_NS = 0
    elif (queue_WE >= s_WE and queue_NS <s_NS):
        if TS == NSGREEN1 and clock_NS > Light_Min:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN1) 
            clock_NS = 0
        if TS == WEGREEN1 and clock_WE > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN1)
            clock_WE = 0
    else:
        if TS == WEGREEN1 and clock_WE > Light_Min:
            traci.trafficlights.setRedYellowGreenState(intersection, NSGREEN1)
            clock_WE = 0
        if TS == NSGREEN1 and clock_NS > Light_Max:
            traci.trafficlights.setRedYellowGreenState(intersection, WEGREEN1)
            clock_NS = 0
    return [TS, clock_WE, clock_NS]

def run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight): #the threshold of West-East and North-South directions
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "Scenarios_2/dsc.sumocfg", "--tripinfo-output", "tripinfo.xml",
                                    "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    """execute the TraCI control loop"""
    traci.init(PORT)
    
    s_NS_IK 
    s_WE_IK #the threshold of North-South and West-East direction for intersection IK

    s_NS_LJ 
    s_WE_LJ #the threshold of North-South and West-East direction for intersection LJ

    s_NS_GD 
    s_WE_GD #the threshold of North-South and West-East direction for intersection GD

    s_NS_FH 
    s_WE_FH #the threshold of North-South and West-East direction for intersection FH

    s_NS_AC
    s_WE_AC #the threshold of North-South and West-East direction for intersection IK

    clock_I_IK=0
    clock_K_IK=0
    clock_IK_LJ=0
    clock_L_LJ=0
    clock_G_GD=0
    clock_IK_GD=0
    clock_GD_FH=0
    clock_LJ_FH=0
    clock_A_AC=0
    clock_E_AC=0  #measures the time since the last switch from RED to GREEN of the traffic light for a queue

    queue_I_IK=0
    queue_K_IK=0
    queue_LK_LJ=0
    queue_L_LJ=0
    queue_G_GD=0
    queue_IK_GD=0
    queue_GD_FH=0
    queue_LJ_FH=0
    queue_A_AC=0
    queue_E_AC=0 #queue length of each direction in the system 

    sum1_I_IK_0 = 0
    sum2_I_IK_0 = 0
    sum1_I_IK_1 = 0
    sum2_I_IK_1 = 0
    sum1_K_IK=0
    sum2_K_IK=0
    sum1_IK_LJ_0 = 0
    sum2_IK_LJ_0 = 0
    sum1_IK_LJ_1 = 0
    sum2_IK_LJ_1 = 0
    sum1_L_LJ = 0
    sum2_L_LJ = 0
    sum1_G_GD_0 = 0
    sum2_G_GD_0 = 0
    sum1_G_GD_1 = 0
    sum2_G_GD_1 = 0
    sum1_IK_GD=0
    sum2_IK_GD=0
    sum1_GD_FH_0 = 0
    sum2_GD_FH_0 = 0
    sum1_GD_FH_1 = 0
    sum2_GD_FH_1 = 0
    sum1_LJ_FH = 0
    sum2_LJ_FH = 0
    sum1_A_AC_0 = 0
    sum2_A_AC_0 = 0
    sum1_A_AC_1 = 0
    sum2_A_AC_1 = 0
    sum1_E_AC_0 = 0 
    sum2_E_AC_0 = 0
    sum1_E_AC_1 = 0 
    sum2_E_AC_1 = 0    #count the number of cars acrossing a sensor

    id_V1=''
    id_V2=''
    id_V3=''
    id_V4=''
    id_V5=''
    id_V6=''
    id_V7=''
    id_V8=''
    id_V9=''
    id_V10=''
    
    id_U1=''
    id_U2=''
    id_U3=''
    id_U4=''
    id_U5=''
    id_U6=''
    id_U7=''
    id_U8=''
    id_U9=''
    id_U10=''

    id_T1=''
    id_T2=''
    id_T3=''
    id_T4=''
    id_T5=''
    id_T6=''
    id_S1=''
    id_S2=''
    id_S3=''
    id_S4=''
    id_S5=''
    id_S6=''  #the id of vehicle accrosing a specific sensor
   
    state_IK = ""
    state_LJ = ""
    state_GD = ""
    state_FH = ""
    state_AC = "" #The red yellow state for the system

    step = 0

    total_latency=0
    car_number=0
    car_latency=0
    truck_number=0
    truck_latency=0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        #The first controller IK ~~~~~~~~~~~~~~~~~~~~~~~~
        #First we compute the queue length of West-East direction
        [queue_I_IK_0, id_V1, id_U1, sum1_I_IK_0, sum2_I_IK_0] = vehicle_number("V1", "U1", id_V1, id_U1, sum1_I_IK_0, sum2_I_IK_0)
        [queue_I_IK_1, id_V2, id_U2, sum1_I_IK_1, sum2_I_IK_1] = vehicle_number("V2", "U2", id_V2, id_U2, sum1_I_IK_1, sum2_I_IK_1)           
        queue_I_IK = queue_I_IK_0 + queue_I_IK_1
        #Then we compute the length in North-South direction
        [queue_K_IK, id_T1, id_S1, sum1_K_IK, sum2_K_IK] = vehicle_number("T1", "S1", id_T1, id_S1, sum1_K_IK, sum2_K_IK)
        ##### Here we simulate the attacking of one sensor T1
        sum1_K_IK=0
        queue_K_IK = sum1_K_IK - sum2_K_IK
        #Now we compute the clock value of the traffic lights(value k in the paper)
        [clock_I_IK, clock_K_IK, TS_IK] = clock_value ("IK", clock_I_IK, clock_K_IK)
        #Now we need to design the traffic light control logic
        [TS_IK, clock_I_IK, clock_K_IK] = controller ("IK", TS_IK, queue_I_IK, queue_K_IK, clock_I_IK, clock_K_IK, Light_Min_IK, Light_Max_IK, s_WE_IK, s_NS_IK)
        

        #The second controller LJ ~~~~~~~~~~~~~~~~~~~~~~~~~~
        #First we compute the queue length of West-East direction
        [queue_IK_LJ_0, id_V3, id_U3, sum1_IK_LJ_0, sum2_IK_LJ_0] = vehicle_number("V3", "U3", id_V3, id_U3, sum1_IK_LJ_0, sum2_IK_LJ_0)
        [queue_IK_LJ_1, id_V4, id_U4, sum1_IK_LJ_1, sum2_IK_LJ_1] = vehicle_number("V4", "U4", id_V4, id_U4, sum1_IK_LJ_1, sum2_IK_LJ_1)           
        queue_IK_LJ = queue_IK_LJ_0 + queue_IK_LJ_1
        #Then we compute the length in North-South direction
        [queue_L_LJ, id_T3, id_S3, sum1_L_LJ, sum2_L_LJ] = vehicle_number("T3", "S3", id_T3, id_S3, sum1_L_LJ, sum2_L_LJ)
        #Now we compute the clock value of the traffic lights(value k in the paper)
        [clock_IK_LJ, clock_L_LJ, TS_LJ] = clock_value ("LJ", clock_IK_LJ, clock_L_LJ)
        #Now we need to design the traffic light control logic
        [TS_LJ, clock_IK_LJ, clock_L_LJ] = controller ("LJ", TS_LJ, queue_IK_LJ, queue_L_LJ, clock_IK_LJ, clock_L_LJ, Light_Min_LJ, Light_Max_LJ, s_WE_LJ, s_NS_LJ)
      

        #The 3rd controller GD ~~~~~~~~~~~~~~~~~~~~~~~~
        #First we compute the queue length of West-East direction
        [queue_G_GD_0, id_V5, id_U5, sum1_G_GD_0, sum2_G_GD_0] = vehicle_number("V5", "U5", id_V5, id_U5, sum1_G_GD_0, sum2_G_GD_0)
        [queue_G_GD_1, id_V6, id_U6, sum1_G_GD_1, sum2_G_GD_1] = vehicle_number("V6", "U6", id_V6, id_U6, sum1_G_GD_1, sum2_G_GD_1)           
        queue_G_GD = queue_G_GD_0 + queue_G_GD_1
        #Then we compute the length in North-South direction
        [queue_IK_GD, id_T2, id_S2, sum1_IK_GD, sum2_IK_GD] = vehicle_number("T2", "S2", id_T2, id_S2, sum1_IK_GD, sum2_IK_GD)
        #Now we compute the clock value of the traffic lights(value k in the paper)
        [clock_G_GD, clock_IK_GD, TS_GD] = clock_value ("GD", clock_G_GD, clock_IK_GD)
        #Now we need to design the traffic light control logic
        [TS_GD, clock_G_GD, clock_IK_GD] = controller ("GD", TS_GD, queue_G_GD, queue_IK_GD, clock_G_GD, clock_IK_GD, Light_Min_GD, Light_Max_GD, s_WE_GD, s_NS_GD)


        #The 4th controller FH ~~~~~~~~~~~~~~~~~~~~~~~~
        #First we compute the queue length of West-East direction
        [queue_GD_FH_0, id_V7, id_U7, sum1_GD_FH_0, sum2_GD_FH_0] = vehicle_number("V7", "U7", id_V7, id_U7, sum1_GD_FH_0, sum2_GD_FH_0)
        [queue_GD_FH_1, id_V8, id_U8, sum1_GD_FH_1, sum2_GD_FH_1] = vehicle_number("V8", "U8", id_V8, id_U8, sum1_GD_FH_1, sum2_GD_FH_1)           
        queue_GD_FH = queue_GD_FH_0 + queue_GD_FH_1
        #Then we compute the length in North-South direction
        [queue_LJ_FH, id_T4, id_S4, sum1_LJ_FH, sum2_LJ_FH] = vehicle_number("T4", "S4", id_T4, id_S4, sum1_LJ_FH, sum2_LJ_FH)
        #Now we compute the clock value of the traffic lights(value k in the paper)
        [clock_GD_FH, clock_LJ_FH, TS_FH] = clock_value ("FH", clock_GD_FH, clock_LJ_FH)
        #Now we need to design the traffic light control logic
        [TS_FH, clock_GD_FH, clock_LJ_FH] = controller ("FH", TS_FH, queue_GD_FH, queue_LJ_FH, clock_GD_FH, clock_LJ_FH, Light_Min_FH, Light_Max_FH, s_WE_FH, s_NS_FH)

        #The 5th controller AC ~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #First we compute the queue length of West-East direction
        [queue_A_AC_0, id_V9, id_U9, sum1_A_AC_0, sum2_A_AC_0] = vehicle_number("V9", "U9", id_V9, id_U9, sum1_A_AC_0, sum2_A_AC_0)
        [queue_A_AC_1, id_V10, id_U10, sum1_A_AC_1, sum2_A_AC_1] = vehicle_number("V10", "U10", id_V10, id_U10, sum1_A_AC_1, sum2_A_AC_1)           
        queue_A_AC = queue_A_AC_0 + queue_A_AC_1
        #Then we compute the length in North-South direction
        [queue_E_AC_0, id_T5, id_S5, sum1_E_AC_0, sum2_E_AC_0] = vehicle_number("T5", "S5", id_T5, id_S5, sum1_E_AC_0, sum2_E_AC_0)
        [queue_E_AC_1, id_T6, id_S6, sum1_E_AC_1, sum2_E_AC_1] = vehicle_number("T6", "S6", id_T6, id_S6, sum1_E_AC_1, sum2_E_AC_1)           
        queue_E_AC = queue_E_AC_0 + queue_E_AC_1
        #Now we compute the clock value of the traffic lights(value k in the paper)
        [clock_A_AC, clock_E_AC, TS_AC] = clock_value1 ("AC", clock_A_AC, clock_E_AC)
        #Now we need to design the traffic light control logic
        [TS_AC, clock_A_AC, clock_E_AC] = controller1 ("AC", TS_IK, queue_A_AC, queue_E_AC, clock_A_AC, clock_E_AC, Light_Min_AC, Light_Max_AC, s_WE_AC, s_NS_AC)    

        
        total_number=traci.simulation.getArrivedNumber()
        if total_number>=1:
            current_list=traci.simulation.getArrivedIDList()
            for i in range (0, len(current_list)):
                v_id=current_list[i]
                if v_id[0:8]=='flowsI2J' or v_id[0:8]=='flowsG2H' or v_id[0:8]=='flowsA2B':
                    car_number+=1
                    car_latency+=step # it means that the latency
                else:
                    truck_number+=1
                    truck_latency+=step   
        
        step += 1
    average_car_latency=1.0*car_latency/car_number
    average_truck_latency=1.0*truck_latency/truck_number
    #print step
    #print car_number, average_car_latency
    #print truck_number, average_truck_latency
    #a=[s_WE, s_NS, car_number, average_car_latency, truck_number, average_truck_latency]
    average_latency= (car_latency + (1.0*Weight)*truck_latency)/(car_number+(1.0*Weight)*truck_number)
    #a.append(average_latency)
    #print >>f, a
    
    traci.close()
    #sys.stdout.flush()
    sumoProcess.kill()
    return average_latency

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true", default=True, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    #generate_routefile()

    Current_Time1 = time.time()
    print time.time()
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    
    s_NS_IK=1 
    s_WE_IK=7 #the threshold of North-South and West-East direction for intersection IK

    s_NS_LJ=1 
    s_WE_LJ=7 #the threshold of North-South and West-East direction for intersection LJ

    s_NS_GD=1 
    s_WE_GD=7 #the threshold of North-South and West-East direction for intersection GD

    s_NS_FH=1 
    s_WE_FH=7 #the threshold of North-South and West-East direction for intersection FH

    s_NS_AC=1
    s_WE_AC=7 #the threshold of North-South and West-East direction for intersection IK
    
    Weight = 6 #the weight of firetrucks, it can be changed
    average_latency = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight) #the optimal value works as start point
    current_latency = average_latency
    arr = [(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC)]
    flag = 1
    while flag == 1:
        for i in range(1, 6, 1): #It is for the NS direction of IK
            try:
                b=arr.index((i, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(i, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((i, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_NS_IK = i
        for j in range(1, 10, 1):
            try:
                b=arr.index((s_NS_IK, j, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, j, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, j, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_WE_IK = j

        for i in range(1, 6, 1): #It is for the NS direction of LJ
            try:
                b=arr.index((s_NS_IK, s_WE_IK, i, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, i, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, i, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_NS_LJ = i
        for j in range(1, 10, 1):
            try:
                b=arr.index((s_NS_IK, s_WE_IK, j, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, j, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, j, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_WE_LJ = j       
                     
        for i in range(1, 6, 1): #It is for the NS direction of GD
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, i, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, i, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, i, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_NS_GD = i
        for j in range(1, 10, 1):
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, j, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, j, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, j, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_WE_GD = j

        for i in range(1, 6, 1): #It is for the NS direction of FH
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, i, s_WE_FH,s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, i, s_WE_FH,s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, i, s_WE_FH,s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_NS_FH = i
        for j in range(1, 10, 1):
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH, j, s_NS_AC, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH, j, s_NS_AC, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH, j, s_NS_AC, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_WE_FH = j
        
        for i in range(1, 6, 1): #It is for the NS direction of AC
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH, i, s_WE_AC))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH, i, s_WE_AC, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH, i, s_WE_AC))
                if tmp < average_latency:
                    current_latency = tmp
                    s_NS_AC = i
        for j in range(1, 10, 1):
            try:
                b=arr.index((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, j))
            except ValueError:
                tmp = run(s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, j, Weight)
                arr.append((s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, j))
                if tmp < average_latency:
                    current_latency = tmp
                    s_WE_AC = j
        

        if current_latency >= average_latency:
            flag = 0
        else:
            average_latency = current_latency

    print >>f, Weight, average_latency, s_NS_IK, s_WE_IK, s_NS_LJ, s_WE_LJ, s_NS_GD, s_WE_GD, s_NS_FH,s_WE_FH,s_NS_AC, s_WE_AC

    Current_Time2 = time.time()

    print >>f, (Current_Time2 - Current_Time1)
