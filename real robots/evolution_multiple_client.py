"""  
Minimize Surprise - Evolution - Thymio

If the robot controlled by this controller should be the master robot, the name must contain 'master'.
Consequently, all other robots may not contain 'master' in their names.
"""

import sys
import random
import math
import os.path
import numpy as np
from genetic_population_multiple_real import GeneticPopulation
from state import State, write_csv 
import parameters

from gi.repository import GObject
# imports for robot
from rob import Robot
# imports for connection
from clientME import Client


master = False
turnTimer = 0     # set by HWP, specifies how long robot is turning
turnFlag = False  # set by HWP if robot has to turn
direction = False # set by HWP, specifies turn direction
driveTimer = 0    # set by HWP, specifies how long the robot drives straight to evade wall
reverse = False   # set by HWP, specifies if the evades wall by driving forawrd or backwards
reverseDrivingFlag = False  # set by HWP in case robot has been driving backards for too long
reverseCounter = 0  # tracks how long the robot has been driving backwards
postEvalCounter = 0


def sigmoid(x):
    """ returns the value of the sigmoid function evaluated at all elements of x """
    return 1 / (1 + np.exp(-x))
    
    
def tanh(x):
    """ returns the value of the sigmoid function evaluated at all elements of x """
    return np.tanh(x)


class Controller():
    """
    Controller for thymio simulation using 1+1 evolution  distributed across a master and his clients

    If the robot controlled by this controller should be the master robot, the robot's name must contain 'master'.
    Consequently, all other robots may not contain 'master' in their names.

    Usage:
    Simply call *control* at every time step. Well, after having created an object of course.

    """

    def __init__(self):
        """ initialises values for avoid behavior and logfiles

        Arguments:
        robot -- reference to supervisor or robot instance 
        name -- robot name 
        master -- true iff this instance is the master robot
        emitter -- reference to webots emitter
        receiver -- reference to webots receiver

        """ 

        # create a genetic population
        self.population = GeneticPopulation(self, parameters.EVAL_TIME, parameters.POST_EVAL_TIME, parameters.RE_EVAL_PROB, parameters.EVALS, parameters.SENSORS, parameters.ACTIONS, parameters.HIDDEN_ACTION, parameters.HIDDEN_PRED, parameters.MUT_RATE, tanh, sigmoid, parameters.RE_EVAL_WEIGHT)
            
        if master == True:   
            # init log files
            self.filename = "results/run"
            
            if parameters.enableDataTracking:
                write_csv("results/parameters", "SEP=,")        
                write_csv("results/parameters", "arenaSizeX," + str(parameters.ARENA_X))
                write_csv("results/parameters", "arenaSizeY," + str(parameters.ARENA_Y))
                write_csv("results/parameters", "robotAmount," + str(parameters.ROBOTS))
                write_csv("results/parameters", "maxAge," + str(self.population.maxAge))
                write_csv("results/parameters", "reEval," + str(self.population.reEval))
                write_csv("results/parameters", "postEvalTime," + str(parameters.POST_EVAL_TIME))
                write_csv("results/parameters", "amountSensors," + str(self.population.king.amountSensors))
                write_csv("results/parameters", "amountActions," + str(self.population.king.amountActions))
                write_csv("results/parameters", "amountHiddenAction," + str(self.population.king.amountHiddenAction))
                write_csv("results/parameters", "amountHiddenPrediction," + str(self.population.king.amountHiddenPrediction))
                write_csv("results/parameters", "mutateRate," + str(self.population.mutateRate))
                write_csv("results/parameters", "transferFuncAction,tanh")
                write_csv("results/parameters", "transferFuncPred,sigmoid")
            
                self._log("SEP=,")
                self._log("king,mutant")

            # register evaluation listener for logging
            self.population.set_evaluation_listener(lambda x,y: self._log(str(x) + "," + str(y)))
                        
        
        elif parameters.enableDataTracking: # client
             # init log file for predictions and sensors 
             self.filename = "results/pred"
             
             if not parameters.useLightSensors:
                 write_csv(self.filename, "SEP=,")
                 write_csv(self.filename, "obstacle avoidance,pred0 (t+1),pred1 (t+1),pred2 (t+1),pred3 (t+1),pred4 (t+1),pred5 (t+1),pred6 (t+1),predg0 (t+1),predg1 (t+1),predp (t+1),s0 (t),s1 (t),s2 (t),s3 (t),s4 (t),s5 (t),s6 (t),sg0 (t),sg1 (t), sp(t),m0 selected,m1 selected,m0 real,m1 real")
             else:
                 write_csv(self.filename, "SEP=,")
                 write_csv(self.filename, "obstacle avoidance,pred0 (t+1),pred1 (t+1),pred2 (t+1),pred3 (t+1),pred4 (t+1),pred5 (t+1),pred6 (t+1),predg0 (t+1),predg1 (t+1),predp (t+1),predl0 (t+1),predl1 (t+1),s0 (t),s1 (t),s2 (t),s3 (t),s4 (t),s5 (t),s6 (t),sg0 (t),sg1 (t), sp(t),sl0(t),sl1(i),m0 selected,m1 selected,m0 real,m1 real")
             
    def getMaster(self):
        return master

    def _log(self, line):
        """ writes the line to the logfile """
        write_csv(self.filename, line)

    def emit(self, msg):
        """ sends a message """
        client.sendMessage(msg)

    def receive(self):
        # read buffer of incomming messages
        tmpBuffer = client.readBuffer()
        return(tmpBuffer)

    def control(self):
        """ manage master and client logic

        on a client: calculates motor values and sets them using 1+1 evolution encapsulated in a genetic population
        on the master:  waits for evaluation scores and distributs new genomes

        """        
        if master == True:
            self.control_master()
        else:
            self.control_client()
        
        return True

    def control_master(self):      
        """ wait for evaluation scores and distribute new genome """ 
               
        if self.population.POST_EVAL:                   
            motor = [0,0]
            robot.setMotorValues(motor[0], motor[1])
            return True
        
        # receive fitness values, determine overall fitness, distribute new genomes           
        self.population.execute_master()
        
        # don't move        
        motor = [0,0]
        robot.setMotorValues(motor[0], motor[1])
        return True

    def control_client(self):
        """ calculates motor values and sets them using 1+1 evolution distributed across a master and his clients """
        global turnFlag
        global reverseDrivingFlag
        global postEvalCounter
        
        if postEvalCounter >= parameters.POST_EVAL_TIME:
            motor = [0,0]
            robot.setMotorValues(motor[0], motor[1])
            #return True

        # transform sensor values into a numpy vector
        if parameters.useLightSensors == False:
            sensors = robot.getAllSensors()
        else:
            sensors = robot.getAllSensorsLight()
                                        
        action, pred = self.population.execute_client(sensors)  # this is the line containing the 1+1 evolution magic
        obstacle_avoidance = 0
         
        if action is not None: 
            motor = [action[0][0] * parameters.MAX_SPEED, action[1][0] * parameters.MAX_SPEED] # retrieve the calculated action values
            tmp = motor.copy() # copy motor values for logging purposes
        else:
            motor = [0,0] 
            tmp = motor.copy() # copy motor values for logging purposes
            
        # calculate or execute HWP    
        if not turnFlag:
            self._hwp(motor, sensors) # check if we are likely to hit a wall
            
        # if we beginn new genome reset HWP
        if action is None:
            reverseDrivingFlag = False
            turnFlag = False 
            
        if turnFlag or reverseDrivingFlag:
            obstacle_avoidance = 1
            motor = self.execHWP() # if we are likely to hit a wall, we may want to avoid this
                
        if self.population.POST_EVAL and not parameters.useLightSensors and action is not None: # log values during post-evaluation
            if parameters.enableDataTracking:
                write_csv(self.filename, str(obstacle_avoidance)+","+str(pred[0][-1])+","+str(pred[1][-1])+","+str(pred[2][-1])+","+str(pred[3][-1])
                                      +","+str(pred[4][-1])+","+str(pred[5][-1])+","+str(pred[6][-1])+","+str(pred[7][-1])+","+str(pred[8][-1])
                                      +","+str(pred[9][-1])+","+str(sensors[0][-1])+","+str(sensors[1][-1])+","+str(sensors[2][-1])+","+str(sensors[3][-1])+","
                                      +str(sensors[4][-1])+","+str(sensors[5][-1])+","+str(sensors[6][-1])+","+str(sensors[7][-1])+","+str(sensors[8][-1])+","+str(sensors[9][-1])
                                      +","+str(tmp[0])+","+str(tmp[1])+","+str(motor[0])+","+str(motor[1]))
            postEvalCounter += 1
        if self.population.POST_EVAL and parameters.useLightSensors and action is not None: # log values during post-evaluation
            if parameters.enableDataTracking:
                write_csv(self.filename, str(obstacle_avoidance)+","+str(pred[0][-1])+","+str(pred[1][-1])+","+str(pred[2][-1])+","+str(pred[3][-1])
                                      +","+str(pred[4][-1])+","+str(pred[5][-1])+","+str(pred[6][-1])+","+str(pred[7][-1])+","+str(pred[8][-1])
                                      +","+str(pred[9][-1])+","+str(pred[10][-1])+","+str(pred[11][-1])+","+str(sensors[0][-1])+","+str(sensors[1][-1])+","+str(sensors[2][-1])+","+str(sensors[3][-1])+","
                                      +str(sensors[4][-1])+","+str(sensors[5][-1])+","+str(sensors[6][-1])+","+str(sensors[7][-1])+","+str(sensors[8][-1])+","+str(sensors[9][-1])+","+str(sensors[10][-1])+","+str(sensors[11][-1])
                                      +","+str(tmp[0])+","+str(tmp[1])+","+str(motor[0])+","+str(motor[1]))   
            postEvalCounter += 1

        # set motor values
        robot.setMotorValues(motor[0], motor[1])


    def _hwp(self, motor, sensors):
        """ stop robot if it wants to drive into other robot or wall """ 
        global turnFlag
        global turnTimer
        global direction
        global driveTimer
        global reverse
        global reverseDrivingFlag
        global reverseCounter
        
        setTurnParameters = False
                
        # robot stopped - no HWP necessary
        if motor[0] == 0 and motor[1] == 0:
            return 
                 
        # check if and for how long the bot is driving in reverse            
        if motor[0] < 0 and motor[1] < 0:
            reverseCounter += 1
            # if longer then 9 seconds 
            if reverseCounter >= 90:
                reverseDrivingFlag = True
        else:
            reverseCounter = 0
            reverseDrivingFlag = False
                                             
        # driving - check for too high pressure or leaving the arena
        # 0.25 are 10 blocks of force
        if (sensors[9][0] > 0.45 or sensors[7][0] >= 0.55 or sensors[8][0] >= 0.55): 
            # if he is not willing to turn
            if (motor[0] == motor[1]) or np.abs((motor[0]+motor[1])/(motor[1]-motor[0])) > (1.0/3.0):               
                setTurnParameters = True 
                
        # if the robot sees a wall or another robot it moves away from it
        if sensors[0][0] > 0.35 or sensors[1][0] > 0.4 or sensors[2][0] > 0.4 or sensors[3][0] > 0.4 or sensors[4][0] > 0.35:
            setTurnParameters = True 
            driveTimer = 30
            reverse = True
             
        elif sensors[5][0] > 0.4 or sensors[6][0] > 0.4:          
            setTurnParameters = True    
            driveTimer = 30
            reverse = False
        
        # set up the hwp parameters    
        if setTurnParameters: 
            turnFlag = True
            turnTimer = random.randrange(25, 45)
                   
            if random.randrange(1, 100) < 50:
                direction = True
            else:
                direction = False
    
        
    def execHWP(self):
        """ 
        Backs up and turns the robot
        """      
        global turnTimer
        global turnFlag
        global driveTimer
        

        if driveTimer > 0:
            driveTimer -= 1
            if reverse:
                return[-parameters.MAX_SPEED, -parameters.MAX_SPEED]
            else:
                return[parameters.MAX_SPEED, parameters.MAX_SPEED]    
    
        if turnTimer > 0 and driveTimer == 0:  
            turnTimer -= 1
            if turnTimer == 0:
                turnFlag = False
        
            if direction:
                return[parameters.MAX_SPEED, -parameters.MAX_SPEED]
            else:
                return[-parameters.MAX_SPEED, parameters.MAX_SPEED]
                
        # if the bot drove backwards too long     
        if reverseDrivingFlag == True:
            return [0,0]   
            
        
if __name__ == '__main__':
    # needed for communication with robot...
    robot = Robot()

    # needed for communication with other robots
    client = Client()

    contr = Controller()
    # GObject -> calls controller every 100 ms
    loop = GObject.MainLoop()
    handle = GObject.timeout_add(100, contr.control)
    
    # exit via control c
    try:
        loop.run()
    except KeyboardInterrupt:
        robot.setMotorValues(0, 0)
        client.closeClient()
        print("exit programm")
    finally:
        robot.setMotorValues(0, 0)
        client.closeClient()
