"""  
Minimize Surprise - Evolution - Thymio

If the robot controlled by this controller should be the master robot, the name must contain 'master'.
Consequently, all other robots may not contain 'master' in their names.
"""

import sys
import random
import math
import os.path
from controller import Robot, Emitter, Receiver, Supervisor, Node, Field
import numpy as np
from genetic_population_multiple import GeneticPopulation
from state import State, write_csv 
import parameters


POSTEVAL = False
turnTimer = 0     # set by HWP, specifies how long robot is turning
turnFlag = False  # set by HWP if robot has to turn
direction = False # set by HWP, specifies turn direction
driveTimer = 0    # set by HWP, specifies how long the robot drives straight to evade wall
reverse = False   # set by HWP, specifies if the evades wall by driving forawrd or backwards
reverseDrivingFlag = False  # set by HWP in case robot has been driving backards for too long
reverseCounter = 0  # tracks how long the robot has been driving backwards
boxAmount = 0


def sigmoid(x):
    """ returns the value of the sigmoid function evaluated at all elements of x """
    return 1 / (1 + np.exp(-x))
    
    
def tanh(x):
    """ returns the value of the sigmoid function evaluated at all elements of x """
    return np.tanh(x)


class Controller():
    """
    Controller for thymio simulation using 1+1 evolution  distributed across a master and his slaves

    If the robot controlled by this controller should be the master robot, the robot's name must contain 'master'.
    Consequently, all other robots may not contain 'master' in their names.

    Usage:
    Simply call *control* at every time step. Well, after having created an object of course.

    """

    def __init__(self, robot, name, master, emitter, receiver):
        """ initialises values for avoid behavior and logfiles

        Arguments:
        robot -- reference to supervisor or robot instance 
        name -- robot name 
        master -- true iff this instance is the master robot
        emitter -- reference to webots emitter
        receiver -- reference to webots receiver

        """
        global ARENA_X 
        global ARENA_Y 
        
        self.robot = robot 
        # name for file I/O 
        self.name = name 

        # store reference, because they are very useful ;)
        self.master = master
        self.emitter = emitter
        self.receiver = receiver

        # create a genetic population
        self.population = GeneticPopulation(self, parameters.EVAL_TIME, parameters.POST_EVAL_TIME, parameters.RE_EVAL_PROB, parameters.EVALS, parameters.SENSORS, parameters.ACTIONS, parameters.HIDDEN_ACTION, parameters.HIDDEN_PRED, parameters.MUT_RATE, tanh, sigmoid, parameters.RE_EVAL_WEIGHT)

        if self.master:   
            # init log files
            self.filename = "results/run"
            
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
            write_csv("results/parameters", "maximumBlockDistance," + str(parameters.maxAllowedRangeX))
            write_csv("results/parameters", "minimumBlockDistance," + str(parameters.maxForbiddenRangeX))      
            

            if not os.path.isfile("results/trajectory.csv"):
                write_csv("results/trajectory", "SEP=,")
                write_csv("results/trajectory", "robot,translation0,translation1,translation2,rotation0,rotation1,rotation2,rotation3")
            
            self._log("SEP=,")
            self._log("king,mutant")

            # register evaluation listener for logging
            self.population.set_evaluation_listener(lambda x,y: self._log(str(x) + "," + str(y)))
                        
            #self.reposition_robots() 
        
        else: # slave
             # init log file for predictions and sensors 
             self.filename = "results/pred_" + str(self.name) 
             
             if not os.path.isfile(self.filename):
                 if not parameters.useLightSensors:
                     write_csv(self.filename, "SEP=,")
                     write_csv(self.filename, "obstacle avoidance,pred0 (t+1),pred1 (t+1),pred2 (t+1),pred3 (t+1),pred4 (t+1),pred5 (t+1),pred6 (t+1),predg0 (t+1),predg1 (t+1),predp (t+1),s0 (t),s1 (t),s2 (t),s3 (t),s4 (t),s5 (t),s6 (t),sg0 (t),sg1 (t), sp(t),m0 selected,m1 selected,m0 real,m1 real")
                 else:
                     write_csv(self.filename, "SEP=,")
                     write_csv(self.filename, "obstacle avoidance,pred0 (t+1),pred1 (t+1),pred2 (t+1),pred3 (t+1),pred4 (t+1),pred5 (t+1),pred6 (t+1),predg0 (t+1),predg1 (t+1),predp (t+1),predl0 (t+1),predl1 (t+1),s0 (t),s1 (t),s2 (t),s3 (t),s4 (t),s5 (t),s6 (t),sg0 (t),sg1 (t), sp(t),sl0(t),sl1(i),m0 selected,m1 selected,m0 real,m1 real")
             
                
    def _log(self, line):
        """ writes the line to the logfile """
        write_csv(self.filename, line)

    def emit(self, msg):
        """ sends a message """
        self.emitter.send(msg)

    def receive(self):
        """ receive a (one!) message and returns it, None if no message received """
        if self.receiver.getQueueLength() > 0:
            msg = self.receiver.getData()
            self.receiver.nextPacket()
            return msg
        return None

    def control(self):
        """ manage master and slave logic

        on a slave: calculates motor values and sets them using 1+1 evolution encapsulated in a genetic population
        on the master:  waits for evaluation scores and distributs new genomes

        """        
        if self.master:
            self.control_master()
        else:
            self.control_slave()

    def control_master(self):      
        """ wait for evaluation scores and distribute new genome """ 
               
        if self.population.POST_EVAL:
            if self.robot.movieIsReady(): # Quit simulation when run is over 
                #save final block positions
                write_csv("results/blockPosVideo", "")
                write_csv("results/blockPosVideo", "Endpositions")
                write_csv("results/blockPosVideo", "x,z")
                count = 1
                while count <= boxAmount:
                    box = robot.getFromDef("box" + str(count))
                    pos = box.getPosition()
                    write_csv("results/blockPosVideo", str(pos[0]) + "," + str(pos[2]))
                    count = count + 1
                    
                self.robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)    
                self.robot.simulationQuit(1)
                
            if self.population.state != State.STOP: 
                # log trajectory for each robot 
                for i in range(1, parameters.ROBOTS+1): 
                    slave = self.robot.getFromDef('T' + str(i))
                    translationValues = slave.getField('translation').getSFVec3f()
                    rotationValues = slave.getField('rotation').getSFRotation() 
                    write_csv("results/trajectory", 'T' + str(i) + ',' + str(translationValues[0]) + ',' + str(translationValues[1]) + ',' + str(translationValues[2]) + ',' + str(rotationValues[0]) + ',' + str(rotationValues[1]) + ',' + str(rotationValues[2]) + ',' + str(rotationValues[3]))
        
        # receive fitness values, determine overall fitness, distribute new genomes           
        self.population.execute_master()
        
        # don't move        
        motor = [0,0]
        leftMotor.setVelocity(motor[0])
        rightMotor.setVelocity(motor[1])
        
        return True

    def control_slave(self):
        """ calculates motor values and sets them using 1+1 evolution distributed across a master and his slaves """
        global turnFlag
        global reverseDrivingFlag
        
        # transform sensor values into a numpy vector
        if parameters.useLightSensors == False:
            sensors = np.array([
                                [outerLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [outerRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [backLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [backRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR], 
                                [groundLeftSensor.getValue()/parameters.MAX_GROUND_SENSOR],
                                [groundRightSensor.getValue()/parameters.MAX_GROUND_SENSOR],
                                [force.getValue()/parameters.MAX_FORCE_SENSOR]
                            ])
        else:
            sensors = np.array([
                                [outerLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [centralRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [outerRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [backLeftSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR],
                                [backRightSensor.getValue()/parameters.MAX_HORIZONTAL_SENSOR], 
                                [groundLeftSensor.getValue()/parameters.MAX_GROUND_SENSOR],
                                [groundRightSensor.getValue()/parameters.MAX_GROUND_SENSOR],
                                [force.getValue()/parameters.MAX_FORCE_SENSOR],
                                [ls0.getValue()/parameters.MAX_LIGHT_SENSOR],
                                [ls1.getValue()/parameters.MAX_LIGHT_SENSOR]
                            ])
                                        
        action, pred = self.population.execute_slave(sensors)  # this is the line containing the 1+1 evolution magic
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
            write_csv(self.filename, str(obstacle_avoidance)+","+str(pred[0][-1])+","+str(pred[1][-1])+","+str(pred[2][-1])+","+str(pred[3][-1])
                                     +","+str(pred[4][-1])+","+str(pred[5][-1])+","+str(pred[6][-1])+","+str(pred[7][-1])+","+str(pred[8][-1])
                                     +","+str(pred[9][-1])+","+str(sensors[0][-1])+","+str(sensors[1][-1])+","+str(sensors[2][-1])+","+str(sensors[3][-1])+","
                                     +str(sensors[4][-1])+","+str(sensors[5][-1])+","+str(sensors[6][-1])+","+str(sensors[7][-1])+","+str(sensors[8][-1])+","+str(sensors[9][-1])
                                     +","+str(tmp[0])+","+str(tmp[1])+","+str(motor[0])+","+str(motor[1]))
        if self.population.POST_EVAL and parameters.useLightSensors and action is not None: # log values during post-evaluation                                    
            write_csv(self.filename, str(obstacle_avoidance)+","+str(pred[0][-1])+","+str(pred[1][-1])+","+str(pred[2][-1])+","+str(pred[3][-1])
                                     +","+str(pred[4][-1])+","+str(pred[5][-1])+","+str(pred[6][-1])+","+str(pred[7][-1])+","+str(pred[8][-1])
                                     +","+str(pred[9][-1])+","+str(pred[10][-1])+","+str(pred[11][-1])+","+str(sensors[0][-1])+","+str(sensors[1][-1])+","+str(sensors[2][-1])+","+str(sensors[3][-1])+","
                                     +str(sensors[4][-1])+","+str(sensors[5][-1])+","+str(sensors[6][-1])+","+str(sensors[7][-1])+","+str(sensors[8][-1])+","+str(sensors[9][-1])+","+str(sensors[10][-1])+","+str(sensors[11][-1])
                                     +","+str(tmp[0])+","+str(tmp[1])+","+str(motor[0])+","+str(motor[1]))   

        # set motor values
        leftMotor.setVelocity(motor[0])
        rightMotor.setVelocity(motor[1])


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
            if reverseCounter >= 900:
                reverseDrivingFlag = True
        else:
            reverseCounter = 0
            reverseDrivingFlag = False
                                             
        # driving - check for too high pressure or leaving the arena
        # 0.25 are 10 blocks of force
        if (sensors[9][0] > 0.25 or sensors[7][0] == 0.0 or sensors[8][0] == 0.0): 
            # if he is not willing to turn
            if (motor[0] == motor[1]) or np.abs((motor[0]+motor[1])/(motor[1]-motor[0])) > (1.0/3.0):               
                setTurnParameters = True 
                
        # if the robot sees a wall or another robot it moves away from it
        if sensors[0][0] > 0.6 or sensors[1][0] > 0.6 or sensors[2][0] > 0.6 or sensors[3][0] > 0.6 or sensors[4][0] > 0.6:
            setTurnParameters = True 
            driveTimer = 50
            reverse = True
             
        elif sensors[5][0] > 0.7 or sensors[6][0] > 0.7:   
            setTurnParameters = True    
            driveTimer = 50
            reverse = False
        
        # set up the hwp parameters    
        if setTurnParameters: 
            turnFlag = True
            turnTimer = random.randrange(70, 120)
                   
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
            
                  
    def resetArena(self):

        global boxAmount
        
        #save final block positions
        write_csv("results/blockPosRun", "")
        write_csv("results/blockPosRun", "Endpositions")
        write_csv("results/blockPosRun", "x,z")      
    
        # reset robot positions
        for i in range(1,parameters.ROBOTS+1):
            bot = robot.getFromDef('T'+str(i))
            trans_field = bot.getField("translation")
            rot_field = bot.getField("rotation")
            trans_field.setSFVec3f(parameters.INITIALTRANS[i-1])
            rot_field.setSFRotation(parameters.INITIALROT[i-1])
    
        #delete boxes 
        count = 1
        while boxAmount > 0:
            box = robot.getFromDef("box" + str(count))
            pos = box.getPosition()
            write_csv("results/blockPosRun", str(pos[0]) + "," + str(pos[2]))
            box.remove()
            count = count + 1
            boxAmount = boxAmount - 1
    
        #respawn boxes
        self.fillArena(2)
    
    
    def fillArena(self, iteration):

        """ 
        Fills empty arena with boxes
        """

        #Set position in top left of the arena
        currentBlockPosX = -(parameters.ARENA_X / 2) + parameters.boxR/2.0
        currentBlockPosY = -(parameters.ARENA_Y / 2) + parameters.boxR/2.0
    
        # spawn first box	
        self.spawnBox(currentBlockPosX, currentBlockPosY, 0)
        #Save Blockpos
        if iteration == 1:
            write_csv("results/blockPosRun", "SEP=,")
            write_csv("results/blockPosRun", "Startpositions")
            write_csv("results/blockPosRun", "x,z")
            write_csv("results/blockPosRun", str(currentBlockPosX) + "," + str(currentBlockPosY))
        else:
            write_csv("results/blockPosVideo", "SEP=,")
            write_csv("results/blockPosVideo", "Startpositions")
            write_csv("results/blockPosVideo", "x,z")
            write_csv("results/blockPosVideo", str(currentBlockPosX) + "," + str(currentBlockPosY))
        
        # calculate next coordiantes based on the set density
        nextBlockPosX = round(random.uniform(currentBlockPosX + parameters.maxForbiddenRangeX, currentBlockPosX + parameters.maxAllowedRangeX), 2)
        nextBlockPosY = round(random.uniform(currentBlockPosY, currentBlockPosY + parameters.maxAllowedRangeY - parameters.maxForbiddenRangeY), 2)
    
        # traverses the arena and spawns boxes at random positions with a fixed minimum distance from each other
        while nextBlockPosY <= (parameters.ARENA_X / 2.0 - parameters.boxR/2.0):
            while nextBlockPosX <= (parameters.ARENA_X / 2.0 - parameters.boxR/2.0):
                #Spawn boxes
                self.spawnBox(nextBlockPosX, nextBlockPosY, iteration)

                #calculate next coordinates
                currentBlockPosX = nextBlockPosX
                nextBlockPosX = round(random.uniform(currentBlockPosX + parameters.maxForbiddenRangeX, currentBlockPosX + parameters.maxAllowedRangeX), 2)
                nextBlockPosY = round(random.uniform(currentBlockPosY, currentBlockPosY + parameters.maxAllowedRangeY - parameters.maxForbiddenRangeY), 2)

            currentBlockPosY = currentBlockPosY + parameters.maxForbiddenRangeY
            nextBlockPosY = round(random.uniform(currentBlockPosY, currentBlockPosY + parameters.maxAllowedRangeY - parameters.maxForbiddenRangeY), 2)
            currentBlockPosX = -(parameters.ARENA_X / 2) + parameters.boxR/2.0
            nextBlockPosX = round(random.uniform(currentBlockPosX, currentBlockPosX + parameters.maxForbiddenRangeX), 2)
        
        # save box amounts in paramters.csv    
        if iteration == 1:
            write_csv("results/parameters", "boxAmountFirst," + str(boxAmount))
        else:    
            write_csv("results/parameters", "boxAmountSecond," + str(boxAmount))  
            
    
    def spawnBox(self, posX, posY, iteration):
        """ 
        Spawns a box based on the input coordinates with random rotation
        """
        global boxAmount
    
        #don't spawn boxes on top of thymio
        if -0.25 < posX < 0.25 and -0.25 < posY < 0.25:
            return True
    
        boxAmount = boxAmount + 1 
        name = "CardboardBox"
        curr_pos = str(posX) + " " + str(parameters.boxSize / 2) + " " + str(posY)
        rotation = round(random.uniform(0, math.pi / 2), 2)
        newBox = "DEF box" + str(boxAmount) + " " + name + " {translation " + curr_pos + " rotation 0 1 0 " + str(rotation) + " size " + sizee + " mass " + str(parameters.boxMass) + "}"
        Field.importMFNodeFromString(root_children_field, -1, newBox)
    
        #save blockpos
        if iteration == 1:
            write_csv("results/blockPosRun", str(posX) + "," + str(posY))
        elif iteration == 2:
            write_csv("results/blockPosVideo", str(posX) + "," + str(posY))
            
        
if __name__ == '__main__':
    # Get reference to the robot.
    if str(sys.argv[1]) == "slave":
        robot = Robot()
    else:
        robot = Supervisor() 

    # Get simulation step length
    timeStep = int(robot.getBasicTimeStep())

    # Get left and right wheel motors
    leftMotor = robot.getMotor("motor.left")
    rightMotor = robot.getMotor("motor.right")

    # Get distance sensors
    outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
    centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
    centralSensor = robot.getDistanceSensor("prox.horizontal.2")
    centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
    outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")
    
    backLeftSensor = robot.getDistanceSensor("prox.horizontal.5")
    backRightSensor = robot.getDistanceSensor("prox.horizontal.6")
    
    groundLeftSensor = robot.getDistanceSensor("prox.ground.0")
    groundRightSensor = robot.getDistanceSensor("prox.ground.1")

    # Enable distance sensors
    outerLeftSensor.enable(timeStep)
    centralLeftSensor.enable(timeStep)
    centralSensor.enable(timeStep)
    centralRightSensor.enable(timeStep)
    outerRightSensor.enable(timeStep)
    
    backLeftSensor.enable(timeStep)
    backRightSensor.enable(timeStep)
    
    groundLeftSensor.enable(timeStep)
    groundRightSensor.enable(timeStep)
    
    # Get and enable Light and Pressure Sensors
    ls0 = robot.getLightSensor('light0')
    ls0.enable(timeStep)
    ls1 = robot.getLightSensor('light1')
    ls1.enable(timeStep)
    force = robot.getTouchSensor('force')
    force.enable(timeStep)
    
    # Disable motor PID control mode
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    # check if this controller is run on the master robot
    controller = Controller(robot, robot.getName(), "master" in robot.getName(), robot.getEmitter("emitter"), robot.getReceiver("receiver"))
    controller.receiver.enable(timeStep)
    
    # supervisor fill arena with blocks
    if not str(sys.argv[1]) == "slave":
        rootNode = robot.getRoot()
        root_children_field = rootNode.getField("children")
        sizee = str(parameters.boxSize) + " " + str(parameters.boxSize) + " " + str(parameters.boxSize)  # String for later uses
        controller.fillArena(1)
    
    # beginning of execution
    while(robot.step(timeStep) != -1):
        controller.control()
