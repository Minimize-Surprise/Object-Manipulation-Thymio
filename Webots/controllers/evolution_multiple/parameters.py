# Constants for the Thymio's speed and sensors
MAX_SPEED = 6 #9.53 - we don't go full speed 
MAX_HORIZONTAL_SENSOR = 4500.0 	# maximum horizontal sensor value
MAX_GROUND_SENSOR = 1100.0   # maximum ground sensor value
MAX_FORCE_SENSOR = 200.0   # maximum force sensor value
MAX_LIGHT_SENSOR = 5000.0  # maximum light sensor value


# paramters for evolution
EVALS = 1000  # Number of evaluations 
EVAL_TIME = 1000
POST_EVAL_TIME = 10000
RE_EVAL_PROB = 0.2 
RE_EVAL_WEIGHT = 0.2 
ACTIONS = 2 
MUT_RATE = 0.1 
ARENA_X = 1.1  # in m 
ARENA_Y = 1.1  # in m
ROBOTS = 4
useLightSensors = False

if useLightSensors:
    SENSORS = 12  # 5 horiontal front + 2 back + pressure + 2 ground + 2 light
    HIDDEN_ACTION = 8
    HIDDEN_PRED = 13
else:
    SENSORS = 10  # 5 horiontal front + 2 back + pressure + 2 ground
    HIDDEN_ACTION = 7
    HIDDEN_PRED = 11


#parameters for boxes
boxSize = 0.025  # We use square boxes
boxR = 0.036 # radius of box 
boxMass = 0.002  
    
#Here you can adjust the density of the blocks
maxForbiddenRangeX = boxR * 1.5 # * 2  # How much free space has to be next to a block
maxAllowedRangeX = boxR * 2 # * 2 # Maximum of space where no block can be

maxForbiddenRangeY = boxR * 1.5 # * 2  # How much free space has to be next to a box
maxAllowedRangeY = boxR * 2 # * 2  # Maximum of space where no box can be

#startpositions of robots
INITIALTRANS = [0, 0.0015, -0.1],[0.1, 0.0015, 0],[0, 0.0015, 0.1],[-0.1, 0.0015, 0]
INITIALROT = [0.00015083, 0.999988, -0.00484433, 3.14157],[0.0046935, -0.999977, 0.00499516, -1.5708],[0.999513, 0.002338, 0.0311202, -0.00969531],[0.00499505, 1, -0.00469339, -1.57085]
