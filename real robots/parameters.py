# Constants for the Thymio's speed and sensors
MAX_SPEED             = 315     # ca. 12.6 cm/s
MAX_HORIZONTAL_SENSOR = 4500.0 	# maximum horizontal sensor value
MAX_GROUND_SENSOR     = 1023.0  # maximum ground sensor value
MAX_FORCE_SENSOR      = 1023.0  # maximum force sensor value
MAX_LIGHT_SENSOR      = 1024.0  # maximum light sensor value


# paramters for evolution
EVALS               = 400  # Number of evaluations
EVAL_TIME           = 100
POST_EVAL_TIME      = 1000
RE_EVAL_PROB        = 0.2
RE_EVAL_WEIGHT      = 0.2
ACTIONS             = 2
MUT_RATE            = 0.1
ARENA_X             = 1.1  # in m
ARENA_Y             = 1.1  # in m
ROBOTS              = 4
useLightSensors     = False
enableDataTracking  = True

if useLightSensors:
    SENSORS         = 12  # 5 horiontal front + 2 back + pressure + 2 ground + 2 light
    HIDDEN_ACTION   = 8
    HIDDEN_PRED     = 13
    genLengthAction = HIDDEN_ACTION * (SENSORS+ACTIONS+1) + ACTIONS * (HIDDEN_ACTION+1)
    genLengthPred   = (HIDDEN_PRED) * (SENSORS+ACTIONS+3) + SENSORS * (HIDDEN_PRED+1) # bias + recursion + initial recurrent values
else:
    SENSORS         = 10  # 5 horiontal front + 2 back + pressure + 2 ground
    HIDDEN_ACTION   = 7
    HIDDEN_PRED     = 11
    genLengthAction = HIDDEN_ACTION * (SENSORS+ACTIONS+1) + ACTIONS * (HIDDEN_ACTION+1)
    genLengthPred   = (HIDDEN_PRED) * (SENSORS+ACTIONS+3) + SENSORS * (HIDDEN_PRED+1) # bias + recursion + initial recurrent values
