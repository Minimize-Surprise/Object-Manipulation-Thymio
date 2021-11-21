# imports for thymio
import dbus
import dbus.mainloop.glib
import sys
from optparse import OptionParser
import force_resistor
from light_sensor import Light_sensor
import parameters
import numpy as np

# FOR LIGHT
# Add the following under "/boot" in the "config.txt"
# "dtoverlay=i2c-gpio,i2c_gpio_sda=5,i2c_gpio_scl=6"

# FOR FORCE
# Trim the potentiometer till the printed value is at its lowest point.
# Then turn it up slowly while the value sill remains at its lowest point.
# If it switches back to a higher number again you've hit the sweet spot.

class Robot:

    def __init__(self):
        self.proxSensorsVal = [0, 0, 0, 0, 0, 0, 0]
        self.groundSensorsVal = [0, 0]
        self.parser = OptionParser()
        self.parser.add_option("-s", "--system", action="store_true", dest="system", default=False,
                          help="use the system bus instead of the session bus")

        (options, args) = self.parser.parse_args()

        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

        if options.system:
            self.bus = dbus.SystemBus()
        else:
            self.bus = dbus.SessionBus()

        # Create Aseba network
        self.network = dbus.Interface(self.bus.get_object('ch.epfl.mobots.Aseba', '/'),
                                      dbus_interface='ch.epfl.mobots.AsebaNetwork')
        
        if parameters.useLightSensors:
            self.lsFront = Light_sensor(3)
            self.lsBack = Light_sensor(1)
            
            
    def getForce(self):
        forceValue = min((force_resistor.readadc(0) / 127.0), 1.0)
        return forceValue


    def getLight(self, i):
        if i == 0:
            return(self.lsFront.read_sensor_value() / parameters.MAX_LIGHT_SENSOR)
        else:
            return(self.lsBack.read_sensor_value() / parameters.MAX_LIGHT_SENSOR)


    def getAllProxSensors(self):
        self.network.GetVariable("thymio-II", "prox.horizontal", reply_handler=self.get_variables_reply,
                                    error_handler=self.get_variables_error)
        
        return [(self.proxSensorsVal[0] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[1] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[2] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[3] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[4] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[5] / parameters.MAX_HORIZONTAL_SENSOR),
                (self.proxSensorsVal[6] / parameters.MAX_HORIZONTAL_SENSOR)]
    
    def getAllSensors(self):
        # update sensors
        self.network.GetVariable("thymio-II", "prox.horizontal", reply_handler=self.get_variables_reply,
                                  error_handler=self.get_variables_error)
        
        self.network.GetVariable("thymio-II", "prox.ground.reflected", reply_handler=self.get_variables_reply_ground,
                                  error_handler=self.get_variables_error)
        
        return np.array([
                [(self.proxSensorsVal[0] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[1] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[2] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[3] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[4] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[5] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[6] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.groundSensorsVal[0] / parameters.MAX_GROUND_SENSOR)],
                [(self.groundSensorsVal[1] / parameters.MAX_GROUND_SENSOR)],
                #[(force_resistor.readadc(0) / parameters.MAX_FORCE_SENSOR)]
                [min((force_resistor.readadc(0) / 127.0), 1.0)]
                ])
    
    def getAllSensorsLight(self):
        # update sensors
        self.network.GetVariable("thymio-II", "prox.horizontal", reply_handler=self.get_variables_reply,
                                  error_handler=self.get_variables_error)
        
        self.network.GetVariable("thymio-II", "prox.ground.reflected", reply_handler=self.get_variables_reply_ground,
                                  error_handler=self.get_variables_error)
        
        return np.array([
                [(self.proxSensorsVal[0] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[1] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[2] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[3] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[4] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[5] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.proxSensorsVal[6] / parameters.MAX_HORIZONTAL_SENSOR)],
                [(self.groundSensorsVal[0] / parameters.MAX_GROUND_SENSOR)],
                [(self.groundSensorsVal[1] / parameters.MAX_GROUND_SENSOR)],
                #[(force_resistor.readadc(0) / parameters.MAX_FORCE_SENSOR)],
                [min((force_resistor.readadc(0) / 127.0), 1.0)],
                [(self.lsFront.read_sensor_value() / parameters.MAX_LIGHT_SENSOR)],
                [(self.lsBack.read_sensor_value() / parameters.MAX_LIGHT_SENSOR)]
                ])
                
    def getAllGroundSensors(self):
        self.network.GetVariable("thymio-II", "prox.ground.reflected", reply_handler=self.get_variables_reply_ground,
            error_handler=self.get_variables_error)
        
        return [(self.groundSensorsVal[0] / parameters.MAX_GROUND_SENSOR),
                (self.groundSensorsVal[1] / parameters.MAX_GROUND_SENSOR)]

    def get_variables_reply(self, r):
        self.proxSensorsVal = r

    def get_variables_reply_ground(self, r):
        self.groundSensorsVal = r

    def get_variables_error(self, e):
        print("error:")
        print(str(e))
        
    def setMotorValues(self, left, right):
        self.network.SetVariable("thymio-II", "motor.left.target", [left])
        self.network.SetVariable("thymio-II", "motor.right.target", [right])
