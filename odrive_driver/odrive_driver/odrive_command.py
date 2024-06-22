#!/usr/bin/env python3

import sys
from time import sleep
# import logging
# import traceback
import math
import odrive
from odrive.enums import *

'''
Find ODrive serial number (need it in hex format) in odrivetool by running:
    hex(dev0.serial_number).split('x')[1].upper()
'''
ODRIVE_SERIAL_NUMBER = "374E305C3133"
# WHEEL_RADIUS = 0.08255  # meters
# WHEEL_BASE =  0.48
# default_logger = logging.getLogger(__name__)
# default_logger.setLevel(logging.DEBUG)
# # create console handler and set level to debug
# ch = logging.StreamHandler()
# ch.setLevel(logging.DEBUG)

# default_logger.addHandler(ch)

class ODriveController:
    # encoder_cpr = 4096
    odrive = None
    right_axis = None
    left_axis = None
    # _preroll_started = False
    # _preroll_completed = False
    # #engaged = False
    def __init__(self):
        # self.logger = logger if logger else default_logger
        self.connect()

    def __del__(self):
        self.disconnect()

    def connect(self):
    #    if self.driver:
    #         self.logger.info("Already connected. Disconnecting and reconnecting.")

        print(f'Attempting to connect to ODrive {ODRIVE_SERIAL_NUMBER}')
        try:
            self.odrive = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
            print(f'Connected to ODrive {ODRIVE_SERIAL_NUMBER}')
        except Exception as e:
            # self.logger.error("No ODrive found. Is device powered?")
            print(f'Failed to connect to ODrive: {e}')
            return False


        self.left_axis = self.odrive.axis0
        self.right_axis = self.odrive.axis1
        self.axes = (self.odrive.axis0, self.odrive.axis1)
        # check for no errors
        # for axis in [self.right_axis, self.left_axis]:
        #     if axis.error != 0:
        #         error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
        #         self.logger.error(error_str)
        #         self.reboot()
        #         return False
            
        self.armed_vel = False
        self.armed_pos = False
        self.calibration_override_timer = 10
        return True
    
    def disconnect(self):
        self.right_axis = None
        self.left_axis = None
        
        #self.engaged = False
        
        if not self.driver:
            # self.logger.error("Not connected.")
            return False
        
        try:
            self.release()
        except:
            # self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.odrive = None
        return True
    
    def reboot(self):
        if not self.odrive:
            # self.logger.error("Not connected.")
            return False
        try:
            self.odrive.reboot()
        # except KeyError:
            # self.logger.error("Rebooted ODrive.")
        # except:
            # self.logger.error("Failed to reboot: " + traceback.format_exc())
        finally:
            self.odrive = None
        return True
    
    def calibrate(self, axes="both"):
        # if not self.driver:
        #     self.logger.error("Not connected.")
        #     return False
        
        # self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        
        # for i, axis in enumerate(self.axes):
        #     self.logger.info("Calibrating axis %d..." % i)
        #     axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        #     time.sleep(1)
        #     while axis.current_state != AXIS_STATE_IDLE:
        #         time.sleep(0.1)
        #     if axis.error != 0:
        #         self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
        #         return False
                
        # return True     *************************   
        # self.armed = False

        # if not calibration_override:
        #     for axis in axes:
        #         if not self.axes[axis].encoder.is_ready:
        #             print(f'Calibrating encoder on axis{axis}')
        #             self.axes[axis].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        #         else:
        #             pass
        #     while len(axes):
        #         for axis in axes:
        #             if self.axes[axis].encoder.is_ready:
        #                 axes.remove(axis)
        # else:
        #     for axis in axes:
        #         print(f'Calibrating encoder on axis{axis}')
        #         self.axes[axis].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        #     print(f'Waiting {self.calibration_override_timer} seconds for calibration to complete...')
        #     sleep(self.calibration_override_timer)
        
        print('Encoders calibrated.')

    def engage(self):
        if not self.odrive:
            # self.logger.error("Not connected.")
            return False

        #self.logger.debug("Setting drive mode.")
        self.arm_velocity_control()
        
        #self.engaged = True
        return True
        
    def release(self):
        if not self.odrive:
            # self.logger.error("Not connected.")
            return False
        #self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE

        #self.engaged = False
        return True
     
    
    def arm_velocity_control(self, axes=None):
        if not axes: axes = self.axes
        for axis in axes:
            print(f'Arming axis{axis}')
            axis.controller.input_vel = 0
            axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.armed_vel = True

    def arm_position_control(self, axes=None):
        if not axes: axes = self.axes
        for axis in axes:
            print(f'Arming axis{axis}')
            axis.controller.input_pos = 0
            axis.controller.config.input_mode = INPUT_MODE_POS_FILTER
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.armed_pos = True

    def drive_tps(self, left_motor_val, right_motor_val):
        if not self.odrive:
            # self.logger.error("Not connected.")
            return
        #try:
        self.left_axis.controller.input_vel = -left_motor_val
        self.right_axis.controller.input_vel = right_motor_val
        #except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
        #    raise ODriveFailure(str(e))

    def command_velocity_tps(self, axis, velocity):
        if self.armed_vel:
            print(f'Commanding velocity {velocity} on axis{axis}')
            self.axes[axis].controller.input_vel = velocity

    def command_position_t(self, axis, position):
        if self.armed_pos:
            self.axes[axis].controller.input_pos = position

    def get_velocity_tps(self, axis):
        return axis.encoder.vel_estimate    if axis  else 0

    def get_position_t(self, axis):
        return axis.encoder.pos_estimate    if axis  else 0  # units: turns

    def left_vel_estimate_tps(self):    return -self.get_velocity_tps(self.left_axis)
    def right_vel_estimate_tps(self):   return self.get_velocity_tps(self.right_axis)
    def left_pos_t(self):               return -self.get_position_t(self.left_axis)
    def right_pos_t(self):              return self.get_position_t(self.right_axis)
    
    def left_vel_estimate_radps(self):    return self.get_velocity_tps(self.left_axis)*2*math.pi
    def right_vel_estimate_radps(self):   return self.get_velocity_tps(self.right_axis)*2*math.pi
    def left_pos_rad(self):               return self.get_position_t(self.left_axis)*2*math.pi
    def right_pos_rad(self):              return self.get_position_t(self.right_axis)*2*math.pi

    # TODO check these match the right motors, but it doesn't matter for now
    def left_temperature(self):   return self.left_axis.motor.get_inverter_temp()  if self.left_axis  else 0.
    def right_temperature(self):  return self.right_axis.motor.get_inverter_temp() if self.right_axis else 0.
    
    def left_current(self):       return self.left_axis.motor.current_control.Ibus  if self.left_axis and self.left_axis.current_state > 1 else 0.
    def right_current(self):      return self.right_axis.motor.current_control.Ibus if self.right_axis and self.right_axis.current_state > 1 else 0.
    # from axis.hpp: https://github.com/madcowswe/ODrive/blob/767a2762f9b294b687d761029ef39e742bdf4539/Firmware/MotorControl/axis.hpp#L26
    MOTOR_STATES = [
        "UNDEFINED",                  #<! will fall through to idle
        "IDLE",                       #<! disable PWM and do nothing
        "STARTUP_SEQUENCE",           #<! the actual sequence is defined by the config.startup_... flags
        "FULL_CALIBRATION_SEQUENCE",  #<! run all calibration procedures, then idle
        "MOTOR_CALIBRATION",          #//<! run motor calibration
        "SENSORLESS_CONTROL",         #//<! run sensorless control
        "ENCODER_INDEX_SEARCH",       #//<! run encoder index search
        "ENCODER_OFFSET_CALIBRATION", #//<! run encoder offset calibration
        "CLOSED_LOOP_CONTROL",        #//<! run closed loop control
        "LOCKIN_SPIN",                #//<! run lockin spin
        "ENCODER_DIR_FIND",
        ]
        
    def left_state(self):       return self.MOTOR_STATES[self.left_axis.current_state] if self.left_axis else "NOT_CONNECTED"
    def right_state(self):      return self.MOTOR_STATES[self.right_axis.current_state] if self.right_axis else "NOT_CONNECTED"
    
    def bus_voltage(self):      return self.driver.vbus_voltage if self.left_axis else 0.
          

    def get_errors(self, axes=[0, 1]):
        for axis in axes:
            print(f'\nGetting errors for axis{axis}:')
            print('Axis: ' + str(hex(self.axes[axis].error)))
            print('Motor: ' + str(hex(self.axes[axis].motor.error)))
            print('Controller :' + str(hex(self.axes[axis].controller.error)))
            print('Encoder: ' + str(hex(self.axes[axis].encoder.error)))
  

if __name__ == '__main__':
    import math
    odrv0 = ODriveController()
    odrv0.encoder_offset_calibration()
    # odrv0.arm_position_control()
    # sleep(2)

    # position = 0
    # while True:
    #     odrv0.command_position(0, 3 * math.cos(position))
    #     odrv0.command_position(1, 3 * math.sin(position))

    #     print(odrv0.get_position(0))
    #     position += math.pi / 100
    #     sleep(0.1)
    odrv0.engage()
    sleep(2)

    position = 0
    while True:
        odrv0.drive(5 * math.cos(position),5 * math.cos(position))

        position += math.pi / 100
        sleep(0.1)

    # position = 1
    # while True:
    #     odrv0.command_position(0, position)
    #     odrv0.command_position(1, position)

    #     print(odrv0.get_position(0))
    #     position *= -1
    #     sleep(5)



  
