from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

motor_pair = MotorPair('A', 'B')
motorR = Motor('A')
motorL = Motor('B')

motor_pair.set_default_speed(20) # Set the default speed of the motor_pair.
motor_pair.set_motor_rotation(17.6, 'cm') # Set the distance that the robot travels for one rotation of its wheels. The value 17.6 comes from
                                        # the diameter of the wheel (5.6cm) multiplied by "π" (3.14).
motor_pair.set_stop_action('brake') # Activate the brakes when the robot stops. The other conditions are 'hold' and 'coast'.

wait_for_seconds(1) # Wait for one second.
hub.motion_sensor.reset_yaw_angle() # Reset the Gyro sensor. The current yaw angle value is equal to 0.

def GoDegMM(speed,angle,mm,kp,ki,kd):
    motorR.set_degrees_counted(0)
    P = 0.0000
    I = 0.0000
    D = 0.0000
    lasterror = 0
    if mm <= 200:
        while motorR.get_degrees_counted() < mm*1.302 :
            error = angle - hub.motion_sensor.get_yaw_angle()
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            diff = P + I + D
            P = kp * error
            I = I + ki * error
            D = kd * (error - lasterror)
            motorL.start_at_power(round(-speed - diff))
            motorR.start_at_power(round(speed - diff))
            lasterror = error
    if mm > 200:

        while motorR.get_degrees_counted() < (70*1.302) :
            error = angle - hub.motion_sensor.get_yaw_angle()
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            diff = P + I + D
            P = kp * error
            I = I + ki * error
            D = kd * (error - lasterror)
            motorL.start_at_power(round(-25 - diff))
            motorR.start_at_power(round(25 - diff))
            lasterror = error
        while motorR.get_degrees_counted() < (mm*1.302 - 70*1.302) :
            error = angle - hub.motion_sensor.get_yaw_angle()
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            diff = P + I + D
            P = kp * error
            I = I + ki * error
            D = kd * (error - lasterror)
            motorL.start_at_power(round(-speed - diff))
            motorR.start_at_power(round(speed - diff))
            lasterror = error
        speed = 25
        while motorR.get_degrees_counted() < mm*1.302 :
            error = angle - hub.motion_sensor.get_yaw_angle()
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            diff = P + I + D
            P = kp * error
            I = I + ki * error
            D = kd * (error - lasterror)
            motorL.start_at_power(round(-speed - diff))
            motorR.start_at_power(round(speed - diff))
            lasterror = error
    motor_pair.stop()

def Turn(speed,angle):
    t1 = 0
    t2 = 0
    if angle < 0:
        if hub.motion_sensor.get_yaw_angle() < 0:
            if hub.motion_sensor.get_yaw_angle() > angle:
                while hub.motion_sensor.get_yaw_angle() != angle:
                    motorR.start_at_power(speed)
                    motorL.start_at_power(speed)
            else:
                while hub.motion_sensor.get_yaw_angle() != angle:
                    motorR.start_at_power(-speed)
                    motorL.start_at_power(-speed)
        elif hub.motion_sensor.get_yaw_angle() <= 0:
            tl = hub.motion_sensor.get_yaw_angle() - angle
            tr = (179 - hub.motion_sensor.get_yaw_angle()) + (-180 - angle)
    elif angle > 0:
        if hub.motion_sensor.get_yaw_angle() > 0:
            if hub.motion_sensor.get_yaw_angle() > angle:
                while hub.motion_sensor.get_yaw_angle() != angle:
                    motorR.start_at_power(speed)
                    motorL.start_at_power(speed)
            else:
                while hub.motion_sensor.get_yaw_angle() != angle:
                    motorR.start_at_power(-speed)
                    motorL.start_at_power(-speed)
        elif hub.motion_sensor.get_yaw_angle() <= 0:
            tr = hub.motion_sensor.get_yaw_angle() + angle
            tl = (-180 - hub.motion_sensor.get_yaw_angle()) + (179 - angle)
    if tr >= tl:
        while hub.motion_sensor.get_yaw_angle() != angle:
            motorR.start_at_power(-speed)
            motorL.start_at_power(-speed)
    else:
        while hub.motion_sensor.get_yaw_angle() != angle:
            motorR.start_at_power(speed)
            motorL.start_at_power(speed)


        
            
        

GoDegMM(75,0,1500,4,0.01,0.07)


