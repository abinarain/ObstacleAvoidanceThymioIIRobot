"""Braitenberg-based obstacle-avoiding robot controller.

Author: Abhishek Narain Singh
Date: 26th September 2020
Email: abhishek.narain@iitdalumni.com
This is a robot code to avoid obstacles and reach an initial distance by adjusting directions again and again

"""

from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.85 * maxMotorVelocity

# Get compass sensor.
compa = robot.getCompass("compass")
compa.enable(timeStep)

# Flag for object nearby
objectNearby=True
    
# Count for how many time steps the robot has not detected an object
counterTimes=0

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    #Get the direction of robot
    # The direction of the robot (yaw) is stored in [rad]. 
    angle = compa.getValues()[0]
    
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    # Set wheel velocities based on sensor values=> This will take care of direction by slowing down the appropriate wheel based on sensor values
    leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 0.59) #The factor of 0.4 can reduce much higher values from base initial velocity
    rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 0.39) #Intentionally introducing bias for turn  by keeping a different dividing factor
    
    # Adjust the speed of the wheels by slowing down and taking right turn when there is a head-on collision
    if(centralSensorValue!=0 and centralLeftSensorValue == 0 and centralRightSensorValue == 0):
        leftMotor.setVelocity(initialVelocity*(-0.99) + (centralRightSensorValue + outerRightSensorValue) / 0.4) #Lowering the base speed and reversing direction and reducing further values
        rightMotor.setVelocity(initialVelocity*(-0.99) + (centralLeftSensorValue + outerLeftSensorValue) / 0.33 + centralSensorValue*0.20) #Lowering the base speed and reversing direction  and reducing more values to give it direction
    
    # Adjust the speed of the wheels by slowing down and taking right turn when there is a head-on collision with also right object closeby
    if(centralSensorValue!=0 and (centralRightSensorValue > centralLeftSensorValue) ):
        leftMotor.setVelocity(initialVelocity*(0.99) - (centralRightSensorValue + outerRightSensorValue) / 0.4) #Lowering the base speed and reversing direction and reducing further values
        rightMotor.setVelocity(initialVelocity*(0.99) - (centralLeftSensorValue + outerLeftSensorValue) / 0.33 - centralSensorValue*0.20) #Lowering the base speed and reversing direction  and reducing more values to give it direction
    
    # Adjust the speed of the wheels by slowing down and taking left turn when there is a head-on collision with also left object closeby 
    if(centralSensorValue!=0 and (centralLeftSensorValue > centralRightSensorValue) ):
        leftMotor.setVelocity(initialVelocity*(0.99) - (centralLeftSensorValue + outerLeftSensorValue) / 0.4 - centralSensorValue*0.20) #Lowering the base speed and reversing direction and reducing further values
        rightMotor.setVelocity(initialVelocity*(0.99) - (centralRightSensorValue + outerRightSensorValue) / 0.37) #Lowering the base speed and reversing direction  and reducing more values to give it direction
        
    
    #or in case there is a complete block from left and right while no block from centre we reverse direction and adjust based on sensors which are closer
    if ( outerLeftSensorValue !=0 and centralSensorValue == 0 and outerRightSensorValue !=0):
        leftMotor.setVelocity(initialVelocity*(-0.99) + (centralRightSensorValue + outerRightSensorValue) / 0.39)
        rightMotor.setVelocity(initialVelocity*(-0.99) + (centralLeftSensorValue + outerLeftSensorValue) / 0.29) #intentionally introducing bias by dividing by a differne factor in order to help move 
        
        
    # Now if the robot has not hit any obstacle for a long time, then it makes sense to adjust the direction in order to get a shorter route
    # Check whether the robot has detected objects or not
    if( centralLeftSensorValue != 0 or centralRightSensorValue != 0 or centralSensorValue != 0 or outerLeftSensorValue != 0 or outerRightSensorValue != 0 ):
        objectNearby = True
        initialVelocity = 0.70 * maxMotorVelocity  # If there are objects nearby then the base velocity should be lowered
        counterTimes = 0
    else:
        counterTimes = counterTimes + 1
        if(counterTimes > 200 ):
            objectNearby=False
            initialVelocity = 0.99 * maxMotorVelocity #If there are no objects nearby then the base velocity can be increased

    # If the robot has been not detecting any object for long time then adjust the direction
    if(objectNearby == False):
        if(angle > 0.03): #i.e. greater than 1.5 degrees
            leftMotor.setVelocity(initialVelocity)
            rightMotor.setVelocity(initialVelocity*0.8)
        elif(angle < -0.03): #i.e. lesser than 1.5 degrees
            leftMotor.setVelocity(initialVelocity*0.8)
            rightMotor.setVelocity(initialVelocity)
        else:
            leftMotor.setVelocity(initialVelocity)
            rightMotor.setVelocity(initialVelocity)


    
