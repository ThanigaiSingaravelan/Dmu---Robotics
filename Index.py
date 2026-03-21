import sim as vrep
import random
import time
import numpy as np
import matplotlib.pyplot as plt

class Robot():
    def __init__(self):
        res, self.rightMotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_blocking)
        res, self.leftMotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)

        res, self.robotBase = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_blocking)

        res, self.US1 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", vrep.simx_opmode_blocking)
        res, self.US2 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor2", vrep.simx_opmode_blocking)
        res, self.US3 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor3", vrep.simx_opmode_blocking)
        res, self.US4 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor4", vrep.simx_opmode_blocking)
        res, self.US5 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor5", vrep.simx_opmode_blocking)
        res, self.US6 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor6", vrep.simx_opmode_blocking)
        res, self.US7 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor7", vrep.simx_opmode_blocking)
        res, self.US8 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8", vrep.simx_opmode_blocking)
        res, self.US9 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor9", vrep.simx_opmode_blocking)
        res, self.US10 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor10", vrep.simx_opmode_blocking)
        res, self.US11 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor11", vrep.simx_opmode_blocking)
        res, self.US12 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12", vrep.simx_opmode_blocking)
        res, self.US13 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor13", vrep.simx_opmode_blocking)
        res, self.US14 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor14", vrep.simx_opmode_blocking)
        res, self.US15 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor15", vrep.simx_opmode_blocking)
        res, self.US16 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor16", vrep.simx_opmode_blocking)

        res = vrep.simxReadProximitySensor(clientID, self.US1, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US2, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US3, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US4, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US5, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US6, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US7, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US8, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US9, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US10, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US11, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US12, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US13, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US14, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US15, vrep.simx_opmode_streaming)
        res = vrep.simxReadProximitySensor(clientID, self.US16, vrep.simx_opmode_streaming)

    def getDistanceReading(self, frontMidSensor):
        res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, frontMidSensor, vrep.simx_opmode_buffer)
        x = np.array(detectedPoint)
        if detectionState == True:
            return np.linalg.norm(x)
        else:
            return 9999

    def getRobotPosition(self):
        res, position = vrep.simxGetObjectPosition(clientID, self.robotBase, -1, vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            return position
        else:
            return None

    def startRobot(self):
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 1, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 1, vrep.simx_opmode_blocking)

    def stopRobot(self):
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, vrep.simx_opmode_blocking)
        print("Robot stopped.")

    def turnRobot(self, turnRight, turnLeft):
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, turnRight, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, turnLeft, vrep.simx_opmode_blocking)

    def Randomwander(self):
        distance_to_travel = random.uniform(1, 3)
        turn_angle = random.uniform(-90, 90)
        turn_direction = random.choice([-1, 1])

        turn_speed = turn_direction * (distance_to_travel / 2)
        forward_speed = random.uniform(1, 3)

        print(f"Traveling forward with speed {forward_speed} and turning {turn_angle} degrees.")
        res = vrep.simxSetJointTargetVelocity(clientID, self.rightMotor, forward_speed + turn_speed, vrep.simx_opmode_blocking)
        res = vrep.simxSetJointTargetVelocity(clientID, self.leftMotor, forward_speed - turn_speed, vrep.simx_opmode_blocking)

    def wallfollowing(self, tolerancevalue):
        if robot.getDistanceReading(robot.US1) < (0.8 - tolerancevalue):
            print(f"US1 Detected. Turning right.")
            robot.turnRobot(2.1, -2.1)
        elif robot.getDistanceReading(robot.US2) < (0.8 - tolerancevalue):
            print(f"US2 Detected. Turning right.")
            robot.turnRobot(2, -2)
        elif robot.getDistanceReading(robot.US3) < (0.8 - tolerancevalue):
            print(f"US3 Detected. Turning right.")
            robot.turnRobot(1.9, -1.9)
        elif robot.getDistanceReading(robot.US4) < (0.8 - tolerancevalue):
            print(f"US4 Detected. Turning right.")
            robot.turnRobot(2, -2)
        elif robot.getDistanceReading(robot.US5) < (0.8 - tolerancevalue):
            print(f"US5 Detected. Turning right.")
            robot.turnRobot(2.1, -2.1)
        elif robot.getDistanceReading(robot.US6) < (0.8 - tolerancevalue):
            print(f"US6 Detected. Slight left turn.")
            robot.turnRobot(0.1, 0.2)
        elif robot.getDistanceReading(robot.US7) < (0.8 - tolerancevalue):
            print(f"US7 Detected. Slight left turn.")
            robot.turnRobot(0.2, 0.1)
        elif robot.getDistanceReading(robot.US8) < (0.4 - tolerancevalue):
            print(f"US8 Detected (close). Slight left turn.")
            robot.turnRobot(0.3, 0.1)
        elif robot.getDistanceReading(robot.US8) < (0.8 - tolerancevalue):
            print(f"US8 Detected. Slight left turn.")
            robot.turnRobot(0.1, 0.4)
        elif robot.getDistanceReading(robot.US9) < (1.8 - tolerancevalue):
            print(f"US9 Detected. Adjusting left.")
            robot.turnRobot(0.1, 1.0)
        elif robot.getDistanceReading(robot.US10) < (0.8 - tolerancevalue):
            print(f"US10 Detected. Adjusting left.")
            robot.turnRobot(0.2, 1.1)
        elif robot.getDistanceReading(robot.US11) < (0.8 - tolerancevalue):
            print(f"US11 Detected. Adjusting left.")
            robot.turnRobot(0.3, 1.2)
        elif robot.getDistanceReading(robot.US12) < (0.8 - tolerancevalue):
            print(f"US12 Detected. Adjusting right.")
            robot.turnRobot(0.4, 1.3)
        elif robot.getDistanceReading(robot.US13) < (1.8 - tolerancevalue):
            print(f"US13 Detected. Adjusting right.")
            robot.turnRobot(1.0, 0.1)
        elif robot.getDistanceReading(robot.US14) < (0.4 - tolerancevalue):
            print(f"US14 Detected (close). Adjusting right.")
            robot.turnRobot(0.1, 0.3)
        elif robot.getDistanceReading(robot.US14) < (1.8 - tolerancevalue):
            print(f"US14 Detected. Adjusting right.")
            robot.turnRobot(0.4, 0.1)
        elif robot.getDistanceReading(robot.US15) < (0.8 - tolerancevalue):
            print(f"US15 Detected. Adjusting right.")
            robot.turnRobot(0.5, 0.2)
        else:
            print(f"No sensors Detected. Executing random wander.")
            robot.Randomwander()


# Connect to V-REP
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
    robot = Robot()
    positions = []
    time_positions = []

    try:
        tolerancevalue = 0.05
        start_time = time.time()
        while True:
            robot.startRobot()
            robot.wallfollowing(tolerancevalue)
            position = robot.getRobotPosition()
            if position:
                positions.append(position[:2])
                time_positions.append(time.time() - start_time)
            if time.time() - start_time > 900:
                break
    except KeyboardInterrupt:
        pass

    robot.stopRobot()
    print("End of program....")

    if positions:
        positions = np.array(positions)
        plt.figure(figsize=(8, 8))
        plt.plot(positions[:, 0], positions[:, 1], marker='o', color='red', label="Pioneer Simulation")
        plt.title("Pioneer Path")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.grid(True)
        plt.legend()
        plt.show()

    if time_positions:
        x_positions = []
        y_positions = []
        z_positions = []
        for position in positions:
            x_positions.append(position[0])
            y_positions.append(position[1])
            z_positions.append(0)

        plt.figure(figsize=(8, 6))
        plt.plot(time_positions, x_positions, label="Simulation X(m)")
        plt.plot(time_positions, y_positions, label="Simulation Y(m)")
        plt.plot(time_positions, z_positions, label="Simulation Z(m)", linestyle='--')
        plt.title("Pioneer Position Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.grid(True)
        plt.show()

else:
    print('Failed to connect to remote API server')
