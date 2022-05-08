from Arucodetector import Arucodetector
import time
import concurrent.futures
import threading
import psutil
import numpy as np
class ArucoTelloController():
    def __init__(self):
        self.arucodetector = None
        self.dronecontroller = None
        self.detectorprocessor = None
        self.velocitythread = None
        self.controllerthread = None
        self.pid = -1
        self.lock = threading.Lock()
        
    def Setup(self, set_detector=True, set_controller=True):
        if set_detector:
            self.SetDetector()
        if set_controller:
            self.SetController()
        
    def SetDetector(self):
        self.arucodetector = Arucodetector()
        self.arucodetector.Setup()
        self.arucodetector.ConnectDrone()
    
    def LoadFile(self, filename):
        return np.loadtxt(filename, delimiter=',')
    
    def LoadControllerArraysFromFiles(self):
        X_input = self.LoadFile("X_control_curve_input.txt")
        X_output = self.LoadFile("X_control_curve_output.txt")
        Y_input = self.LoadFile("Y_control_curve_input.txt")
        Y_output = self.LoadFile("Y_control_curve_output.txt")
        Z_input = self.LoadFile("Z_control_curve_input.txt")
        Z_output = self.LoadFile("Z_control_curve_output.txt")
        Theta_input = self.LoadFile("Theta_control_curve_input.txt")
        Theta_output = self.LoadFile("Theta_control_curve_output.txt")
        return [X_input, Y_input, Z_input, Theta_input], [X_output, Y_output, Z_output, Theta_output]

    def CreateControllersFromArrays(self, controller_arrays):
        self.lateral_controller = self.CreateDictionaryWithArrayPairs(controller_arrays[0][0], controller_arrays[1][0])
        self.vertical_controller = self.CreateDictionaryWithArrayPairs(controller_arrays[0][1], controller_arrays[1][1])
        self.longitual_controller = self.CreateDictionaryWithArrayPairs(controller_arrays[0][2], controller_arrays[1][2])
        self.yaw_controller = self.CreateDictionaryWithArrayPairs(controller_arrays[0][3], controller_arrays[1][3])

    def CreateDictionaryWithArrayPairs(self, input_array, output_array):
        return {input_array[i]:output_array[i] for i in range(len(input_array))}

    def SetController(self):
        self.CreateControllersFromArrays(self.LoadControllerArraysFromFiles())
    
    def GetDetector(self):
        return self.arucodetector
        
    def GetControllers(self):
        return self.lateral_controller, self.vertical_controller, self.longitual_controller, self.yaw_controller
    
    def GetLateralController(self):
        return self.GetControllers()[0]
    
    def GetVerticalController(self):
        return self.GetControllers()[1]

    def GetLongitualController(self):
        return self.GetControllers()[2]
    
    def GetYawController(self):
        return self.GetControllers()[3]
    
    def Fly(self, takeoff=True):
        if takeoff:
            self.GetDetector().GetDrone().takeoff()
            self.GetDetector().is_flying = True
    
    def GetVelocity(self, function):
        sum_of_velocities = 0
        steps_size_for_resolution = 5
        for i in range(steps_size_for_resolution):
            sum_of_velocities += self.GetDetector().GetVelocity(function)
        return sum_of_velocities/steps_size_for_resolution
    
    def GetVelocityX(self):
        return self.GetVelocity(self.GetDetector().GetClosestMarkerByCameraX)
    
    def GetVelocityY(self):
        return self.GetVelocity(self.GetDetector().GetClosestMarkerByCameraY)
    
    def GetVelocityZ(self):
        return self.GetVelocity(self.GetDetector().GetClosestMarkerByCameraZ)
    
    def GetVelocityTheta(self):
        return self.GetVelocity(self.GetDetector().GetClosestMarkerByCameraTheta)
    
    def UpdateVelocity(self):
        while self.GetDetector().IsDroneStreaming():
            if self.GetDetector().IsMarkerDetected():
                self.GetVelocityX()
                self.GetVelocityY()
                self.GetVelocityZ()
                self.GetVelocityTheta()
            time.sleep(0.001)
        return 0
    
    def ControlDrone(self):
        while self.GetDetector().IsDroneStreaming():
            self.ControlPosition()
            time.sleep(0.001)
        return 0
    
    def ControlPosition(self):
        x, y, z, theta = self.GenerateControlSignals()
        self.SendControlSignalsToTheDrone(x, y, z, theta)

    def SendControlSignalsToTheDrone(self, x, y, z, theta):
        # if x is not None:
        #     self.GetDetector().GetDrone().set_roll(-x)
        # if y is not None:
        #     self.GetDetector().GetDrone().set_throttle(-y)
        if z is not None:
            self.GetDetector().GetDrone().set_pitch(-z)
        if theta is not None:
            self.GetDetector().GetDrone().set_yaw(-theta)

    def GetCameraPositions(self):
        return (self.GetDetector().GetClosestMarkerByCameraX(False),
                self.GetDetector().GetClosestMarkerByCameraY(False),
                self.GetDetector().GetClosestMarkerByCameraZ(False),
                self.GetDetector().GetClosestMarkerByCameraTheta(False))

    
    def GenerateControlSignals(self):
        x, y, z, theta = self.GetCameraPositions()
        return (self.ControlLateralPosition(x), 
                self.ControlVerticalPosition(y), 
                self.ControlLongitualPosition(z), 
                self.ControlYawAngle(theta))
    
    def ControlLateralPosition(self, x):
        if x is not None:
            x = self.EncapsulateBoundary(x, -100, 100)
            return self.GetLateralController()[x]

    
    def ControlVerticalPosition(self, y):
        if y is not None:
            y = self.EncapsulateBoundary(y, -100, 100)
            return self.GetVerticalController()[y]
    
    def ControlLongitualPosition(self, z):
        if z is not None:
            z = self.EncapsulateBoundary(z, 0, 60)
            return self.GetLongitualController()[z]

    def ControlYawAngle(self, theta):
        if theta is not None:
            theta = self.EncapsulateBoundary(theta, -70, 70)
            return self.GetYawController()[theta]

    def EncapsulateBoundary(self, x, minimum: float, maximum: float):
        if x < minimum:
            return minimum
        elif x > maximum:
            return maximum
        else:
            return round(float(x), 0)
    
    def Run(self, run=True, fly = False):
        if run:
            try:
                self.Fly(fly)
                with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                    self.detectorprocessor = executor.submit(self.GetDetector().Run)
                    self.velocitythread = executor.submit(self.UpdateVelocity)
                    self.controllerthread = executor.submit(self.ControlDrone)
            except Exception as e:
                return e
            finally:
                return self.detectorprocessor.result(), self.velocitythread.result(), self.controllerthread.result()
        else:
            return "Complete!", 0, 0
    
    def End(self):
        if self.GetDetector().IsDroneConnected():
            self.StopStreamingDrone()
        return "Ended"

    def StopStreamingDrone(self):
        try:
            connections = psutil.net_connections()
            port = 9000
            for con in connections:
                pid = self.GetPIDFromPort(port, con)
                if(pid != -1):
                    break
            p = psutil.Process(pid)
  
        except Exception as ex:
            print(ex)
        finally:
            p.terminate()

    def GetPIDFromPort(self, port, con):
        self.GetRightAddressPID(port, con)
        self.GetLeftAddressPID(port, con)
        return self.pid

    def GetLeftAddressPID(self, port, con):
        if self.CheckAddressPortForPID(con.laddr, port):
            self.SetPID(con.pid)

    def GetRightAddressPID(self, port, con):
        if self.CheckAddressPortForPID(con.raddr, port):
            self.SetPID(con.pid)

    def SetPID(self, pid):
        self.pid = pid
        
    def CheckAddressPortForPID(self, address, port):
        if address != tuple():
            if address.port == port:
                return True
        return False