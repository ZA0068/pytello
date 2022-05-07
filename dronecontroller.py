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

    def CreateControllersFromArrays(self, controllerarrays):
        self.Lateral_Controller = self.CreateDictionaryWithArrayPairs(controllerarrays[0][0], controllerarrays[1][0])
        self.Vertical_Controller = self.CreateDictionaryWithArrayPairs(controllerarrays[0][1], controllerarrays[1][1])
        self.Longitual_Controller = self.CreateDictionaryWithArrayPairs(controllerarrays[0][2], controllerarrays[1][2])
        self.Yaw_Controller = self.CreateDictionaryWithArrayPairs(controllerarrays[0][3], controllerarrays[1][3])

    def CreateDictionaryWithArrayPairs(self, input_array, output_array):
        return {input_array[i]:output_array[i] for i in range(len(input_array))}

    def SetController(self):
        self.CreateControllersFromArrays(self.LoadControllerArraysFromFiles())
    
    def GetDetector(self):
        return self.arucodetector
        
    def GetControllers(self):
        return self.Lateral_Controller, self.Vertical_Controller, self.Longitual_Controller, self.Yaw_Controller
    
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
                self.lock.acquire()
                print(self.GetVelocityZ())
                self.lock.release()
            time.sleep(0.001)
        return 0
    
    def ControlDrone(self):
        while self.GetDetector().IsDroneStreaming():
            self.lock.acquire()
            self.ControlPosition()
            self.lock.release()
            time.sleep(0.001)
        return 0
    
    def ControlPosition(self):
        x, y, z, theta = self.GetControlPositions()
        print(x, y, z, theta)
        self.SendControlPositionsToDrone(x, y, z, theta)

    def SendControlPositionsToDrone(self, x, y, z, theta):
        if x is not None:
            self.GetDetector().GetDrone().set_roll(x)
        if y is not None:
            self.GetDetector().GetDrone().set_throttle(y)
        if z is not None:
            self.GetDetector().GetDrone().set_pitch(z)
        if theta is not None:
            self.GetDetector().GetDrone().set_yaw(theta)

    def GetControlPositions(self):
        x = self.ControlLateralPosition(self.GetDetector().GetClosestMarkerByCameraX())
        y = self.ControlVerticalPosition(self.GetDetector().GetClosestMarkerByCameraY())
        z = self.ControlLongitualPosition(self.GetDetector().GetClosestMarkerByCameraZ())
        theta = self.ControlYawAngle(self.GetDetector().GetClosestMarkerByCameraTheta())
        return x,y,z,theta
    
    def ControlLateralPosition(self, x):
        if x is not None:
            self.GetControllers().SetX(x)
            return self.GetControllers().GetX()
    
    def ControlVerticalPosition(self, y):
        if y is not None:
            self.GetControllers().SetY(y)
            return self.GetControllers().GetY()
    
    def ControlLongitualPosition(self, z):
        if z is not None:
            self.GetControllers().SetZ(z)
            return self.GetControllers().GetZ()

    def ControlYawAngle(self, theta):
        if theta is not None:
            self.GetControllers().SetTheta(theta)
            return self.GetControllers().GetTheta()
    
    def Run(self, run=True, fly = False):
        self.Fly(fly)
        if run:
            try:
                with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                    self.detectorprocessor = executor.submit(self.GetDetector().Run)
                    self.velocitythread = executor.submit(self.UpdateVelocity)
                    self.controllerthread = executor.submit(self.ControlDrone)
            except Exception as e:
                return e
            finally:
                return self.detectorprocessor.result() , self.velocitythread.result(), self.controllerthread.result()
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