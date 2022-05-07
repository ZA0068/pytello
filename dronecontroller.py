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
    
    def LoadControllerFromFiles(self):
        X_input = self.LoadFile("X_control_curve_input.txt")
        X_output = self.LoadFile("X_control_curve_output.txt")
        Lateral_Controller = self.CreateDictionaryWithArrayPairs(X_input, X_output)
        Y_input = self.LoadFile("Y_control_curve_input.txt")
        Y_output = self.LoadFile("Y_control_curve_output.txt")
        Vertical_Controller = self.CreateDictionaryWithArrayPairs(Y_input, Y_output)
        Z_input = self.LoadFile("Z_control_curve_input.txt")
        Z_output = self.LoadFile("Z_control_curve_output.txt")
        Longitual_Controller = self.CreateDictionaryWithArrayPairs(Z_input, Z_output)
        Theta_input = self.LoadFile("Theta_control_curve_input.txt")
        Theta_output = self.LoadFile("Theta_control_curve_output.txt")
        Yaw_Controller = self.CreateDictionaryWithArrayPairs(Theta_input, Theta_output)
        return 0

    def CreateDictionaryWithArrayPairs(self, input_array, output_array):
        return {input_array[i]:output_array[i] for i in range(len(input_array))}

    def SetController(self):
        self.LoadControllerFromFiles()
        self.dronecontroller.Setup()
    
    def GetDetector(self):
        return self.arucodetector
        
    def GetController(self):
        return self.dronecontroller
    
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
            self.GetController().SetX(x)
            return self.GetController().GetX()
    
    def ControlVerticalPosition(self, y):
        if y is not None:
            self.GetController().SetY(y)
            return self.GetController().GetY()
    
    def ControlLongitualPosition(self, z):
        if z is not None:
            self.GetController().SetZ(z)
            return self.GetController().GetZ()

    def ControlYawAngle(self, theta):
        if theta is not None:
            self.GetController().SetTheta(theta)
            return self.GetController().GetTheta()
    
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