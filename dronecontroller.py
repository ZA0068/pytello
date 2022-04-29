from Arucodetector import Arucodetector
from Controller import DroneController
import os
import concurrent.futures

class ArucoTelloController():
    def __init__(self):
        self.arucodetector = None
        self.dronecontroller = None
        self.detectorprocessor = None
        
    def Setup(self):
        self.SetDetector()
        self.SetController()
        
    def SetDetector(self):
        self.arucodetector = Arucodetector()
        self.arucodetector.Setup()
        self.arucodetector.ConnectDrone()
    
    def SetController(self):
        self.dronecontroller = DroneController()
        self.dronecontroller.Setup()
    
    def GetDetector(self):
        return self.arucodetector
        
    def GetController(self):
        return self.dronecontroller
    
    def Fly(self, takeoff=True):
        if takeoff:
            self.GetDetector().GetDrone().takeoff()
            self.GetDetector().is_flying = True
    
    def ControlX(self, input):
        self.GetController().SetX(input)
        return self.GetController().GetX()
    
    def ControlY(self, input):
        self.GetController().SetY(input)
        return self.GetController().GetY()
    
    def ControlZ(self, input):
        self.GetController().SetZ(input)
        return self.GetController().GetZ()
    
    def ControlTheta(self, input):
        self.GetController().SetTheta(input)
        return self.GetController().GetTheta()
    
    def GetVelocity(self, function):
        return self.GetDetector().GetVelocity(function)
    
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
            print(self.GetVelocityX())
    
    def Run(self):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            self.detectorprocessor = executor.submit(self.GetDetector().Run)
            self.velocity_updater = executor.submit(self.UpdateVelocity)
        return self.detectorprocessor.result()
    
    def End(self):
        os.system("netstat -ano | findstr :9000")    
        return "Ended"                