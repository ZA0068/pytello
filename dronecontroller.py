from Arucodetector import Arucodetector
from Controller import DroneController
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
    
    # def Fly(self, takeoff=True):
    #     if takeoff:
    #         self.GetDetector().GetDrone().takeoff()
    #         self.GetDetector().is_flying = True
    
    def Run(self):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            self.detectorprocessor = executor.submit(self.GetDetector().Run)
        return self.detectorprocessor.result()
        
                    