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
    
    def SetController(self):
        self.dronecontroller = DroneController()
        self.dronecontroller.Setup()
    
    def GetDetector(self):
        return self.arucodetector
        
    def GetController(self):
        return self.dronecontroller
    
    def Run(self):
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            self.detectorprocessor = executor.submit(self.GetDetector().Run)
        return self.detectorprocessor.result()
        
                    