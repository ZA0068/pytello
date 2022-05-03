from Arucodetector import Arucodetector
from Controller import DroneController
import os
import time
import concurrent.futures
import psutil
import sys
class ArucoTelloController():
    def __init__(self):
        self.arucodetector = None
        self.dronecontroller = None
        self.detectorprocessor = None
        self.velocitythread = None
        self.pid = -1
        
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
            if self.GetDetector().IsMarkerDetected():
                print(self.GetVelocityX())
            time.sleep(0.01)
        return 0
    
    def Run(self):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            self.detectorprocessor = executor.submit(self.GetDetector().Run)
            self.velocitythread = executor.submit(self.UpdateVelocity)
        return self.detectorprocessor.result() , self.velocitythread.result()
    
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