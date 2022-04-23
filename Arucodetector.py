import cv2 as cv
import numpy as np
import os
import time
import math
import sys
import traceback
import av
import time
import tellopy

class Arucodetector:
    def __init__(self):
        self.marker_size: int = 0
        self.total_markers_in_dictionary: int = 0
        self.is_drone_connected: bool = False
        self.is_drone_streaming: bool = False
        self.drone: tellopy.Tello = None
        self.container: av.container.input.InputContainer = None
        self.RotationMatrix: np.array = None
        self.marker_length: int = 0
        self.font: int = 0
        self.amount_frame_to_skip = 0
        self.waitkey = -1
        self.image = None
        self.attributes = None
        self.dictionary = None
        self.parameters = None
        
                
# Setters

    def Setup(self):
        self.SetFlippedMatrix()
        self.SetMarkerLength(10)
        self.SetFont(cv.FONT_HERSHEY_PLAIN)
        self.SetMarkerSize(5)
        self.SetNumbersOfMarkersInDictionary(50)
        self.SetCameraCalibrations()

    def SetNumbersOfMarkersInDictionary(self, totalMarkers) -> None:
        self.total_markers_in_dictionary = totalMarkers;

    def SetFont(self, font):
        self.font = font

    def SetMarkerSize(self, markerSize) -> None:
        self.marker_size = markerSize;

    def SetDrone(self, drone) -> None:
        self.drone = drone

    def SetConnectionStatus(self, status) -> None:
        self.is_drone_connected = status

    def SetStreamingStatus(self, status) -> None:
        self.is_drone_streaming = status

    def SetArucoDictionaryForDetector(self):
        self.SetAttribute()
        self.SetDictionary()
        self.SetParameters()

    def SetParameters(self):
        self.parameters = cv.aruco.DetectorParameters_create()

    def SetDictionary(self):
        self.dictionary = cv.aruco.Dictionary_get(self.GetAttribute())

    def SetAttribute(self):
        self.attributes = getattr(cv.aruco, f'DICT_{self.GetMarkerSize()}X{self.GetMarkerSize()}_{self.GetNumbersOfMarkersInDictionary()}')

    def SetContainer(self, retry) -> None:
        while self.container is None and 0 < retry:
            retry -= 1
            try:
                self.container = av.open(self.GetDrone().get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')
            finally:
                self.SetStreamingStatus(True)

    def SetMarkerLength(self, length) -> None:
        self.marker_length: int = length

    def SetFlippedMatrix(self) -> None:
        RotationMatrix = np.zeros((3,3), dtype=np.float32)
        RotationMatrix[0,0] =  1.0
        RotationMatrix[1,1] = -1.0
        RotationMatrix[2,2] = -1.0
        self.rotation_matrix = RotationMatrix

    def SetDistortionCoefficients(self, array) -> None:
        self.distCoeffs = array

    def SetCameraMatrix(self, array) -> None:
        self.cameraMatrix = array

    def SetFrameSkip(self, frameskip) -> None:
        self.amount_frame_to_skip = frameskip

    def SetCameraCalibrations(self) -> None:
        self.SetCameraMatrix(self.LoadFile('cameraMatrix.txt'))
        self.SetDistortionCoefficients(self.LoadFile('cameraDistortion.txt'))

    def SetWaitKey(self, wait_in_milliseconds):
        self.waitkey = cv.waitKey(wait_in_milliseconds)
    
    def SetImage(self, frame):
        self.image = cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR)

# Getters

    def GetAttribute(self):
        return self.attributes

    def GetFlippedMatrix(self) -> np.array:
        return self.rotation_matrix

    def GetCameraMatrix(self) -> np.array:
        return self.cameraMatrix

    def GetDictionary(self) -> dict:
        return self.dictionary

    def GetDistortionCoefficients(self) -> np.array:
        return self.distCoeffs
    
    def GetCameraMatrix(self) -> np.array:
        return self.cameraMatrix

    def GetDrone(self) -> tellopy.Tello:
        return self.drone

    def GetMarkerLength(self) -> int:
        return self.marker_length
    
    def GetMarkerSize(self) -> int:
        return self.marker_size

    def GetAmountFrameToSkip(self) -> int:
        return self.amount_frame_to_skip

    def GetNumbersOfMarkersInDictionary(self) -> int:
        return self.total_markers_in_dictionary

    def GetFont(self) -> int:
        return self.font

    def GetKey(self):
        return self.waitkey

    def GetContainer(self) -> av.container.input.InputContainer:
        return self.container

    def GetImage(self) -> np.array:
        return self.image
    
    def GetParameters(self):
        return self.parameters
    
    def GetAllMarkerProperties(self):
        return self.GetMarkerCorners(), self.GetMarkerIds(), self.GetRejectedCandidates()
    
    def GetMarkerCorners(self):
        return self.corners
    
    def GetMarkerIds(self):
        return self.ids
    
    def GetRejectedCandidates(self):
        return self.rejected
    
# Boolean

    def IsDroneConnected(self) -> bool:
        return self.is_drone_connected
    
    def IsDroneStreaming(self) -> bool:
        return self.is_drone_streaming

    def IsMarkerDetected(self) -> bool:
        return self.GetMarkerIds() is not None
    
    def ExitStream(self) -> bool:
        return self.GetKey() == 27 or self.GetKey() == ord('q')
    
# Others

    def ConnectDrone(self) -> None:
        try:
            self.SetDrone(tellopy.Tello())
            self.GetDrone().connect()
            self.GetDrone().wait_for_connection(60.0)
            self.SetContainer(3)
            self.SetConnectionStatus(True)
        except Exception as ex:
            self.Exception(ex)

    def Exception(self, ex):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

    def LoadFile(self, filename):
        return np.loadtxt(filename, delimiter=',')

    def FindMarkers(self, image) -> tuple:
        self.corners, self.ids, self.rejected = cv.aruco.detectMarkers(
            self.ConvertImageToGrayscale(image), 
            self.GetDictionary(),
            parameters=self.GetParameters(),
            cameraMatrix = self.GetCameraMatrix(),
            distCoeff = self.GetDistortionCoefficients())

    def ConvertImageToGrayscale(self, image):
        return cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    def FindClosestMarker(self):
        return self.GetMarkerIds()

    # def draw(self) -> np.ndarray:
    #     self.FindMarkers()
    #     if self.ids is not None:
    #         cv.aruco.drawDetectedMarkers(self.img, self.corners, self.ids)
    #         rt_vec = cv.aruco.estimatePoseSingleMarkers(self.corners, 10, self.cameraMatrix, self.distCoeffs)
    #         for i in range(self.ids.size):
    #               cv.aruco.drawAxis(self.img, self.cameraMatrix, self.distCoeffs, rt_vec[0][i,i,:], rt_vec[1][i,i,:], 1)
    #               str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(rt_vec[1][0,0,:][0], rt_vec[1][0,0,:][1], rt_vec[1][0,0,:][2])
    #               cv.putText(self.img, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv.LINE_AA)


    def Run(self) -> None:
        if self.IsDroneConnected():
            self.SetArucoDictionaryForDetector()
            self.SetFrameSkip(300)
            for frame in self.GetContainer().decode(video=0):
                if self.SkipFrames():
                    continue
                self.Stream(frame)
                if self.ExitStream():
                    break
   
    def Stream(self, frame):
        start_time = time.time()
        self.DisplayImage(frame)
        self.UpdateFrameSkip(start_time, self.GetTimeBase(frame))

    def UpdateFrameSkip(self, start_time, time_base) -> None:
        self.SetFrameSkip(int((time.time() - start_time)/time_base))

    def GetTimeBase(self, frame):
        if frame.time_base < 1.0/60:
            return 1.0/60
        return frame.time_base

    def DisplayImage(self, frame):
        self.SetImage(frame)
        self.FindMarkers(self.GetImage())
        cv.imshow('Original', self.GetImage())
        self.SetWaitKey(1)


    def SkipFrames(self) -> int:
        self.SetFrameSkip(self.GetAmountFrameToSkip() - 1)
        return self.GetAmountFrameToSkip() > 0

    def End(self) -> None:
        self.DeleteCameraCalibration()
        self.DisconnectDrone()
        self.StopStream()

    def StopStream(self) -> None:
        if self.IsDroneStreaming():
            try:
                cv.destroyAllWindows()
                self.SetStreamingStatus(False)
            except av.AVError as ave:
                print(ave)
                print('retry...')

    def DeleteCameraCalibration(self):
        self.SetCameraMatrix(None)
        self.SetDistortionCoefficients(None)

    def DisconnectDrone(self) -> None:
        if self.IsDroneConnected():
            self.GetDrone().quit()
            self.SetDrone(None)
            self.SetConnectionStatus(False)
