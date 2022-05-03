import cv2 as cv
import numpy as np
import time
import math
import sys
import traceback
import av
import time
import tellopy
import colorsys
import threading
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
        self.ids = None
        self.corners = None
        self.rejected = None
        self.rt_vec = None
        self.spacing = None
        self.run_process = None
        self.is_flying = False
        self.lock = threading.Lock()
        self.is_connection_failed = False
        
    def Setup(self):
        self.SetFlippedMatrix()
        self.SetMarkerLength(10)
        self.SetFont(cv.FONT_HERSHEY_PLAIN)
        self.SetMarkerSize(5)
        self.SetNumbersOfMarkersInDictionary(50)
        self.SetCameraCalibrations()
        self.SetSpacing(21)

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

    def SetSpacing(self, spacing):
        self.spacing = spacing

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

    def SetWaitKey(self, wait_in_milliseconds) -> None:
        self.waitkey = cv.waitKey(wait_in_milliseconds)
    
    def SetImage(self, frame) -> None:
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
    
    def GetMarkerIDLength(self) -> int:
        return self.GetMarkerIds().size
    
    def GetClosestMarkerDistance(self):
        return self.GetClosestMarker(0)
    
    def GetClosestMarkerIndex(self):
        return self.GetClosestMarker(1)
    
    def GetClosestMarkerId(self):
        return self.GetMarkerIds()[self.GetClosestMarkerIndex()]
    
    def GetClosestMarkerByCamera(self, function):
        if self.IsMarkerDetected():
            return function(self.GetClosestMarkerIndex())
        return None
        
    def GetClosestMarkerByCameraX(self):
        return self.GetClosestMarkerByCamera(self.GetCameraXPosition)

    def GetClosestMarkerByCameraY(self):
        return self.GetClosestMarkerByCamera(self.GetCameraYPosition)

    def GetClosestMarkerByCameraZ(self):
        return self.GetClosestMarkerByCamera(self.GetCameraZPosition)

    def GetClosestMarkerByCameraTheta(self):
        return self.GetClosestMarkerByCamera(self.GetCameraYaw)


    def GetClosestMarker(self, select = 1):
        if self.IsMarkerDetected():
            return self.FindSmallestValueAndIndex(self.GetMarkerZPosition)[select]
        return None
    
    def GetRotoTranslationVector(self) -> np.array:
        return self.rt_vec

    def GetTextString(self, name, coord, vec, index):
        return f"{name} {coord[0]}=%4.0f  {coord[1]}=%4.0f  {coord[2]}=%4.0f"%(vec(index)[0], vec(index)[1], vec(index)[2])

    def GetMarkerRotationVector(self, index) -> np.array:
        if index < 0:
            return self.GetRotoTranslationVector()[0][:]
        return self.GetRotoTranslationVector()[0][index, 0, :]
    
    def GetMarkerTranslationVector(self, index) -> np.array:
        if index < 0:
            return self.GetRotoTranslationVector()[1][:]
        return self.GetRotoTranslationVector()[1][index, 0, :]

    def GetMarkerXPosition(self, index = 0) -> float:
        return self.GetMarkerTranslationVector(index)[0] 
    
    def GetMarkerYPosition(self, index = 0) -> float:
        return self.GetMarkerTranslationVector(index)[1]

    def GetMarkerZPosition(self, index = 0) -> float:
        return self.GetMarkerTranslationVector(index)[2]

    def GetMarkerPsi(self, index = 0) -> float:
        return self.GetMarkerCameraEulerAngles(index)[0]
    
    def GetMarkerTheta(self, index = 0) -> float:
        return self.GetMarkerCameraEulerAngles(index)[1]
    
    def GetMarkerPhi(self, index = 0) -> float:
        return self.GetMarkerCameraEulerAngles(index)[2]    

    def GetRotationMatrix(self, index) -> np.matrix:
        if self.GetMarkerIds() is None:
            return None
        return np.matrix(cv.Rodrigues(self.GetMarkerRotationVector(index))[0]).T

    def GetMarkerCameraEulerAngles(self, index) -> np.array:
        return np.rad2deg(self.rotationMatrixToEulerAngles(self.GetFlippedMatrix()*self.GetRotationMatrix(index)))

    def GetCameraTranslationVector(self, index):
        if self.GetMarkerIds() is None:
            return None
        return -self.GetRotationMatrix(index)*np.matrix(self.GetMarkerTranslationVector(index)).T

    def GetCameraXPosition(self, index = 0) -> float:
        return self.GetCameraTranslationVector(index)[0]
    
    def GetCameraYPosition(self, index = 0) -> float:
        return self.GetCameraTranslationVector(index)[1]

    def GetCameraZPosition(self, index = 0) -> float:
        return self.GetCameraTranslationVector(index)[2]
    
    def GetCameraRoll(self, index = 0) -> float:
        return self.GetMarkerPsi(index)
    
    def GetCameraPitch(self, index = 0) -> float:
        return self.GetMarkerPhi(index)

    def GetCameraYaw(self, index = 0) -> float:
        return self.GetMarkerTheta(index)
    
    def GetSpacing(self)-> int:
        return self.spacing
    
    def GetVelocity(self, function):
        elapsed = time.time()
        vel1 = function()
        time.sleep(0.01)
        vel2 = function()
        if vel1 != None and vel2 != None:
            return round((vel2[0,0]- vel1[0,0])/(time.time() - elapsed),2)
        return 0.0
    
    def GetConnectionFailureFlag(self):
        return self.is_connection_failed
# Boolean

    def IsDroneConnected(self) -> bool:
        return self.is_drone_connected
    
    def IsDroneStreaming(self) -> bool:
        return self.is_drone_streaming

    def IsMarkerDetected(self) -> bool:
        return self.GetMarkerIds() is not None
    
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    def ExitStream(self) -> bool:
        return self.GetKey() == 27 or self.GetKey() == ord('q')
    

# Others

    def FindSmallestValueAndIndex(self, function):
        index = 0
        minimum = function(index)
        for i in range(self.GetMarkerIds().size-1):
            if function(i+1) < minimum:
                minimum = function(i+1)
                index = i
        return minimum, index

    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def HSV2RGB(self, h, s, v):
        return tuple(round(i * 255) for i in colorsys.hsv_to_rgb(h/360.0,s,v))

    def CycleColor(self, index, phase=0):
        return self.HSV2RGB(30*index + phase, 1, 1)

    def ConnectDrone(self) -> None:
        try:
            self.SetDrone(tellopy.Tello())
            self.GetDrone().connect()
            self.GetDrone().wait_for_connection(60.0)
            self.SetContainer(3)
            self.SetConnectionStatus(True)
        except Exception as ex:
            self.Exception(ex)
            self.SetConnectionErrorFlag(True)

    def SetConnectionErrorFlag(self, flag):
        self.is_connection_failed = flag

    def Exception(self, ex):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

    def LoadFile(self, filename):
        return np.loadtxt(filename, delimiter=',')

    def FindMarkers(self, image) -> tuple:
        self.corners, self.ids, self.rejected = cv.aruco.detectMarkers(
            image           = self.ConvertImageToGrayscale(image), 
            dictionary      = self.GetDictionary(),
            parameters      =self.GetParameters(),
            cameraMatrix    = self.GetCameraMatrix(),
            distCoeff       = self.GetDistortionCoefficients())

    def ConvertImageToGrayscale(self, image):
        return cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    def FindClosestMarker(self):
        return self.GetMarkerIds()

    def DrawDetectedMarkers(self) -> None:
        if self.IsMarkerDetected():
            self.DrawBoundingBoxesOnMarkers()
            self.SetRotoTranslationVector()
            for index in range(self.GetMarkerIDLength()):
                  self.DrawAxesOnMarkers(index)
                  self.DrawText(index)

    def DrawText(self, index, Draw=True):
        if Draw:
            self.WriteMarkerPosition(index)
            self.WriteMarkerOrientation(index)
            self.WriteCameraPosition(index)
            self.WriteCameraOrientation(index)

    def WriteMarkerPosition(self, index):
        self.WriteText(name     = "Marker Position:",
                       coord    = ["x", "y", "z"],
                       vec      = self.GetMarkerTranslationVector,
                       spacing  = 0,
                       index    = index)

    def WriteMarkerOrientation(self, index):
        self.WriteText(name     = "Marker Attitude:",
                       coord    = ["psi", "theta", "phi"],
                       vec      = self.GetMarkerCameraEulerAngles,
                       spacing  = 6*self.GetSpacing(),
                       index    = index)
        
    def WriteCameraPosition(self, index):
        self.WriteText(name     = "Camera Position:",
                       coord    = ["x'", "y'", "z'"],
                       vec      = self.GetCameraTranslationVector,
                       spacing  = 2*6*self.GetSpacing(),
                       index    = index)

    def WriteCameraOrientation(self, index):
        self.WriteText(name     = "Camera Attitude:",
                       coord    = ["psi'", "theta'", "phi'"],
                       vec      = self.GetMarkerCameraEulerAngles,
                       spacing  = 3*6*self.GetSpacing(),
                       index    = index)
        
    def WriteText(self, name, coord, vec, spacing, index):
        str_position = self.GetTextString(name, coord, vec, index)
        cv.putText(img          = self.GetImage(),
                   text         = str_position,
                   org          = (0, 21 + index*21 + spacing),
                   fontFace     = self.GetFont(),
                   fontScale    = 1.5,
                   color        = self.CycleColor(index, 120),
                   thickness    = 2,
                   lineType     = cv.LINE_AA)


    def DrawAxesOnMarkers(self, index):
        cv.aruco.drawAxis(self.GetImage(), 
                          self.GetCameraMatrix(),
                          self.GetDistortionCoefficients(),
                          self.GetMarkerRotationVector(index),
                          self.GetMarkerTranslationVector(index),
                          self.GetMarkerLength())

    def SetRotoTranslationVector(self):
        self.rt_vec = self.EstimatePoseFromSingeMarker()

    def EstimatePoseFromSingeMarker(self):
        return cv.aruco.estimatePoseSingleMarkers(self.GetMarkerCorners(),
                                                  self.GetMarkerLength(),
                                                  self.GetCameraMatrix(),
                                                  self.GetDistortionCoefficients())

    def DrawBoundingBoxesOnMarkers(self):
        cv.aruco.drawDetectedMarkers(self.GetImage(), self.GetMarkerCorners(), self.GetMarkerIds())

# Running processes and threads

    def Start(self):
        self.Run()

    def Run(self) -> None:
        if self.IsDroneConnected():
            self.SetArucoDictionaryForDetector()
            self.SetFrameSkip(300)
            for frame in self.GetContainer().decode(video=0):
                if self.SkipFrames():
                    continue
                self.lock.acquire()
                self.Stream(frame)
                self.lock.release()
                if self.ExitStream():
                    return self.End()
   
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
        self.MarkerDetection()
        cv.imshow('Original', self.GetImage())
        self.SetWaitKey(1)

    def MarkerDetection(self):
        self.FindMarkers(self.GetImage())
        self.DrawDetectedMarkers()

    def SkipFrames(self) -> int:
        self.SetFrameSkip(self.GetAmountFrameToSkip() - 1)
        return self.GetAmountFrameToSkip() > 0

    def End(self) -> None:
        self.DeleteCameraCalibration()
        self.Land()
        self.StopStream()
        self.DisconnectDrone()
        return "Complete!"

    def Land(self) -> None:
        if self.is_flying: 
            self.GetDrone.land()
            self.is_flying = False

    def StopStream(self) -> None:
        if self.IsDroneStreaming():
            cv.destroyAllWindows()
            self.SetStreamingStatus(False)

    def DeleteCameraCalibration(self):
        self.SetCameraMatrix(None)
        self.SetDistortionCoefficients(None)

    def DisconnectDrone(self) -> None:
        if self.IsDroneConnected() and not self.GetConnectionFailureFlag():
            self.GetDrone().quit()
            self.SetDrone(None)
            self.SetConnectionStatus(False)
