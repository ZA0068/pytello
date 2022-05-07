import unittest


# from Arucodetector import Arucodetector
# import av
# import tellopy
# class TestArucodetector(unittest.TestCase):

#     def setUp(self):
#         self.testdetector = Arucodetector()
#         self.testdetector.Setup()
#         self.testdetector.ConnectDrone()
#         self.testdetector.Run()

#     def tearDown(self):
#         self.testdetector.End()
#         del self.testdetector
#         print('Testing are done.')

#     def test_SetDrone(self):
#         self.assertTrue(self.testdetector.IsDroneConnected())

#     def test_SetCameraCalibration(self):
#         self.assertIsNotNone(self.testdetector.GetCameraMatrix())
#         self.assertIsNotNone(self.testdetector.GetDistortionCoefficients())

#     def test_stream(self):
#         self.assertTrue(self.testdetector.IsDroneStreaming())

#     def test_container(self):
#         self.assertIsInstance(self.testdetector.GetContainer(), av.container.input.InputContainer)

#     def test_GetImageFromStream(self):
#         self.assertIsNotNone(self.testdetector.GetImage())

#     def test_FindAnyMarker(self):
#         self.assertTrue(self.testdetector.IsMarkerDetected())


#     def test_FindClosestMarker(self):
#         self.assertTrue(self.testdetector.IsMarkerDetected())
#         self.assertGreater(self.testdetector.GetClosestMarkerDistance(), -1)
#         self.assertIsNotNone(self.testdetector.GetClosestMarkerId())

#     def test_Find_marker_pos_coordinates(self):
#         self.assertIsNotNone(self.testdetector.GetMarkerXPosition())
#         self.assertIsNotNone(self.testdetector.GetMarkerYPosition())
#         self.assertIsNotNone(self.testdetector.GetMarkerZPosition())

#     def test_Find_marker_angles(self):
#         self.assertIsNotNone(self.testdetector.GetMarkerPsi())
#         self.assertIsNotNone(self.testdetector.GetMarkerTheta())
#         self.assertIsNotNone(self.testdetector.GetMarkerPhi())

#     def test_Find_camera_position(self):
#         self.assertIsNotNone(self.testdetector.GetCameraXPosition())
#         self.assertIsNotNone(self.testdetector.GetCameraYPosition())
#         self.assertIsNotNone(self.testdetector.GetCameraZPosition())

#     def test_Find_camera_angles(self):
#         self.assertIsNotNone(self.testdetector.GetCameraRoll())
#         self.assertIsNotNone(self.testdetector.GetCameraPitch())
#         self.assertIsNotNone(self.testdetector.GetCameraYaw())


from Controller import DroneController
import simpful as fuzzy
class TestDroneController(unittest.TestCase):
    
    def setUp(self):
        self.drone_controller = DroneController()
        self.drone_controller.SetLog()
        self.drone_controller.Setup()

    def tearDown(self):
        print("testing are done \n")

    # def test_Drone_controller_setup(self):
    #     controller = self.drone_controller.GetController()
    #     self.assertIsInstance(controller, fuzzy.FuzzySystem)
    #     self.assertNotEqual(controller._lvs.__len__(), 0)
    #     self.assertNotEqual(controller._rules.__len__(), 0)
# 
    # def test_Drone_controller_functionality_center(self):
        # self.drone_controller.SetX(0)
        # self.assertEqual(self.drone_controller.GetX() ,0.0)
        # self.drone_controller.SetY(0)
        # self.assertEqual(self.drone_controller.GetY() , 0.0)
        # self.drone_controller.SetZ(30)
        # self.assertEqual(self.drone_controller.GetZ() , 0.0)
        # self.drone_controller.SetTheta(0)
        # self.assertEqual(self.drone_controller.GetTheta() , 0.0)
# 
    # def test_Drone_controller_functionality_variants(self):
        # self.drone_controller.SetX(10)
        # self.assertEqual(self.drone_controller.GetX() , 0.13)
        # self.drone_controller.SetX(-10)
        # self.assertEqual(self.drone_controller.GetX() , -0.13)
        # 
        # self.drone_controller.SetY(10)
        # self.assertEqual(self.drone_controller.GetY() , -0.13)
        # self.drone_controller.SetY(-10)
        # self.assertEqual(self.drone_controller.GetY() , 0.13)
        # 
        # self.drone_controller.SetZ(20)
        # self.assertEqual(self.drone_controller.GetZ() , -0.47)
        # self.drone_controller.SetZ(40)
        # self.assertEqual(self.drone_controller.GetZ() , 0.47)
    #  
        # self.drone_controller.SetTheta(10)
        # self.assertEqual(self.drone_controller.GetTheta() , -0.13)
        # self.drone_controller.SetTheta(-10)
        # self.assertEqual(self.drone_controller.GetTheta() , 0.13)
# 

    # def test_Drone_controller_functionality_extreme(self):
        # self.drone_controller.SetX(100)
        # self.assertEqual(self.drone_controller.GetX() , 0.9)
        # self.drone_controller.SetY(100)
        # self.assertEqual(self.drone_controller.GetY() , -0.9)
        # self.drone_controller.SetZ(60)
        # self.assertEqual(self.drone_controller.GetZ() , 0.93)
        # self.drone_controller.SetTheta(70)
        # self.assertEqual(self.drone_controller.GetTheta() , -0.73)
# 
        # self.drone_controller.SetX(-100)
        # self.assertEqual(self.drone_controller.GetX() , -0.9)
        # self.drone_controller.SetY(-100)
        # self.assertEqual(self.drone_controller.GetY() , 0.9)
        # self.drone_controller.SetZ(-60)
        # self.assertEqual(self.drone_controller.GetZ() , -0.93)
        # self.drone_controller.SetTheta(-70)
        # self.assertEqual(self.drone_controller.GetTheta() , 0.73)

    def test_Create_Control_Surface(self):
        result = self.drone_controller.CreateControlCurves()
        self.assertEqual(result, "Success!")
        
from dronecontroller import ArucoTelloController

class TestDroneControllerLive(unittest.TestCase):

    def setUp(self):
        print("testing the drone live")
        self.drone = ArucoTelloController()
        self.drone.Setup()
        self.returner, self.velocity_return_value, self.control_return_value = self.drone.Run(fly = True)
        
    def tearDown(self):
        self.drone.End()
        print('Testing are done.')

    def test_Drone_Live(self):
        self.assertEqual(self.returner, "Complete!")
        
    def test_Drone_Live_Marker(self):
        self.assertTrue(self.drone.GetDetector().IsMarkerDetected())
        self.assertIsNotNone(self.drone.GetDetector().GetClosestMarker())
        
    def test_Drone_Live_X(self):
        self.assertAlmostEqual(self.drone.GetDetector().GetClosestMarkerByCameraX(), 0.0, delta = 2)
        self.assertAlmostEqual(self.drone.ControlX(self.drone.GetDetector().GetClosestMarkerByCameraX()), 0.0, delta = 2)
 
    def test_Drone_Live_Y(self):
        self.assertAlmostEqual(self.drone.GetDetector().GetClosestMarkerByCameraY(), 0.0, delta = 2)
        self.assertAlmostEqual(self.drone.ControlY(self.drone.GetDetector().GetClosestMarkerByCameraY()), 0.0, delta = 2)

 
    def test_Drone_Live_Z(self):
        self.assertAlmostEqual(self.drone.GetDetector().GetClosestMarkerByCameraZ(), 30.0, delta = 2)
        self.assertAlmostEqual(self.drone.ControlZ(self.drone.GetDetector().GetClosestMarkerByCameraZ()), 0.0, delta = 2)
    

    def test_Drone_Live_Theta(self):
        self.assertAlmostEqual(self.drone.GetDetector().GetClosestMarkerByCameraTheta(), 0.0, delta = 2)
        self.assertAlmostEqual(self.drone.ControlTheta(self.drone.GetDetector().GetClosestMarkerByCameraTheta()), 0.0, delta = 2)

 
    def test_Drone_Live_Velocity(self):
        self.assertIsInstance(self.drone.GetVelocityX(), float)
        self.assertIsInstance(self.drone.GetVelocityY(), float)
        self.assertIsInstance(self.drone.GetVelocityZ(), float)
        self.assertIsInstance(self.drone.GetVelocityTheta(), float)
        
    def test_Drone_Live_Flight(self):
        self.assertEqual(self.returner, "Complete!")
        self.assertEqual(self.velocity_return_value, 0)
        self.assertEqual(self.control_return_value, 0)

if __name__ == '__main__':
    unittest.main()