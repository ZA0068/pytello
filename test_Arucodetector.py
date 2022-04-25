import unittest
from Arucodetector import Arucodetector
import tellopy
import simpful as fuzzy
import av
class TestArucodetector(unittest.TestCase):

    def setUp(self):
        self.testdetector = Arucodetector()
        self.testdetector.Setup()
        self.testdetector.ConnectDrone()
        self.testdetector.Run()

    def tearDown(self):
        self.testdetector.End()
        del self.testdetector
        print('Testing are done.')

    def test_SetDrone(self):
        self.assertTrue(self.testdetector.IsDroneConnected())

    def test_SetCameraCalibration(self):
        self.assertIsNotNone(self.testdetector.GetCameraMatrix())
        self.assertIsNotNone(self.testdetector.GetDistortionCoefficients())

    def test_stream(self):
        self.assertTrue(self.testdetector.IsDroneStreaming())

    def test_container(self):
        self.assertIsInstance(self.testdetector.GetContainer(), av.container.input.InputContainer)

    def test_GetImageFromStream(self):
        self.assertIsNotNone(self.testdetector.GetImage())

    def test_FindAnyMarker(self):
        self.assertTrue(self.testdetector.IsMarkerDetected())


    def test_FindClosestMarker(self):
        self.assertTrue(self.testdetector.IsMarkerDetected())
        self.assertGreater(self.testdetector.GetClosestMarkerDistance(), -1)
        self.assertIsNotNone(self.testdetector.GetClosestMarkerId())

    def test_Find_marker_pos_coordinates(self):
        self.assertIsNotNone(self.testdetector.GetMarkerXPosition())
        self.assertIsNotNone(self.testdetector.GetMarkerYPosition())
        self.assertIsNotNone(self.testdetector.GetMarkerZPosition())

    def test_Find_marker_angles(self):
        self.assertIsNotNone(self.testdetector.GetMarkerPsi())
        self.assertIsNotNone(self.testdetector.GetMarkerTheta())
        self.assertIsNotNone(self.testdetector.GetMarkerPhi())

    def test_Find_camera_position(self):
        self.assertIsNotNone(self.testdetector.GetCameraXPosition())
        self.assertIsNotNone(self.testdetector.GetCameraYPosition())
        self.assertIsNotNone(self.testdetector.GetCameraZPosition())

    def test_Find_camera_angles(self):
        self.assertIsNotNone(self.testdetector.GetCameraRoll())
        self.assertIsNotNone(self.testdetector.GetCameraPitch())
        self.assertIsNotNone(self.testdetector.GetCameraYaw())

# class TestDroneController(unittest.TestCase):
#     def setUp(self):
#         self.drone_controller = Arucodetector()
#         self.drone_controller.SetDrone()
#         self.drone_controller.CalibrateCamera()
#         self.drone_controller.Start()

#     def tearDown(self):
#         self.drone_controller.End()
#         print("testing are done \n")

#     def test_SetDrone(self):
#         self.assertTrue(self.drone_controller.isconnected)
#         self.assertIsNotNone(self.drone_controller.cameraMatrix)
#         self.assertIsNotNone(self.drone_controller.distCoeffs)
#         self.assertTrue(self.drone_controller.isstreaming)

#     def test_Drone_controller_setup(self):
#         self.drone_controller.SetController()
#         controller = self.drone_controller.GetController()
#         self.assertIsInstance(controller, fuzzy.FuzzySystem)
#         self.assertNotEqual(controller._lvs.__len__(), 0)
#         self.assertNotEqual(controller._crispvalues.__len__(), 0)
#         self.assertNotEqual(controller._rules.__len__(), 0)

#     def test_Drone_controller_functionality(self):
#         self.drone_controller.SetController()
#         self.drone_controller.SetX(0)
#         self.assertEqual(self.drone_controller.GetX() ,0.0)
#         self.drone_controller.SetY(0)
#         self.assertEqual(self.drone_controller.GetY() , 0.0)
#         self.drone_controller.SetZ(30)
#         self.assertEqual(self.drone_controller.GetZ() , 0.0)
#         self.drone_controller.SetTheta(0)
#         self.assertEqual(self.drone_controller.GetTheta() , 0.0)

#     def test_Drone_controller_functionality_extremes(self):
#         self.drone_controller.SetController()
#         self.drone_controller.SetX(10)
#         self.assertEqual(self.drone_controller.GetX() , 40.0)
#         self.drone_controller.SetY(10)
#         self.assertEqual(self.drone_controller.GetY() , 40.0)
#         self.drone_controller.SetZ(20)
#         self.assertEqual(self.drone_controller.GetZ() , -40.0)
#         self.drone_controller.SetTheta(10)
#         self.assertEqual(self.drone_controller.GetTheta() , 18.0)

#         self.drone_controller.SetX(-10)
#         self.assertEqual(self.drone_controller.GetX() , -40.0)
#         self.drone_controller.SetY(-10)
#         self.assertEqual(self.drone_controller.GetY() , -40.0)
#         self.drone_controller.SetZ(40)
#         self.assertEqual(self.drone_controller.GetZ() , 40.0)
#         self.drone_controller.SetTheta(-10)
#         self.assertEqual(self.drone_controller.GetTheta() , -18.0)

#     def test_Drone_controller_functionality_beyondlimit(self):
#         self.drone_controller.SetController()
#         self.drone_controller.SetX(50)
#         self.assertEqual(self.drone_controller.GetX() , 40.0)
#         self.drone_controller.SetY(50)
#         self.assertEqual(self.drone_controller.GetY() , 40.0)
#         self.drone_controller.SetZ(15)
#         self.assertEqual(self.drone_controller.GetZ() , -40.0)
#         self.drone_controller.SetTheta(70)
#         self.assertEqual(self.drone_controller.GetTheta() , 40.0)

#         self.drone_controller.SetX(-50)
#         self.assertEqual(self.drone_controller.GetX() , -40.0)
#         self.drone_controller.SetY(-50)
#         self.assertEqual(self.drone_controller.GetY() , -40.0)
#         self.drone_controller.SetZ(100)
#         self.assertEqual(self.drone_controller.GetZ() , 40.0)
#         self.drone_controller.SetTheta(-70)
#         self.assertEqual(self.drone_controller.GetTheta() , -40.0)

# class TestDroneControllerLive(unittest.TestCase):

#     def setUp(self):
#         self.drone_controller = Arucodetector()
#         self.drone_controller.SetDrone()
#         self.drone_controller.CalibrateCamera()
#         self.drone_controller.SetController()
#         self.drone_controller.Start()

#     def tearDown(self):
#         self.drone_controller.End()
#         print("testing are done \n")

#     def test_Drone_Live(self):
#         self.drone_controller.Run()
#         self.assertTrue(self.drone_controller.IsMarkerDetected())
#         self.assertTrue(self.drone_controller.FindClosestMarker())
#         self.assertAlmostEqual(self.drone_controller.GetClosestCameraX(), 0.0, delta = 2)
#         self.assertAlmostEqual(self.drone_controller.ControlX(), 0.0, delta = 2)
#         self.assertAlmostEqual(self.drone_controller.GetClosestCameraY(), 0.0, delta = 2)
#         self.assertAlmostEqual(self.drone_controller.ControlY(), 0, delta = 2)

#     def test_Drone_Live_Z(self):
#         self.drone_controller.Run()
#         self.assertTrue(self.drone_controller.IsMarkerDetected())
#         self.assertTrue(self.drone_controller.FindClosestMarker())
#         self.assertAlmostEqual(self.drone_controller.GetClosestMarkerZ(), 30.0, delta = 2)
#         self.assertAlmostEqual(self.drone_controller.ControlZ(), 0, delta = 2)
        
#     def test_Drone_Live_Theta(self):
#         self.drone_controller.Run()
#         self.assertTrue(self.drone_controller.IsMarkerDetected())
#         self.assertTrue(self.drone_controller.FindClosestMarker())
#         self.assertAlmostEqual(self.drone_controller.GetClosestCameraYaw(), 0.0, delta = 2)
#         self.assertAlmostEqual(self.drone_controller.ControlYaw(), 0, delta = 2)

#     def test_Drone_Live_Concurrency(self):
#         self.drone_controller.Run()
#         self.assertTrue(self.drone_controller.IsMarkerDetected())
#         self.assertTrue(self.drone_controller.FindClosestMarker())

#     def test_Drone_Live_Velocity(self):
#         self.drone_controller.Run()
#         self.assertIsInstance(self.drone_controller.GetVelocityX(0), int)
#         self.assertIsInstance(self.drone_controller.GetVelocityY(0), int)
#         self.assertIsInstance(self.drone_controller.GetVelocityZ(0), int)
#         self.assertIsInstance(self.drone_controller.GetVelocityTheta(0), int)



#     def test_Drone_Live_Flight(self):
#         self.drone_controller.Run(False)
#         self.assertTrue(self.drone_controller.IsMarkerDetected())
#         self.assertIsNotNone(self.drone_controller.FindClosestMarker())

if __name__ == '__main__':
    unittest.main()