import unittest
from src.kalman.sensor import Sensor, SensorType
from numpy.random import seed, standard_normal
from numpy import add, subtract, allclose, arctan2, arccos, degrees
from numpy.linalg import norm
 
class TestSensor(unittest.TestCase): 
    def setUp(self):
        pass  

    def test_sensor_two_objects_corect_position(self):
         # Create actual positions
        seed(0)
        noisedPos = [add(n, o) for n, o in zip(standard_normal(3) * 1000, [0,0,0])]
        actualPos = [subtract(x, y) for x,y in zip(noisedPos, [0,0,0])]
      
        # Reset Seed
        seed(0)
        (poss, new) = Sensor([0,0,0],SensorType.Carthesian, [[[0,0,0]], [[0,0,0]]]).scan(0)
        self.assertTrue(new, "Expected new measurement. Got old.")
        self.assertEqual(len(poss), 2, "Expected one position for one object")
        self.assertTrue(allclose(poss[0], actualPos))

    def test_sensor_invalid_dimension(self):
        self.assertRaises(IndexError, Sensor, [0,0], SensorType.Carthesian, [[[0]]])
 
    def test_sensor_oldMeasurement(self):
        (_, new) = Sensor([0,0,0], SensorType.Carthesian, [[[0,0,0]]]).scan(1)
        self.assertFalse(new)

    def test_sensor_carthesian_correct(self):
        # Create actual positions
        seed(0)
        noisedPos = [add(n, o) for n, o in zip(standard_normal(3) * 1000, [0,0,0])]
        actualPos = [subtract(x, y) for x,y in zip(noisedPos, [0,0,0])]
      
        # Reset Seed
        seed(0)
        (pos, new) = Sensor([0,0,0],SensorType.Carthesian, [[[0,0,0]]]).scan(0)
        self.assertTrue(new, "Expected new measurement. Got old.")
        self.assertEqual(len(pos), 1, "Expected one position for one object")
        self.assertTrue(allclose(pos, actualPos))

    def test_sensor_3d_polar_coordinates_correct(self):
        # Generate actual positions
        seed(0)
        noisedPos = [add(n, o) for n, o in zip(standard_normal(3) * 1000, [0,0,0])]
        actualPos = [subtract(x, y) for x,y in zip(noisedPos, [0,0,0])]

        r = norm(actualPos)
        theta = degrees(arctan2(actualPos[1], actualPos[0]))
        phi = degrees(arccos(actualPos[2]/r))
        polarCoord = [r, theta, phi]

        seed(0)
        (pos, new) = Sensor([0,0,0],SensorType.Polar, [[[0,0,0]]]).scan(0)
        self.assertTrue(new)
        self.assertEqual(len(pos), 1)
        self.assertTrue(allclose(pos, polarCoord))      


    def test_sensor_2d_polar_coordinates_correct(self):
         # Generate actual positions
        seed(0)
        noisedPos = [add(n, o) for n, o in zip(standard_normal(3) * 1000, [0,0])]
        actualPos = [subtract(x, y) for x,y in zip(noisedPos, [0,0,0])]

        r = norm(actualPos)
        theta = degrees(arctan2(actualPos[1], actualPos[0]))
        polarCoord = [r, theta]

        seed(0)
        (pos, new) = Sensor([0,0], SensorType.Polar, [[[0,0]]]).scan(0)
        self.assertTrue(new)
        self.assertEqual(len(pos), 1)
        self.assertTrue(allclose(pos, polarCoord))   

if __name__ == '__main__':
    unittest.main()