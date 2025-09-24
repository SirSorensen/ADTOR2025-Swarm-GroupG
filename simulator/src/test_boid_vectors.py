import unittest
import math
import numpy as np
from unittest.mock import Mock, patch
from boid import Boid, calc_average_radian, calc_opposite_angle, ALIGN_COEFFICIENT, SEPARATION_COEFFICIENT, COHESION_COEFFICIENT, CLOSE_RANGE_RADIUS
from readings import Signal, MessageData


class TestBoidVectorCalculations(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a boid with basic parameters
        self.boid = Boid(id=1, pos=np.array([100.0, 100.0]), heading=0.0)
        
        # Mock the verbose attribute if it exists
        if not hasattr(self.boid, 'verbose'):
            self.boid.verbose = False
    
    def create_mock_signal(self, distance, bearing, heading=0.0, sensor_idx=0):
        """Helper method to create mock signal objects."""
        mock_signal = Mock(spec=Signal)
        mock_signal.distance = distance
        mock_signal.bearing = bearing
        mock_signal.sensor_idx = sensor_idx
        
        # Create mock message data
        mock_message = Mock(spec=MessageData)
        mock_message.heading = heading
        mock_signal.message = mock_message
        
        return mock_signal


class TestStandaloneFunctions(unittest.TestCase):
    """Test standalone vector calculation functions."""
    
    def test_calc_average_radian_single_angle(self):
        """Test averaging a single radian angle."""
        angles = [math.pi / 4]  # 45 degrees
        result = calc_average_radian(angles)
        self.assertAlmostEqual(result, math.pi / 4, places=6)
    
    def test_calc_average_radian_opposite_angles(self):
        """Test averaging opposite angles should result in average direction."""
        angles = [0, math.pi]  # 0° and 180°
        result = calc_average_radian(angles)
        # The average of opposite vectors should be close to 0 (or ±π)
        self.assertTrue(abs(result) < 0.1 or abs(abs(result) - math.pi) < 0.1)
    
    def test_calc_average_radian_quadrant_angles(self):
        """Test averaging angles from each quadrant."""
        angles = [0, math.pi/2, math.pi, 3*math.pi/2]  # 0°, 90°, 180°, 270°
        result = calc_average_radian(angles)
        # Should average to approximately 0 (all quadrants cancel out)
        self.assertAlmostEqual(result, 0, places=1)
    
    def test_calc_average_radian_wrap_around(self):
        """Test averaging angles that wrap around 2π."""
        angles = [0.1, 2*math.pi - 0.1]  # Very close to 0° from both sides
        result = calc_average_radian(angles)
        # Should be close to 0
        self.assertTrue(abs(result) < 0.2 or abs(abs(result) - 2*math.pi) < 0.2)
    
    def test_calc_opposite_angle_positive(self):
        """Test calculating opposite angle for positive input."""
        angle = math.pi / 4  # 45°
        result = calc_opposite_angle(angle)
        expected = angle - math.pi  # Should be -135°
        self.assertAlmostEqual(result, expected, places=6)
    
    def test_calc_opposite_angle_negative(self):
        """Test calculating opposite angle for negative input."""
        angle = -math.pi / 4  # -45°
        result = calc_opposite_angle(angle)
        expected = angle + math.pi  # Should be 135°
        self.assertAlmostEqual(result, expected, places=6)
    
    def test_calc_opposite_angle_zero(self):
        """Test calculating opposite angle for zero input."""
        angle = 0.0
        result = calc_opposite_angle(angle)
        expected = -math.pi  # Should be -180°
        self.assertAlmostEqual(result, expected, places=6)


class TestBoidVectorMethods(TestBoidVectorCalculations):
    """Test boid instance methods for vector calculations."""
    
    def test_get_close_boids_empty(self):
        """Test getting close boids when no signals are present."""
        self.boid.rab_signals = []
        close_boids = self.boid.get_close_boids()
        self.assertEqual(len(close_boids), 0)
    
    def test_get_close_boids_filtering(self):
        """Test that close boids are properly filtered by distance."""
        # Create signals at different distances
        close_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS - 10, 0)
        far_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, math.pi/2)
        
        self.boid.rab_signals = [close_signal, far_signal]
        close_boids = self.boid.get_close_boids()
        
        self.assertEqual(len(close_boids), 1)
        self.assertEqual(close_boids[0], close_signal)
    
    def test_get_far_boids_filtering(self):
        """Test that far boids are properly filtered by distance."""
        close_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS - 10, 0)
        far_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, math.pi/2)
        
        self.boid.rab_signals = [close_signal, far_signal]
        far_boids = self.boid.get_far_boids()
        
        self.assertEqual(len(far_boids), 1)
        self.assertEqual(far_boids[0], far_signal)
    
    def test_get_average_bearing_single_boid(self):
        """Test averaging bearing with a single boid."""
        signal = self.create_mock_signal(50, math.pi/4)  # 45 degrees
        
        result = self.boid.get_average_bearing([signal])
        expected = np.array([np.cos(math.pi/4), np.sin(math.pi/4)])
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)
    
    def test_get_average_bearing_multiple_boids(self):
        """Test averaging bearing with multiple boids."""
        signal1 = self.create_mock_signal(50, 0)  # 0 degrees
        signal2 = self.create_mock_signal(60, math.pi/2)  # 90 degrees
        
        result = self.boid.get_average_bearing([signal1, signal2])
        
        # Expected: average of (1,0) and (0,1) = (0.5, 0.5)
        expected = np.array([0.5, 0.5])
        np.testing.assert_array_almost_equal(result, expected, decimal=6)
    
    def test_calc_align_vector_no_boids(self):
        """Test alignment vector calculation with no boids."""
        self.boid.rab_signals = []
        
        result = self.boid.calc_align_vector()
        expected = np.array([0, 0])
        
        np.testing.assert_array_equal(result, expected)
    
    def test_calc_align_vector_with_boids(self):
        """Test alignment vector calculation with far boids."""
        # Create far boids with different headings
        signal1 = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, 0, heading=0)
        signal2 = self.create_mock_signal(CLOSE_RANGE_RADIUS + 20, math.pi/2, heading=math.pi/2)
        
        self.boid.rab_signals = [signal1, signal2]
        
        result = self.boid.calc_align_vector()
        
        # Should be normalized vector pointing in average heading direction
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(np.linalg.norm(result), 1.0, places=5)
    
    def test_calc_separation_vector_no_close_boids(self):
        """Test separation vector calculation with no close boids."""
        # Only far boids
        signal = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, 0)
        self.boid.rab_signals = [signal]
        
        result = self.boid.calc_separation_vector()
        expected = np.array([0, 0])
        
        np.testing.assert_array_equal(result, expected)
    
    def test_calc_separation_vector_with_close_boids(self):
        """Test separation vector calculation with close boids."""
        # Create close boid
        signal = self.create_mock_signal(CLOSE_RANGE_RADIUS - 10, math.pi/4)
        self.boid.rab_signals = [signal]
        
        result = self.boid.calc_separation_vector()
        
        # Should be a 2D vector pointing away from the close boid
        self.assertEqual(len(result), 2)
        # The vector should be non-zero since we have a close boid
        self.assertGreater(np.linalg.norm(result), 0)
    
    def test_calc_cohesion_vector_no_far_boids(self):
        """Test cohesion vector calculation with no far boids."""
        # Only close boids
        signal = self.create_mock_signal(CLOSE_RANGE_RADIUS - 10, 0)
        self.boid.rab_signals = [signal]
        
        result = self.boid.calc_cohesion_vector()
        expected = np.array([0, 0])
        
        np.testing.assert_array_equal(result, expected)
    
    def test_calc_cohesion_vector_with_far_boids(self):
        """Test cohesion vector calculation with far boids."""
        # Create far boids
        signal1 = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, 0)
        signal2 = self.create_mock_signal(CLOSE_RANGE_RADIUS + 20, math.pi/2)
        self.boid.rab_signals = [signal1, signal2]
        
        result = self.boid.calc_cohesion_vector()
        
        # Should be a 2D vector pointing toward the average position of far boids
        self.assertEqual(len(result), 2)
        # Should be non-zero since we have far boids
        self.assertGreater(np.linalg.norm(result), 0)
    
    @patch('boid.ALIGN_COEFFICIENT', 1.0)
    @patch('boid.SEPARATION_COEFFICIENT', 2.0)
    @patch('boid.COHESION_COEFFICIENT', 3.0)
    def test_get_total_vector_weighted_combination(self):
        """Test that total vector properly combines all three vectors with coefficients."""
        # Create mixed signals for different behaviors
        close_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS - 10, 0, heading=0)
        far_signal = self.create_mock_signal(CLOSE_RANGE_RADIUS + 10, math.pi/2, heading=math.pi/4)
        
        self.boid.rab_signals = [close_signal, far_signal]
        
        # Calculate individual vectors
        align_vec = self.boid.calc_align_vector()
        sep_vec = self.boid.calc_separation_vector()
        coh_vec = self.boid.calc_cohesion_vector()
        
        # Calculate total vector
        result = self.boid._get_target_vector()
        
        # Should equal the weighted sum with mocked coefficients
        expected = align_vec * 1.0 + sep_vec * 2.0 + coh_vec * 3.0
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)


class TestVectorMagnitudeAndDirection(TestBoidVectorCalculations):
    """Test vector properties like magnitude and direction."""
    
    def test_unit_vectors_from_bearings(self):
        """Test that bearing-based vectors are properly normalized."""
        bearings = [0, math.pi/2, math.pi, 3*math.pi/2]
        
        for bearing in bearings:
            signal = self.create_mock_signal(50, bearing)
            result = self.boid.get_average_bearing([signal])
            
            # Should be unit vector
            magnitude = np.linalg.norm(result)
            self.assertAlmostEqual(magnitude, 1.0, places=6)
    
    def test_vector_directions(self):
        """Test that vectors point in expected directions."""
        # Test specific bearing directions
        test_cases = [
            (0, [1, 0]),           # East
            (math.pi/2, [0, 1]),   # North
            (math.pi, [-1, 0]),    # West
            (3*math.pi/2, [0, -1]) # South
        ]
        
        for bearing, expected_direction in test_cases:
            signal = self.create_mock_signal(50, bearing)
            result = self.boid.get_average_bearing([signal])
            
            np.testing.assert_array_almost_equal(result, expected_direction, decimal=6)


if __name__ == '__main__':
    # Run the tests
    unittest.main(verbosity=2)