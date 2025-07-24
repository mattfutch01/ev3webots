import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.transforms import Affine2D
import math

class DataVisualizer:
    """Class for visualizing robot position, orientation, and ultrasonic sensor data."""
    
    def __init__(self, ultrasonic_x_offset, ultrasonic_y_offset, ultrasonic_theta_offset, 
                 min_x = -.3, max_x = 3, min_y = -3, max_y = 3,
                 position_marker_size=10, position_marker_color='b', ultrasonic_marker_size=5, ultrasonic_marker_color='r'):
        """
        Initializes DataVisualizer with offsets and marker properties for visualization.
        
        Args:
            ultrasonic_x_offset (float): Offset for ultrasonic sensor in the x-direction.
            ultrasonic_y_offset (float): Offset for ultrasonic sensor in the y-direction.
            ultrasonic_theta_offset (float): Offset for ultrasonic sensor orientation in radians.
        """
        self.ultrasonic_x_offset = ultrasonic_x_offset
        self.ultrasonic_y_offset = ultrasonic_y_offset
        self.ultrasonic_theta_offset = ultrasonic_theta_offset  # Offset in radians
        
        # Plot Parameters
        self.ultrasonic_marker_size = ultrasonic_marker_size
        self.position_marker_size = position_marker_size
        self.ultrasonic_marker_color = ultrasonic_marker_color
        self.position_marker_color = position_marker_color

        self.max_x = max_x
        self.min_x = min_x
        self.max_y = max_y
        self.min_y = min_y

    def transform_coordinates(self, coordinates):
        """
        Transforms coordinates from right-handed to left-handed coordinate system.
        
        Args:
            coordinates (list): List containing x and y coordinates.
        
        Returns:
            list: Transformed coordinates in left-handed system [x', y'].
        """
        return [coordinates[1], coordinates[0]]

    def visualize_data(self, position_list, theta_list, ultrasonic_distance_list):
        """
        Visualizes robot position, orientation, and ultrasonic sensor data.
        
        Args:
            position_list (list): List of robot positions [(x1, y1), (x2, y2), ...].
            theta_list (list): List of robot orientations in radians [theta1, theta2, ...].
            ultrasonic_distance_list (list): List of ultrasonic sensor distances from robot [(d1, d2, ...)].
        """
        plt.figure()

        for i in range(len(position_list)):
            transformed_position_coordinates = self.transform_coordinates(position_list[i])

            # Rotate heading based on theta
            t = Affine2D().rotate_deg(theta_list[i] * 180 / math.pi)
            
            # Plot robot position and orientation
            plt.plot(transformed_position_coordinates[0], transformed_position_coordinates[1], 
                     marker=MarkerStyle('^', 'bottom', t), linestyle='-', color=self.position_marker_color, markersize=self.position_marker_size)

            # Compute and plot transformed ultrasonic sensor coordinates
            ultrasonic_x_offset = self.ultrasonic_x_offset * math.cos(theta_list[i] + self.ultrasonic_theta_offset) - \
                                  self.ultrasonic_y_offset * math.sin(theta_list[i] + self.ultrasonic_theta_offset) + \
                                  ultrasonic_distance_list[i] * math.cos(theta_list[i] + self.ultrasonic_theta_offset)
            
            ultrasonic_y_offset = self.ultrasonic_x_offset * math.cos(theta_list[i] + self.ultrasonic_theta_offset) + \
                                  self.ultrasonic_y_offset * math.sin(theta_list[i] + self.ultrasonic_theta_offset) + \
                                  ultrasonic_distance_list[i] * math.sin(theta_list[i] + self.ultrasonic_theta_offset)

            transformed_ultrasonic_coordinates = self.transform_coordinates([ultrasonic_x_offset + position_list[i][0], ultrasonic_y_offset + position_list[i][1]])

            plt.plot(transformed_ultrasonic_coordinates[0], transformed_ultrasonic_coordinates[1], marker="o", linestyle='-', 
                     color=self.ultrasonic_marker_color, markersize=self.ultrasonic_marker_size)

        plt.grid()
        plt.xlabel('Y')
        plt.ylabel('X')
        plt.axis([self.max_y, self.min_y, self.min_x, self.max_x])
        plt.title('Robot Position and Ultrasonic Sensor Data')
        plt.show()
        exit()  # Note: The exit() function will terminate the program; ensure this is intended behavior.