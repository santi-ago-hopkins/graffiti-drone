from imageprocessing import Image
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Pose

class Planning(Node):
    def __init__(self, waypoints):
        #node stuff
        super().__init__('planner_node')

        # publish to /planner topic
        self.control_publisher = self.create_publisher(
            PoseArray,
            '/path',
            1)


        self.speed = 1.0 
        self.speed_multiplier = 0.8
        self.path = None
        self.waypoints = waypoints

    #convert np array to poses to publish
    def numpyToPoseArrayPublish(self):
        poses = PoseArray()
        poses.header.frame_id = "global_path" 
        poses.header.stamp = rclpy.Time.now()
        
        for point in self.path:
            pose = Pose()
            pose.position = Point(x=float(point[0]), y=float(point[1]), z=0)
            poses.poses.append(pose)

        #publish to planner topic
        self.control_publisher.publish(poses)        



    #very unoptimized
    def nearestNeighborPath(self):
        def distance(point1, point2):
            return np.linalg.norm(np.array(point1) - np.array(point2))

        def find_closest_point(current_point, unvisited):
            distances = np.linalg.norm(unvisited - np.array(current_point), axis=1)
            closest_index = np.argmin(distances)
            return unvisited[closest_index], closest_index
    
        current_point = (0, 0)

        unvisited = np.array(self.waypoints)
        self.path = [current_point]
        
        while len(unvisited) > 0:
            closest_point, index = find_closest_point(current_point, unvisited)
            self.path.append(tuple(closest_point))
            current_point = closest_point
            unvisited = np.delete(unvisited, index, axis=0)
        
        self.path.append((0,0))

        return self.path

    
    # may make sense to implement something like this later that allows us to slow the speed near the point
    def set_velocities(self):
        '''using self.multipler (determines how slow we go near the point), we want to return a list with velocities as a function of position
        allows us to slow the drone down closer to the POI'''
        pass

    def plot(self): 
        # separate x and y coords
        x, y = zip(*self.path)
        
        # calc differences
        dx = np.diff(x)
        dy = np.diff(y)
        
        plt.figure(figsize=(10, 10))
        plt.scatter(x, y, c='blue', s=50, label='Waypoints')
        plt.quiver(x[:-1], y[:-1], dx, dy, scale_units='xy', angles='xy', scale=1, color='red', label='Path')
        
        plt.title('Nearest Neighbor Planner')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

if __name__ == "__main__":
    #take image stuff
    img = Image('images/square.jpg')
    waypoints = img.getWaypoints()
    waypoints = img.pixels_to_xy()
    
    #planning stuff
    nn_planner = Planning(waypoints)
    nn_planner.nearestNeighborPath()
    nn_planner.plot()

    # publish 

    nn_planner.numpyToPoseArrayPublish()