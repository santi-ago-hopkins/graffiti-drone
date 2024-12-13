from imageprocessing import Image
import numpy as np
import matplotlib.pyplot as plt
import rclpy #uncomment me before flashing
from rclpy.node import Node # uncomment  me
from geometry_msgs.msg import PoseArray, Point, Pose
from std_msgs.msg import Int32
import math 

class Planning(Node):
    def __init__(self, waypoints):
        #node stuff
        super().__init__('planner_node')

        # publish to /planner topic
        self.planner_publisher = self.create_publisher(
            Pose,
            '/path',
            1)
        
        #subscribe to /visit topic
        self.control_subscriber = self.create_subscriber(
            Int32, 
            '/visit',
            self.feed_callback,
            1
        )

        # self.speed_multiplier = 0.8
        self.path = None
        self.waypoints = waypoints
        self.resolution = 0.25 # one point every 25 cm (0.25 m)
        self.current_pose_ind = 0
        self.next_pose_ind = 1

        
    # create callback that gives next point when received 
    def feed_callback(self, msg: Int32):
        # get message and set that to current pose, message is current pose index
        self.next_pose_ind = msg + 1
        self.planner_publisher.publish(self.path[self.next_pose_ind])
        
    
    # simple nearest neighbors algorithm
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

    
    #may make sense to implement something like this later that allows us to slow the speed near the point
    def set_velocities(self):
        '''using self.multipler (determines how slow we go near the point), we want to return a list with velocities as a function of position
        allows us to slow the drone down closer to the POI'''
        pass


    # should return a self.path with added waypoints
    def interpolatePoints(self):    
        def calculate_distance(point1, point2):
            # Assuming points are in (x, y) format
            x1, y1 = point1
            x2, y2 = point2
            return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        #iterate through self.path
        new_path = []
        for i, point in enumerate(self.path):
            if i < len(self.path) - 1:
                current_point = point
                next_point = self.path[i + 1]
                dist = calculate_distance(self, current_point, next_point)

                if dist > self.resolution:
                    num_points_to_add = np.floor(dist/self.resolution)
                    step = (next_point - current_point) / (num_points_to_add + 1)

                    for j in range(1, num_points_to_add + 1):
                        interpolated_point = current_point + j * step
                        new_path.append(interpolated_point)



        new_path.append(self.path[-1])
        self.path = new_path

        return self.path

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
    nn_planner.plot() # plot me! 

    # publish 
    nn_planner.numpyToPoseArrayPublish()
