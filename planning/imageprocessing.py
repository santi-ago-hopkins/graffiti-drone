import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

class Image:
    def __init__(self, path_to_image):
        # Read the image and detect edges
        self.image = cv.imread(path_to_image)
        self.image_height = self.image.shape[0]
        self.image_width = self.image.shape[1]
        self.gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        self.waypoints = []
        self.canvas_height = 10.0 # in meters
        self.canvas_width = 10.0 # also in meters 
        self.spray_can_radius = 0.05 # in meters

    def get_waypoints(self):
        # Apply Gaussian blur to reduce noise
        blurred = cv.GaussianBlur(self.gray, (5, 5), 0)

        # Use adaptive thresholding instead of Canny edge detection
        edges = cv.adaptiveThreshold(blurred, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 9)

        # Find contours
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Extract waypoints
        self.waypoints = []
        for contour in contours:
            # Option 1: Sample points along the contour
            epsilon = 0.001 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            self.waypoints.extend(approx.reshape(-1, 2))


        return self.waypoints
    
    def grid_planning(self):
        resized_image = cv.resize(self.gray, (int(self.canvas_width * 500), int(self.canvas_height * 500)))
        cv.imshow('resized_image', resized_image)

        _, binary_image = cv.threshold(resized_image, 127, 255, cv.THRESH_BINARY)
        print(binary_image)

        spray_can_square_side = self.spray_can_radius * np.sqrt(2)
        num_rows = int(self.canvas_height / spray_can_square_side)
        num_cols = int(self.canvas_width / spray_can_square_side)
        row_spacing = col_spacing = spray_can_square_side * 500

        print(num_rows, num_cols, row_spacing, col_spacing)
        # Identify waypoints in black regions
        for i in range(num_rows):
            for j in range(num_cols):
                # Define the grid cell boundaries
                x_start = int(j * col_spacing)
                y_start = int(i * row_spacing)
                x_end = int(x_start + col_spacing)
                y_end = int(y_start + row_spacing)

                # Extract the grid cell
                # print(y_start, y_end, x_start, x_end)
                grid_cell = binary_image[y_start:y_end, x_start:x_end]
                # print(grid_cell.shape)

                # Check if the grid cell contains any black pixels
                if np.all(grid_cell <= 127):  # Black pixels have intensity 0
                    # Add the center of the grid cell as a waypoint
                    x_center = int(x_start + col_spacing / 2)
                    y_center = int(y_start + row_spacing / 2)
                    self.waypoints.append((x_center, y_center))

        print('Waypoints: ', self.waypoints)

        # Visualize the identified black areas and waypoints
        image_with_waypoints = cv.cvtColor(resized_image, cv.COLOR_GRAY2BGR)
        for (x, y) in self.waypoints:
            cv.circle(image_with_waypoints, (x, y), 5, (0, 0, 255), -1)  # Draw waypoints as red dots

        # image before
        plt.figure(figsize=(12, 6))
        plt.subplot(1, 2, 1)
        plt.title("Binary Image")
        plt.imshow(binary_image, cmap='gray')
        plt.axis('off')

        # with waypoints
        plt.subplot(1, 2, 2)
        plt.title("Waypoints on Image")
        plt.imshow(cv.cvtColor(image_with_waypoints, cv.COLOR_BGR2RGB))
        plt.axis('off')
        plt.show()

    def nearest_neighbor_path(self):
        '''returns the optimized path between the waypoints'''
        
        waypoints = np.array(self.waypoints)
        
        n = len(waypoints)
        # Calculate pairwise distances using broadcasting
        diff = waypoints[:, np.newaxis, :] - waypoints[np.newaxis, :, :]
        dist_matrix = np.sqrt(np.sum(diff * diff, axis=2))
        
        unvisited = np.array(list(range(1, n))) # Start from first waypoint
        route = np.array([0])
        current = 0
        
        while len(unvisited) > 0:
            distances = dist_matrix[current, unvisited]
            min_idx = np.argmin(distances)
            next_point = unvisited[min_idx]
            route = np.append(route, next_point)
            unvisited = np.delete(unvisited, min_idx)
            current = next_point
            
        self.optimized_path = waypoints[route]
        print('Optimized path: ', self.optimized_path)
        self.plot_path()
        return self.optimized_path
    
    
    
    def plot_path(self):
        plt.figure(figsize=(10, 8))
        plt.imshow(cv.cvtColor(self.image, cv.COLOR_BGR2RGB), alpha=0.5)
        
        path_x, path_y = zip(*self.optimized_path)
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Optimized Path')
        plt.scatter(path_x, path_y, c='blue', s=50, marker='o', label='Waypoints')
        
        # Highlight first and last points
        plt.scatter(path_x[0], path_y[0], c='green', s=100, marker='*', label='Start')
        plt.scatter(path_x[-1], path_y[-1], c='red', s=100, marker='*', label='End')
        
        plt.title('Optimized Path')
        plt.legend()
        plt.axis('off')
        plt.show()

    def pixels_to_xy(self):
        pixel2xy = self.canvas_width / self.image_width
        xy_waypoints = []
        for point in self.waypoints:
            xy_point = point * pixel2xy
            xy_waypoints.append(xy_point)
        self.waypoints = np.array(xy_waypoints)
        return self.waypoints


    # Create a figure and axis
    def plot(self):
        fig, ax = plt.subplots(figsize=(10, 8))

        # Plot the original image
        ax.imshow(cv.cvtColor(self.image, cv.COLOR_BGR2RGB), alpha=0.5)

        # Plot contours
        # contour = ax.contour(edges, levels=10, colors='blue', alpha=0.7)
        # ax.clabel(contour, inline=True, fontsize=8)

        # Plot waypoints
        x, y = zip(*self.waypoints)
        ax.scatter(x, y, c='red', s=20, marker='o', label='Waypoints')

        # Customize the plot
        ax.set_title('Contours and Waypoints Visualization')
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.legend()

        # plt.colorbar(contour, label='Edge Intensity')
        # plt.grid(True)
        plt.show()
        return ax
    
def main():
    # img = Image('images/square.jpg')
    # img.get_waypoints()
    # img.pixels_to_xy()
    # print(img.waypoints)
    # plot = img.plot()

    img = Image('images/square.jpg')
    img.grid_planning()
    img.nearest_neighbor_path()
    
if __name__ == "__main__":
    main()