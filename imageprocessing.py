import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

class Image:
    def __init__(self, path_to_image):
        # Read the image and detect edges
        self.image = cv.imread(path_to_image)
        self.gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        self.waypoints = None

    def getWaypoints(self):
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
    img = Image('images/smiley1.jpg')
    img.getWaypoints()
    plot = img.plot()
    pass
    
if __name__ == "__main__":
    main()
