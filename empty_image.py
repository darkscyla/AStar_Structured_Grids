import cv2
import numpy as np

from containers.grid import Grid


if __name__ == "__main__":
    # Properties to change the outputs
    draw_thickness = 1
    line_color = (0, 0, 255)
    background_color = (255, 255, 255)
    dimensions = (500, 700, 3)

    # Setup the domain and obstacles
    my_grid = Grid(resolution=(dimensions[0], dimensions[1]))

    # Find the shortest path
    print("Computing AStar shortest path...")
    
    path = my_grid.astar(
        start=(0, 0), end=(dimensions[0] - 1, dimensions[1] - 1), obstacles=()
    )
    
    print("Done computing")

    # Create the white image
    image = np.zeros(dimensions, dtype=np.uint8)
    image.fill(255)

    print("Drawing image...")

    # Draw the AStar path
    for pt in path:
        # OpenCV uses reverse numbering
        cv2.circle(image, (pt[1], pt[0]), 0, line_color, draw_thickness)

    cv2.imwrite("results/empty.png", image)

    print("Done!")
