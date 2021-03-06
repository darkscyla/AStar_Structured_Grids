# --- Internal Imports ---
from collections import namedtuple
from containers.grid import Grid

# --- Standard Imports ---
import cv2
import os
import numpy as np


MazeStruct = namedtuple("MazeStruct", "file_name start end")


if __name__ == "__main__":
    # Properties to change the outputs
    draw_thickness = 1
    line_color = (0, 255, 0)
    visited_color = (128, 128, 0)

    obstacle_threshold = 128

    resources_dir = "resources/"
    results_dir = "results/"

    maze_to_solve = (
        MazeStruct("maze_circular.png", (5, 170), (164, 164)),
        MazeStruct("maze_delta.png", (0, 164), (281, 164)),
        MazeStruct("maze_hex.png", (2, 175), (370, 193)),
        MazeStruct("maze_standard.png", (0, 153), (321, 170)),
        MazeStruct("maze_large.png", (1, 1), (511, 511)),
        MazeStruct("maze_xl.png", (3, 3), (997, 997)),
        MazeStruct("maze_xxl.png", (740, 510), (417, 510)),
        MazeStruct("maze_xxl_orig.png", (740, 510), (417, 510)),
        MazeStruct("maze_xxxl.png", (0, 4), (1796, 1801)),
    )

    for maze in maze_to_solve:
        img = cv2.imread(os.path.join(resources_dir, maze.file_name))

        # Grayscale image
        gs_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ny, nx = gs_img.shape

        # Setup the domain
        my_grid = Grid(resolution=(nx, ny))

        # Get the points where value is lower than the threshold. Numpy indexes oppositely
        obstacles_tuple = tuple(
            map(tuple, np.transpose(np.where(gs_img < obstacle_threshold)))
        )

        # Find the shortest path
        print("{}: Computing AStar shortest path...".format(maze.file_name))

        path = my_grid.astar(start=maze.start, end=maze.end, obstacles=obstacles_tuple)

        print("{}: Done computing".format(maze.file_name))

        print("{}: Drawing and saving image...".format(maze.file_name))

        # Draw the visited nodes
        for pt in (
            {*my_grid.sorted_nodes_map.keys(), *my_grid.closed_nodes.keys()}
            .difference(obstacles_tuple)
            .difference(path)
        ):
            cv2.circle(img, (pt[1], pt[0]), 0, visited_color, 1)

        # Draw the AStar path
        for pt in path:
            # OpenCV draw functions use reverse numbering
            cv2.circle(img, (pt[1], pt[0]), 0, line_color, draw_thickness)

        cv2.imwrite(os.path.join(results_dir, maze.file_name), img)

        print("{}: Done!".format(maze.file_name))
