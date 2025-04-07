import heapq
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


def convert_image_to_matrix(image_path, maze_length=410, maze_width=470):
    image_path = "maze5.jpeg"
    img = cv.imread(image_path)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    blur= cv.GaussianBlur(gray,(3,3),0)
    resized_image = cv.resize(blur, (maze_width, maze_length))
    edged = cv.Canny(image=resized_image, threshold1=45, threshold2=200, apertureSize=3)
    blur= cv.GaussianBlur(edged,(5,5),0)
    blur[blur > 50] = 255
    edged = cv.Canny(image=blur, threshold1=45, threshold2=200, apertureSize=3)
    edged[edged > 0] = 1
    return np.array(edged)



class MazeSolver:
    def __init__(self, maze, start, goal):
        self.maze = maze
        self.start = start
        self.goal = goal
        self.rows = len(maze)
        self.cols = len(maze[0])
        
        self.direction_mapping = {
            "right": (0, 1),
            "down": (1, 0),
            "left": (0, -1),
            "up": (-1, 0)
        }
        self.DIRECTIONS = ["right", "down", "left", "up"]
    
    def h_computation(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar_search(self):
        open_list = []
        heapq.heappush(open_list, (0, self.start))

        g_score = {self.start: 0}
        came_from = {}
        
        decision_points = []  # node with two or more oprion
        
        while open_list:
            _, current = heapq.heappop(open_list)

            if current == self.goal:
                return self.output_listing(came_from, current, decision_points)

            valid_neighbors = 0 # for decision point check
            neighbors = []
            
            for direction in self.DIRECTIONS:
                movement = self.direction_mapping[direction]
                neighbor = (current[0] + movement[0], current[1] + movement[1])

                # Check if the neighbor is valid and keeps a one-cell distance from walls
                if self.is_valid(neighbor): #here we check the validity of neghbor and one-cell distance from wall
                    valid_neighbors += 1
                    neighbors.append((neighbor, direction))

            if valid_neighbors > 1: #if valid neghbors is more than one, make it desision point
                decision_points.append(current)
            
            for neighbor, direction in neighbors:
                g_score2 = g_score[current] + 1

                if neighbor not in g_score or g_score2 < g_score[neighbor]:
                    g_score[neighbor] = g_score2
                    f_score = g_score2 + self.h_computation(neighbor, self.goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = (current, direction)

        return None
    
    def is_valid(self, position):
        row, col = position

        if not (0 <= row < self.rows and 0 <= col < self.cols): #inside matrix
            return False
        
        if self.maze[row][col] != 0: #check it it zerp
            return False
        
        for r_offset in [-22, 0, 22]: # Check surrounding cells to be sure they are a one-cell distance from walls
            for c_offset in [-22, 0, 22]:
                check_row, check_col = row + r_offset, col + c_offset
                if 0 <= check_row < self.rows and 0 <= check_col < self.cols:
                    if self.maze[check_row][check_col] == 1:
                        return False
        
        return True

    def output_listing(self, came_from, current, decision_points):

        path = [] 
        path_with_flags = []  # which has desicion point
        directions = []  # movement
        while current in came_from:
            prev, direction = came_from[current]
            path.append(current)  # make normal path
            path_with_flags.append((current, current in decision_points, direction))  # path with flag and direction
            directions.append(direction)  # direction list
            current = prev
        path.append(self.start) 
        path_with_flags.append((self.start, self.start in decision_points, None))  # star position doesnt have flag
        path.reverse()        # Rrevesing path tree
        path_with_flags.reverse()  
        directions.reverse() 
        return path, directions, path_with_flags
    
def summarize_path(path): # summerizing path with same movement
        idx = 0 
        move_cnt = 0
        current = {"direction":path[idx], "cm":1}
        summarized = [current]
        for idx in range(1, len(path)):
            if path[idx] == current["direction"]:
                summarized[move_cnt]["cm"] += 1
            else:
                move_cnt += 1
                current = {"direction":path[idx], "cm":1}
                summarized.append(current)
        # summarized = [s for s in summarized if s["cm"] > 5] # remove movement less than 3 cm
        return summarized    



maze = convert_image_to_matrix('/Users/soheil/Desktop/Robot____Project/maze1.jpeg')
plt.imshow(maze)
# print(maze)

start = (50, 400)  # Starting position
goal = (400, 250)   # Goal position
print(len(maze) - 1)
print(len(maze[0]) - 1)

# Create a MazeSolver object
solver = MazeSolver(maze, start, goal)

# Perform A* search and get the path, directions, and path with flags
result = solver.astar_search()
if result:
    path, directions, path_with_flags = result
    # print("Normal path:", path)
    # print("List of movements:", directions)
    # print("Path with decision point flags and directions:", path_with_flags)
else:
    print("No path found")

x1=[p[0] for p in path]
y1=[p[1] for p in path]
plt.scatter(y1,x1,s=10,axes=plt.gca(),color="r")
# plt.savefig("mazemtrx.png")



summarized_path = summarize_path(directions)


initial_x = 400
initial_y = 50
x = [initial_x]
y = [initial_y]

for s in summarized_path:
    if s["direction"] == "left":
        x.append(x[-1] - s["cm"])
        y.append(y[-1])
    if s["direction"] == "right":
        x.append(x[-1] + s["cm"])
        y.append(y[-1])
    if s["direction"] == "down":
        x.append(x[-1])
        y.append(y[-1] + s["cm"])
    if s["direction"] == "up":
        x.append(x[-1])
        y.append(y[-1]) - s["cm"]

print("number of movement: ", len(summarized_path))
print("\nSummerized Path: ",*summarized_path, sep='\n')
print("\nX,Y movment: ", *list(zip(x,y)), sep='\n')
# print("x-movement:", *x, sep='\n')
# print("y-movment:", y)

plt.plot(x,y,color='white', linestyle='dashed')
# plt.scatter(y,x,s=5,axes=plt.gca(),color="w")
plt.savefig("mazemtrx-movment5.png")  