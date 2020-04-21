# header files required 
import numpy as np
import random


# define class
class RRTStar(object):    
    
    # init function
    def __init__(self, start, goal):
        """
        Inputs:
        
        start: this is the start coordinate of the robot. It is a tuple of form (x, y, theta).
        goal: this is the goal coordinate of the robot. It is a tuple of form (x, y).
        """
        
        # start variable - tuple of of form (x, y)
        self.start = start
        
        # goal variable - tuple of form (x, y)
        self.goal = goal
        
        # the map size along x and y dimensions in cms (map dimension are from -500 to 500 for both x and y direction)
        # map size is 1000 cm x 1000 cm
        self.xLength = 500
        self.yLength = 500
        
        # clearance variable - distance of the robot from the obstacle
        self.clearance = 25.0
        
        # radius variable - the radius of the robot (taken from turtlebot datasheet)
        self.radius = 20.0
        
        # wheelDistance - the distance between wheels (taken from turtlebot datasheet)
        self.wheelDistance = 34.0
        
        # wheelRadius - the radius of the wheels (taken from turtlebot datasheet)
        self.wheelRadius = 3.8
        
        # distance - hashmap that stores net distance of the node from the start and the goal
        self.distance = {}
        
        # path - hashmap used for backtracking from the goal node to the start node
        self.path = {}
        
        # costToCome - hashmap to store the distance of the nodes from the start node
        self.costToCome = {}
        
        # costToGo - hashmap to store the distance of the nodes from the goal node
        self.costToGo = {}
        
        # hashMap - custom variable to hold visited nodes
        self.hashMap = {}
        
        # goalThreshold - threshold from goal node
        self.goalThreshold = 15
        
    # move is valid or not
    def IsValid(self, currX, currY):
        """
        Inputs:
        
        currX - the current x-position of the robot.
        currY - the current y-posiiton of the robot.
        
        Outputs:
        
        True / False depending on whether the nodes lies within the map or not.
        """
        
        nodeInMap = (currX >= (-self.xLength + self.radius + self.clearance) and currX <= (self.xLength - self.radius - self.clearance) and currY >= (-self.yLength + self.radius + self.clearance) and currY <= (self.yLength - self.radius - self.clearance))
        return nodeInMap
    
    # checks for an obstacle in the given map
    def IsObstacle(self, row, col):
        """
        Inputs:
        
        row - the current x-position of the robot.
        col - the current y-posiiton of the robot.
        
        Outputs:
        
        True / False depending on whether the nodes lies within obstacle or not.
        """
        
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circles(obstacles) in the given map
        dist1 = ((row - 200.0) * (row - 200.0) + (col - 300.0) * (col - 300.0)) - ((100 + sum_of_c_and_r) * (100 + sum_of_c_and_r))
        dist2 = ((row - 200.0) * (row - 200.0) + (col + 300.0) * (col + 300.0)) - ((100 + sum_of_c_and_r) * (100 + sum_of_c_and_r))
        dist3 = ((row + 200.0) * (row + 200.0) + (col + 300.0) * (col + 300.0)) - ((100 + sum_of_c_and_r) * (100 + sum_of_c_and_r))
        dist4 = ((row) * (row) + (col) * (col)) - ((100 + sum_of_c_and_r) * (100 + sum_of_c_and_r))
        
        # check first square(obstacle) in the given map
        (x1, y1) = (325 - sqrt_of_c_and_r, -75 - sqrt_of_c_and_r)
        (x2, y2) = (325 - sqrt_of_c_and_r, 75 + sqrt_of_c_and_r)
        (x3, y3) = (475 + sqrt_of_c_and_r, 75 + sqrt_of_c_and_r)
        (x4, y4) = (475 + sqrt_of_c_and_r, -75 - sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist5 = 1
        dist6 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist5 = 0
            dist6 = 0
        
        # check second square(obstacle) in the given map
        (x1, y1) = (-325 + sqrt_of_c_and_r, -75 - sqrt_of_c_and_r)
        (x2, y2) = (-325 + sqrt_of_c_and_r, 75 + sqrt_of_c_and_r)
        (x3, y3) = (-475 - sqrt_of_c_and_r, 75 + sqrt_of_c_and_r)
        (x4, y4) = (-475 - sqrt_of_c_and_r, -75 - sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist7 = 1
        dist8 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist7 = 0
            dist8 = 0

        # check third square(obstacle) in the given map
        (x1, y1) = (-125 + sqrt_of_c_and_r, 375 + sqrt_of_c_and_r)
        (x2, y2) = (-125 + sqrt_of_c_and_r, 225 - sqrt_of_c_and_r)
        (x3, y3) = (-275 - sqrt_of_c_and_r, 225 - sqrt_of_c_and_r)
        (x4, y4) = (-275 - sqrt_of_c_and_r, 375 + sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist9 = 0
            dist10 = 0
        
        # return true if obstacle, otherwise false
        if(dist1 <= 0 or dist2 <= 0 or dist3 <= 0 or dist4 <= 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
        return False
    
    # animate path and show the nodes on map
    def animate(self, exploredStates, backtrackStates):
        """
        Inputs:
        
        exploredStates: list of explored states when going from start to  goal node.
        backtrackStates: list of states to go from start to goal node.
        """
        
        startX = []
        startY = []
        endX = []
        endY = []
        explored_startX = []
        explored_startY = []
        explored_endX = []
        explored_endY = []
        fig, ax = plt.subplots()
        plt.xlabel("x-coordinate(in m)")
        plt.ylabel("y-coordinate(in m)")
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(-self.xLength / 100.0, self.xLength / 100.0)
        plt.ylim(-self.yLength / 100.0, self.yLength / 100.0)
        count = 0
        
        # obstacle space
        obstacleX = []
        obstacleY = []
        size = []
        for index1 in range(-self.xLength, self.xLength):
            for index2 in range(-self.yLength, self.yLength):
                if(self.IsObstacle(index1, index2)):
                    obstacleX.append(index1 / 100.0)
                    obstacleY.append(index2 / 100.0)     
                    size.append(15)      
        obstacleX = np.array(obstacleX)
        obstacleY = np.array(obstacleY)
        plt.scatter(obstacleX, obstacleY, color='b', s=size)

        # explore node space
        for index in range(1, len(exploredStates)):
            parentNode = self.path[exploredStates[index]][0]
            explored_startX.append(parentNode[0] / 100.0)
            explored_startY.append(parentNode[1] / 100.0)
            explored_endX.append((exploredStates[index][0] - parentNode[0]) / 100.0)
            explored_endY.append((exploredStates[index][1] - parentNode[1]) / 100.0)    
            #if(count % 2000 == 0):
            #    plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 1, color = 'g', label = 'Explored region')
            #    plt.savefig("output/phase3/sample" + str(count) + ".png", dpi=1700)
            count = count + 1

        # backtrack space
        if(len(backtrackStates) > 0):
            for index in range(1, len(backtrackStates)):
                startX.append(backtrackStates[index-1][0] / 100.0)
                startY.append(backtrackStates[index-1][1] / 100.0)
                endX.append((backtrackStates[index][0] - backtrackStates[index-1][0]) / 100.0)
                endY.append((backtrackStates[index][1] - backtrackStates[index-1][1]) / 100.0)    
                #if(count % 5 == 0):
                #    plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 1, color = 'r', label = 'Backtrack path')
                #    plt.savefig("output/phase3/sample" + str(count) + ".png", dpi=1700)
                count = count + 1

        plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 1, color = 'g', label = 'Explored region')
        if(len(backtrackStates) > 0):
            plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 1, color = 'r', label = 'Backtrack path')
        #plt.savefig("output.png", dpi=1700)
        plt.legend()
        plt.show()
        plt.close()
    
    # eucledian heuristic
    def euc_heuristic(self, point1, point2):
        """
        Inputs:
        
        point1 - the first position of the robot, tuple (x, y).
        point2 - the second posiiton of the robot, tuple (x, y).
        
        Output:
        
        Returns the eucledian distance between point1 and point2
        """
        
        return (np.sqrt(((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2)))
