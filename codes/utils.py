"""
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal Shantam Bajpai
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
"""


# header files
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from heapq import heappush, heappop


# class for RRT
class RRT(object):
    # init function
    def __init__(self, start, goal, clearance, radius):
        self.start = start
        self.goal = goal
        self.xLength = 300.0
        self.yLength = 200.0
        self.clearance = clearance
        self.radius = radius
        self.numIterations = 10
        self.path = {}
        self.graph = {}
        self.vertices = []
    
    # move is valid or not
    def IsValid(self, currX, currY):
        return (currX >= (self.radius + self.clearance) and currX <= (self.xLength - self.radius - self.clearance) and currY >= (self.radius + self.clearance) and currY <= (self.yLength - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, currX, currY):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circle
        dist1 = ((currY - 150) * (currY - 150) + (currX - 225) * (currX - 225)) - ((25 + sum_of_c_and_r) * (25 + sum_of_c_and_r))
        
        # check eclipse
        dist2 = ((((currY - 100) * (currY - 100)) / ((20 + sum_of_c_and_r) * (20 + sum_of_c_and_r))) + (((currX - 150) * (currX - 150)) / ((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r)))) - 1
        
        # check triangles
        (x1, y1) = (120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r))
        (x2, y2) = (150 - sqrt_of_c_and_r, 50)
        (x3, y3) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        first = ((currX - y1) * (x2 - x1)) - ((y2 - y1) * (currY - x1))
        second = ((currX - y2) * (x3 - x2)) - ((y3 - y2) * (currY - x2))
        third = ((currX - y3) * (x1 - x3)) - ((y1 - y3) * (currY - x3))
        dist3 = 1
        if(first <= 0 and second <= 0 and third <= 0):
            dist3 = 0
            
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        (x3, y3) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.714))
        first = ((currX - y1) * (x2 - x1)) - ((y2 - y1) * (currY - x1))
        second = ((currX - y2) * (x3 - x2)) - ((y3 - y2) * (currY - x2))
        third = ((currX - y3) * (x1 - x3)) - ((y1 - y3) * (currY - x3))
        dist4 = 1
        if(first >= 0 and second >= 0 and third >= 0):
            dist4 = 0
        
        # check rhombus
        (x1, y1) = (10 - sqrt_of_c_and_r, 225)
        (x2, y2) = (25, 200 - sqrt_of_c_and_r)
        (x3, y3) = (40 + sqrt_of_c_and_r, 225)
        (x4, y4) = (25, 250 + sqrt_of_c_and_r)
        first = ((currX - y1) * (x2 - x1)) - ((y2 - y1) * (currY - x1))
        second = ((currX - y2) * (x3 - x2)) - ((y3 - y2) * (currY - x2))
        third = ((currX - y3) * (x4 - x3)) - ((y4 - y3) * (currY - x3))
        fourth = ((currX - y4) * (x1 - x4)) - ((y1 - y4) * (currY - x4))
        dist5 = 1
        dist6 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist5 = 0
            dist6 = 0
        
        # check square
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (120 - sqrt_of_c_and_r, 75)
        (x3, y3) = (150, 100 + sqrt_of_c_and_r)
        (x4, y4) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.714))
        first = ((currX - y1) * (x2 - x1)) - ((y2 - y1) * (currY - x1))
        second = ((currX - y2) * (x3 - x2)) - ((y3 - y2) * (currY - x2))
        third = ((currX - y3) * (x4 - x3)) - ((y4 - y3) * (currY - x3))
        fourth = ((currX - y4) * (x1 - x4)) - ((y1 - y4) * (currY - x4))
        dist7 = 1
        dist8 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist7 = 0
            dist8 = 0
        
        # check rod
        first = ((currX - 95) * (8.66 + sqrt_of_c_and_r)) - ((5 + sqrt_of_c_and_r) * (currY - 30 + sqrt_of_c_and_r))
        second = ((currX - 95) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (currY - 30 + sqrt_of_c_and_r))
        third = ((currX - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (currY - 67.5))
        fourth = ((currX - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (currY - 76.15 - sqrt_of_c_and_r))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist9 = 0
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
        return False

    # euc heuristic (becomes weighted a-star when weight made greater than 1.0)
    def EucHeuristic(self, currX, currY, weight = 1.0):
        return weight * (np.sqrt(((self.goal[0] - currX) ** 2) + ((self.goal[1] - currY) ** 2)))
    
    # euc distance between points
    def EucDistance(self, x1, y1, x2, y2):
        return (np.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2)))
    
    # plot map
    def PlotMap(self):
        # plot the obstacle map
        plt.xlim(0, self.xLength)
        plt.ylim(0, self.yLength)
        plt.xlabel("x-coordinate")
        plt.ylabel("y-coordinate")
        obstacleX = []
        obstacleY = []
        size = []
        for index1 in range(0, int(self.xLength)):
            for index2 in range(0, int(self.yLength)):
                if(self.IsObstacle(index1, index2)):
                    obstacleX.append(index1)
                    obstacleY.append(index2)     
                    size.append(15)      
        obstacleX = np.array(obstacleX)
        obstacleY = np.array(obstacleY)
        plt.scatter(obstacleX, obstacleY, color='b', s=size)
        
        # plot vertices
        vertexX = []
        vertexY = []
        size = []
        for vertex in self.vertices:
            vertexX.append(vertex[0])
            vertexY.append(vertex[1])
            size.append(15)
        vertexX = np.array(vertexX)
        vertexY = np.array(vertexY)
        plt.scatter(vertexX, vertexY, color='g', s=size)
        
        plt.show()
    
    # random position generator
    def RandomPosition(self):
        randX = round(random.random() * self.xLength, 2)
        randY = round(random.random() * self.yLength, 2)
        return (randX, randY)
    
    # nearest neighbour in the graph
    def NearestNeighbour(self, currX, currY):
        # set vertex to -1
        minDistance = float('inf')
        nearestVertex = -1
        
        # loop through vertices of graph
        for vertex in self.vertices:
            distance = self.EucDistance(vertex[0], vertex[1], currX, currY)
            if(distance < minDistance):
                minDistance = distance
                nearestVertex = vertex
        
        # return nearest vertex
        return nearestVertex
    
    # rrt algo
    def Search(self):
        # initialize graph
        currIteration = 0
        self.vertices.append(self.start)
        
        # run algo
        while(currIteration < self.numIterations):
            # generate random x and y
            (currX, currY) = self.RandomPosition()
            
            # check for obstacle and boundary
            if((self.IsValid(currX, currY) == False) and (self.IsObstacle(currX, currY) == True)):
                continue
            
            # find nearest vertex to the random point in graph
            (nearX, nearY) = self.NearestNeighbour(currX, currY)
            
            # add vertex to graph if not an obstacle
            newX = round((nearX + currX) / 2.0, 2)
            newY = round((nearY + currY) / 2.0, 2)
            if((self.IsValid(newX, newY) == True) and (self.IsObstacle(newX, newY) == False)):
                self.vertices.append((newX, newY))
            
            # next iteration
            currIteration = currIteration + 1
        
        print(self.vertices)
