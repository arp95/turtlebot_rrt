#!/usr/bin/python

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
import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from heapq import heappush, heappop
import rospy
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import pow, atan2, sqrt
import time


# move robot function
def move_robot(pub_vel, dvx, dvy, dw):
    """
    Inputs:
    
    pub_vel: the ROS publisher which publishes the information to topic 'cmd_vel_mux/input/navi'
    dvx: velocity along x-direction
    dvy: velocity along y-direction
    dw: angular velocity
    """
    
    r = rospy.Rate(100)
    vel_value = Twist()
    velocity = np.sqrt(dvx * dvx + dvy * dvy) / 100.0
    endTime = rospy.Time.now() + rospy.Duration(1)
    while rospy.Time.now() < endTime:
        vel_value.linear.x = velocity
        vel_value.angular.z = dw
        pub_vel.publish(vel_value)
        r.sleep()


# class for AStar
class AStar(object):
    
    # init function
    def __init__(self, start, goal, wheelRPM, clearance):
        """
        Inputs:
        
        start: this is the start coordinate of the robot. It is a tuple of form (x, y, theta).
        goal: this is the goal coordinate of the robot. It is a tuple of form (x, y).
        wheelRPM: this is the values of RPM of the wheels. It is of form (leftRPM, rightRPM).
        clearance: this is the clearance that the robot needs to have with the obstacles.
        """
        
        # start variable - tuple of of form (x, y, theta)
        self.start = start
        
        # goal variable - tuple of form (x, y)
        self.goal = goal
        
        # the map size along x and y dimensions in cms (map dimension are from -500 to 500 for both x and y direction)
        # map size is 1000 cm x 1000 cm
        self.xLength = 500
        self.yLength = 500
        
        # wheelRPM variable - tuple of form (leftRPM, rightRPM)
        self.wheelRPM = wheelRPM
        
        # clearance variable - distance of the robot from the obstacle
        self.clearance = min(clearance + 10, 25)
        
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
        
        # frequency - the value of frequency for curved path
        self.frequency = 100
    

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
    
    
    # return updated position by taking into account non-holonomic constraint of robot
    def GetNewPositionOfRobot(self, currentNode, leftRPM, rightRPM):
        """
        Inputs:
        
        currentNode: the current node, tupe of type (x, y, theta)
        leftRPM: the value of RPM of left wheel
        rightRPM: the value of RPM of right wheel
        
        Outputs:
        
        x: Updated x-coordinate
        y: Updated y-coordinate
        theta: Updated angle value
        cost: Net cost to move from (currentNode, (x, y))
        dvx: linear velocity along x-direction to move from (currentNode, (x, y))
        dvy: linear velocity along y-direction to move from (currentNode, (x, y))
        w: angle threshold during each time step dt
        flag: determines whether it is possible to move from (currentNode, (x, y))
        """
        
        # calculate angular valocity of left wheel and right wheel and initialise vairables
        leftAngularVelocity = leftRPM * 2 * np.pi / 60.0
        rightAngularVelocity = rightRPM * 2 * np.pi / 60.0
        x = currentNode[0]
        y = currentNode[1]
        theta = currentNode[2]
        dx = 0
        dy = 0
        dtheta = 0
        cost = 0
        flag = True
        w = (self.wheelRadius / self.wheelDistance) * (rightAngularVelocity - leftAngularVelocity)
        
        # get updated node after moving 100 steps
        for index in range(0, self.frequency):
            dvx = self.wheelRadius * 0.5 * (leftAngularVelocity + rightAngularVelocity) * math.cos(theta)
            dvy = self.wheelRadius * 0.5 * (leftAngularVelocity + rightAngularVelocity) * math.sin(theta)
            dx = dvx / self.frequency
            dy = dvy / self.frequency
            dtheta = w / self.frequency
            x = x + dx
            y = y + dy
            theta = theta + dtheta
            cost = cost + np.sqrt(dx ** 2 + dy ** 2)
            
            if(self.IsValid(x, y) == False or self.IsObstacle(x, y)):
                flag = False
                break
                
        # pruning
        if(flag != False and self.hashMap.get(int(int(x * 100) + int(y * 10))) != None):
            flag = False
            
        # return updated location
        return (x, y, theta, cost, dvx, dvy, w, flag)

    
    # action move
    def ActionMoveRobot(self, currentNode, leftRPM, rightRPM):
        """
        currentNode: the current node, tupe of type (x, y, theta)
        leftRPM: the value of RPM of left wheel
        rightRPM: the value of RPM of right wheel
        
        Outputs:
        
        flag: True/ False, if possible to move to the updated node
        newX: Updated x-coordinate
        newY: Updated y-coordinate
        newTheta: Updated angle value
        dvx: linear velocity along x-direction to move from (currentNode, (newX, newY))
        dvy: linear velocity along y-direction to move from (currentNode, (newX, newY))
        dw: angle threshold during each time step dt
        cost: Net cost to move from (currentNode, (x, y))
        """
        
        # update position
        (newX, newY, newTheta, cost, dvx, dvy, dw, flag) = self.GetNewPositionOfRobot(currentNode, leftRPM, rightRPM)
        self.hashMap[int(int(newX * 100) + int(newY * 10))] = 1
        
        # check obstacle
        if(flag == True and self.IsValid(newX, newY) and self.IsObstacle(newX, newY) == False):
            return (True, newX, newY, newTheta, dvx, dvy, dw, cost)
        return (False, newX, newY, newTheta, dvx, dvy, dw, cost)

    
    # update action
    def UpdateAction(self, currentNode, weight, newX, newY, newTheta, action, cost):
        """
        Inputs:
        
        currentNode: the current node, tupe of type (x, y, theta)
        weight: cost to move from (currentNode,(newX, newY))
        newX: Updated x-coordinate
        newY: Updated y-coordinate
        newTheta: Updated angle value
        action: containing dvx and dvy values for further storing
        cost: the additional cost to move from (currentNode, (newX, newY))
        
        Outputs:
        
        True / False depedning on whether an optimal node found
        """
        
        newCostToCome = self.costToCome[currentNode] + weight
        newCostToGo = self.euc_heuristic(newX, newY)
        newDistance = newCostToCome + newCostToGo + cost

        if(self.distance.get((newX, newY, newTheta)) == None):
            self.distance[(newX, newY, newTheta)] = float('inf')                    
        if(self.distance[(newX, newY, newTheta)] > newDistance):
            self.distance[(newX, newY, newTheta)] = newDistance
            self.costToCome[(newX, newY, newTheta)] = newCostToCome
            self.costToGo[(newX, newY, newTheta)] = newCostToGo
            self.path[(newX, newY, newTheta)] = (currentNode, action)
            return True
        return False
        
        
    # eucledian heuristic (becomes weighted a-star when weight made greater than 1.0)
    def euc_heuristic(self, currX, currY, weight = 3.0):
        """
        Inputs:
        
        currX - the current x-position of the robot.
        currY - the current y-posiiton of the robot.
        weight - the weight used for A-star algorithm
        
        Output:
        
        Returns the eucledian distance between goal node and the current node(currX, currY)
        """
        
        return weight * (np.sqrt(((self.goal[0] - currX) ** 2) + ((self.goal[1] - currY) ** 2)))
    
    
    # a-star algo
    def search(self):
        """
        Outputs:
        
        exploredStates: the states explored when moving from start node to goal node.
        backtrackStates: the path from start node to goal node.
        actions: list containing the (dvx, dvy) values for each possible node between start and goal node.
        distance: the total distance between start node and goal node.
        """
        
        # mark source node and create a queue
        exploredStates = []
        queue = []
        self.costToCome[self.start] = 0
        self.costToGo[self.start] = self.euc_heuristic(self.start[0], self.start[1])
        self.distance[self.start] = self.costToCome[self.start] + self.costToGo[self.start]
        heappush(queue, (self.distance[self.start], self.costToCome[self.start], self.start))
        backtrackNode = None
        flag = 0
        steps = 0
        
        # run A-star
        while(len(queue) > 0):
            
            # get current node
            _, _, currentNode = heappop(queue) 
            self.hashMap[int(int(currentNode[0] * 100) + int(currentNode[1] * 10))] = 1
            exploredStates.append(currentNode)
            steps = steps + 1
            
            # if goal node then break, using the distance formula
            if(np.square(np.abs(currentNode[0] - self.goal[0])) + np.square(np.abs(currentNode[1] - self.goal[1])) < self.goalThreshold):
                backtrackNode = currentNode
                flag = 1
                break
               
            # break if steps greater than 1000000 (exit when no path exists)
            if(steps > 1000000):
                break

            # traverse the edges
            # action 1
            (moveOnePossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, 0, self.wheelRPM[0])
            if(moveOnePossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[0], newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
            
            # action 2
            (moveTwoPossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], 0)
            if(moveTwoPossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[0], newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 3
            (moveThreePossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], self.wheelRPM[0])
            if(moveThreePossible):
                updateHeap = self.UpdateAction(currentNode, (self.wheelRPM[0] * 1.4142), newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
              
            # action 4
            (moveFourPossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, 0, self.wheelRPM[1])      
            if(moveFourPossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[1], newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 5
            (moveFivePossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], 0)
            if(moveFivePossible):
                updateHeap = self.UpdateAction(currentNode, self.wheelRPM[1], newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

            # action 6
            (moveSixPossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], self.wheelRPM[1])
            if(moveSixPossible):
                updateHeap = self.UpdateAction(currentNode, (self.wheelRPM[1] * 1.4142), newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))
                    
            # action 7
            (moveSevenPossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[0], self.wheelRPM[1])
            if(moveSevenPossible):
                updateHeap = self.UpdateAction(currentNode, max((self.wheelRPM[0] * 1.4142), (self.wheelRPM[1] * 1.4142)), newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

            
            # action 8
            (moveEightPossible, newX, newY, newTheta, dvx, dvy, dw, cost) = self.ActionMoveRobot(currentNode, self.wheelRPM[1], self.wheelRPM[0])
            if(moveEightPossible):
                updateHeap = self.UpdateAction(currentNode, max((self.wheelRPM[0] * 1.4142), (self.wheelRPM[1] * 1.4142)), newX, newY, newTheta, (dvx, dvy, dw), cost)
                if(updateHeap):
                    heappush(queue, (self.distance[(newX, newY, newTheta)], self.costToCome[(newX, newY, newTheta)], (newX, newY, newTheta)))

        # return if no optimal path
        if(flag == 0):
            return (exploredStates, [], float('inf'))
        
        # backtrack path
        backtrackStates = []
        actions = []
        node = backtrackNode
        action = None
        while(node != self.start):
            backtrackStates.append(node)
            if(action != None):
                actions.append(action)
            node, action = self.path[node]
        backtrackStates.append(self.start)
        actions.append(action)
        backtrackStates = list(reversed(backtrackStates))  
        actions = list(reversed(actions))    
        return (exploredStates, backtrackStates, actions, self.distance[backtrackNode])


# make ros publisher
rospy.init_node('turtlebot_astar')
pub_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
time.sleep(2)

# map size is 1000 cm x 1000 cm
print()
print()
startX = float(input("Enter the x-coordinate for start node(in m) : "))
startY = float(input("Enter the y-coordinate for start node(in m) : "))
startOrientation = float(input("Enter the orientation for start node : "))
goalX = float(input("Enter the x-coordinate for goal node(in m) : "))
goalY = float(input("Enter the y-coordinate for goal node(in m) : "))
firstRPM = float(input("Enter the first value of RPM : "))
secondRPM = float(input("Enter the second value of RPM : "))
clearance = float(input("Enter the clearance of the rigid robot(in m) : "))

# take start and goal node as input
start = (startX * 100.0, startY * 100.0, startOrientation)
goal = (goalX * 100.0, goalY * 100.0)
wheelRPM = (firstRPM, secondRPM) # (100, 50)
astar = AStar(start, goal, wheelRPM, clearance * 100)

if(astar.IsValid(start[0], start[1])):
    if(astar.IsValid(goal[0], goal[1])):
        if(astar.IsObstacle(start[0],start[1]) == False):
            if(astar.IsObstacle(goal[0], goal[1]) == False):
                states = astar.search()
                explored_states = states[0]
                backtrack_states = states[1]
                actions = states[2]

                # move robot in ROS from start to goal node
                for index in range(0, len(actions)):
                    dvx, dvy, dw = actions[index]
                    move_robot(pub_vel, dvx, dvy, dw) 

                # print optimal path found or not
                if(len(backtrack_states) == 0):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found.")
            else:
                print("The entered goal node is an obstacle ")
                print("Please check README.md file for running turtlebot_astar.py file.")
        else:
            print("The entered start node is an obstacle ")
            print("Please check README.md file for running turtlebot_astar.py file.")
    else:
        print("The entered goal node outside the map ")
        print("Please check README.md file for running turtlebot_astar.py file.")
else:
    print("The entered start node is outside the map ")
    print("Please check README.md file for running turtlebot_astar.py file.")
