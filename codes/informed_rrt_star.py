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
from utils import *
import sys


startX = float(input("Enter the x-coordinate for start node : "))
startY = float(input("Enter the y-coordinate for start node : "))
goalX = float(input("Enter the x-coordinate for goal node : "))
goalY = float(input("Enter the y-coordinate for goal node : "))

# take start and goal node as input
start = (startX, startY)
goal = (goalX, goalY)
rrt = InformedRRTStar(start, goal)
if(rrt.IsValid(start[0], start[1])):
    if(rrt.IsValid(goal[0], goal[1])):
        if(rrt.IsObstacle(start[0],start[1]) == False):
            if(rrt.IsObstacle(goal[0], goal[1]) == False):
                (explored_states, backtrack_states) = rrt.search()
                
                # animate the path
                rrt.animate(explored_states, backtrack_states)
                
                print(len(explored_states))
                print(len(backtrack_states))
            else:
                print("The entered goal node is an obstacle ")
                print("Please check README.md file for running informed_rrt_star.py file.")
        else:
            print("The entered start node is an obstacle ")
            print("Please check README.md file for running informed_rrt_star.py file.")
    else:
        print("The entered goal node outside the map ")
        print("Please check README.md file for running informed_rrt_star.py file.")
else:
    print("The entered start node is outside the map ")
    print("Please check README.md file for running informed_rrt_star.py file.")
