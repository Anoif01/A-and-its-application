 # -*- coding: utf-8 -*-
"""
Created on Sun Nov  3 22:19:28 2019

@author: lssya
"""
import numpy as np

import point

# 实现一个描述地图结构的类
class MyMap:
    def __init__(self,size,nbObstacle):
        self.size = size
        self.obstacle = nbObstacle
        self.GenerateObstacle()
        
    def GenerateObstacle(self):
        self.obstacle_point = []
        self.obstacle_point.append(point.Point(self.size//2, self.size//2))
        self.obstacle_point.append(point.Point(self.size//2, self.size//2-1))
        
        # Generate an obstacle in the middle
        for i in range(self.size//2-4, self.size//2):
            self.obstacle_point.append(point.Point(i, self.size-i))
            self.obstacle_point.append(point.Point(i, self.size-i-1))
            self.obstacle_point.append(point.Point(self.size-i, i))
            self.obstacle_point.append(point.Point(self.size-i, i-1))
        
        # Generate the others obstacles by random
        for i in range(self.obstacle-1):
            x = np.random.randint(self.size*0.4, self.size*0.6)
            y = np.random.randint(self.size*0.4, self.size*0.6)
            self.obstacle_point.append(point.Point(x, y))
            
            # Obstacles in random directions(horizon or vertical)
            if (np.random.rand() > 0.5): # vertical
                for l in range(self.size//4):
                    self.obstacle_point.append(point.Point(x, y+l))
                    pass
            else:
                for l in range(self.size//4): # horizon
                    self.obstacle_point.append(point.Point(x+l, y))
                    pass

    def getObstacles(self):
        return self.obstacle_point
                
                
                
                
                
                
                
                
                
                
                
                
                