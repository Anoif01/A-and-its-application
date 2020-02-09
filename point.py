# -*- coding: utf-8 -*-
"""
Created on Sun Nov  3 22:16:41 2019

@author: lssya
"""

import sys

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        # define the cost of a point as the maximum size  
        self.cost = sys.maxsize