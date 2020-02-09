# -*- coding: utf-8 -*-
"""
Created on Sun Nov  3 22:44:22 2019

@author: lssya
"""
import sys
import time
import numpy as np
from matplotlib.patches import Rectangle
import point
import My_Map



class AStar:
    
#%%  构造函数
    
    def __init__(self, Map_size, obstacle_point, p_start, p_end):
        self.Map_size = Map_size
        self.obstacle_point = obstacle_point
        self.p_start = p_start
        self.p_end = p_end
        self.open_set = []
        self.close_set = []
        
        
#%% 节点到起点的移动代价，对应g(n)
        
        
    def GCost(self, p):
        h_diagonal = min(np.abs(self.p_start.x - p.x -1), np.abs(self.p_start.y - p.y - 1))
        h_stright = np.abs(self.p_start.x - p.x - 1) + np.abs(self.p_start.y - p.y - 1)
        # Distance to start point
        return h_stright + (np.sqrt(2) - 2) * h_diagonal
    
# 节点到终点的启发函数，对应h(n)
    def HCost(self, p):
        h_diagonal = min(np.abs(self.p_end.x - p.x - 1), np.abs(self.p_end.y - p.y - 1))
        h_stright = np.abs(self.p_end.x - p.x - 1) + np.abs(self.p_end.y - p.y - 1)
        # Distance to start point
        return h_stright + (np.sqrt(2) - 2) * h_diagonal
    
# 代价总和，即对应f(n)
    def TotalCost(self, p):
        return self.GCost(p) + self.HCost(p)
    
    
#%% 判断点是否有效，不在地图内部或者障碍物所在点都是无效的
        
    
    def IsValidPoint(self, x, y):
        if x < 0 or y < 0:
            return False
        if x >= self.Map_size[0] or y >= self.Map_size[1]:
            return False
        for p in self.obstacle_point:
            if x==p.x and y==p.y:
                return False
        return True
    
    
#%% 判断点是否在某个集合中,open_set 或 close_set?
        
    
    def IsInPointList(self, p, point_list):
        for pt in point_list:
            if pt.x == p.x and pt.y == p.y:
                return True
        return False
    
    def IsInOpenList(self, p):
        return self.IsInPointList(p, self.open_set)

    def IsInCloseList(self, p):
        return self.IsInPointList(p, self.close_set)
    
    
#%% 判断点是否是起点，终点
        
    
    def IsStartPoint(self, p):
        return p.x == self.p_start.x and p.y == self.p_start.y
    
    def IsEndPoint(self, p):
        return p.x == self.p_end.x and p.y == self.p_end.y
    
    
#%% 将当前状态保存到图片中，图片以当前时间命名
        
    
    def SaveImage(self, plt):
        exec_time = int(round(time.time() * 1000))
        filename = './' + str(exec_time) + '.png'
        plt.savefig(filename)
        
    
#%% 针对每一个节点进行处理：如果是没有处理过的节点，则计算优先级设置父节点，并且添加到open_set中。
        
        
    def ProcessPoint(self, x, y, parent):
        if not self.IsValidPoint(x, y):
            return # Do nothing for invalid point
        p = point.Point(x, y)
        if self.IsInCloseList(p):
            return # Do nothing for visited point
        print('Process Point [', p.x, ',', p.y, ']', ', cost: ', p.cost)
        if not self.IsInOpenList(p):
            p.parent = parent
            p.cost = self.TotalCost(p)
            self.open_set.append(p)
            
            
#%% 从open_set中找到优先级最高的节点，返回其索引
            
            
    def SelectPointInOpenList(self):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in self.open_set:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index
    
    
#%% 从终点往回沿着parent构造结果路径。然后从起点开始绘制结果，结果使用绿色方块，最后保存一个图片。
        
    
    def BuildPath(self, p, ax, plt, start_time):
        path = []
        while True:
            path.insert(0, p) # Insert first
            if self.IsStartPoint(p):
                break
            else:
                p = p.parent   
        for p in path:
            rec = Rectangle((p.x, p.y), 1, 1, color='g')
            ax.add_patch(rec)
            plt.draw()
            
        self.SaveImage(plt)
        end_time = time.time()
        print('===== Algorithm finish in %.4f seconds' %(end_time-start_time))
        
        
#%% 算法实现
        
        
    def RunAndSaveImage(self, ax, plt, ):
        start_time = time.time()
        
        # 初始化起点以及起点的优先值，并加入open_set
        self.p_start.cost = 0
        self.open_set.append(self.p_start)
    
        while True:
            index = self.SelectPointInOpenList() # 返回open_set中优先级最高的索引
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index] # 返回open_set中优先级最高的点
            rec = Rectangle((p.x, p.y), 1, 1, color='c') # 绘制p点
            ax.add_patch(rec)
    
            if self.IsEndPoint(p): # 判断是否为终点，如果是的话，绘制找到的最短路径
                return self.BuildPath(p, ax, plt, start_time)
    
            del self.open_set[index] #将该p点从open_set中删除，加入close_set中
            self.close_set.append(p)
    
            # Process all neighbors 处理以p为父节点的所有相邻点（8个）
            x = p.x
            y = p.y
            self.ProcessPoint(x-1, y+1, p)
            self.ProcessPoint(x-1, y, p)
            self.ProcessPoint(x-1, y-1, p)
            self.ProcessPoint(x, y-1, p)
            self.ProcessPoint(x+1, y-1, p)
            self.ProcessPoint(x+1, y, p)
            self.ProcessPoint(x+1, y+1, p)
            self.ProcessPoint(x, y+1, p)
    
    
    
    
    
    
    
    
    
    
    
    

    