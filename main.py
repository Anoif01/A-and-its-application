# -*- coding: utf-8 -*-
"""
Created on Sun Nov  3 23:17:22 2019

@author: lssya
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import My_Map
import A_star
import point



#%% Demander des informations a partir d'utilisateur


print("Donner la taille de map,le nb d'obstacle, separees par un espace:")
size, nbObstacle = map(int,input().split())

print("Donner ensuite le x et y point de depart, separees par un espace:")
print("x,y sont dans [0,%d]" % (size-1))
x_start, y_start = map(int,input().split())

if x_start < 0 or x_start > size-1 or y_start < 0 or y_start > size-1:
    raise AssertionError 
else:    
    p_start = point.Point(x_start, y_start)

print("Donner ensuite le x et y point d'arrive, separees par un espace:")
print("x,y sont dans [0,%d]" % (size-1))
x_end, y_end = map(int,input().split())
if x_end < 0 or x_end > size-1 or y_end < 0 or y_end > size-1:
    raise AssertionError 
else:    
    p_end = point.Point(x_end, y_end)



#%% Dessiner un map


plt.figure(figsize=(5, 5))
Map = My_Map.MyMap(size, nbObstacle) # 创建一个随机地图

obs_point = Map.getObstacles()
ax = plt.gca()
ax.set_xlim([0, Map.size]) # 设置图像的内容与地图大小一致
ax.set_ylim([0, Map.size])

for i in range(Map.size): # 绘制地图：对于障碍物绘制一个灰色的方块，其他区域绘制一个白色的的方块
    for j in range(Map.size):
        rec = Rectangle((i, j), width=1, height=1, edgecolor='gray', facecolor='w')
        ax.add_patch(rec)
                
for i in range(Map.size): # 绘制地图：对于障碍物绘制一个灰色的方块，其他区域绘制一个白色的的方块
    for j in range(Map.size):
        for p in obs_point:
            if i==p.x and j==p.y:
                rec = Rectangle((i, j), width=1, height=1, color='gray')
                ax.add_patch(rec)                

rec = Rectangle((x_start, y_start), width = 1, height = 1, facecolor='b') # 设置起点为蓝色方块
ax.add_patch(rec) # 绘制起点

rec = Rectangle((x_end-1, x_end-1), width = 1, height = 1, facecolor='r') # 设置终点为红色方块
ax.add_patch(rec) # 绘制终点

plt.axis('equal') # 设置图像的坐标轴比例相等
plt.axis('off') # 并且隐藏坐标轴
plt.tight_layout()


#%% Lancer l'algorithme de A*

a_star = A_star.AStar([size,size], obs_point, p_start, p_end)
a_star.RunAndSaveImage(ax, plt) # 调用算法来查找路径



