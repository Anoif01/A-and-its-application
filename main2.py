# -*- coding: utf-8 -*-
"""
Created on Sat Jan  4 17:35:49 2020

@author: lssya
"""

import time
import os
os.chdir(r'C:\Users\lssya\Desktop\A星')
import numpy as np
import matplotlib.image as mpimg 
import matplotlib.pyplot as plt
import seaborn as sns
import point
import A_star
from matplotlib.patches import Rectangle
#import cv2

#一张图片的像素值范围是[0,255], 因此默认类型是unit8


#%%  Reading image
    
file = 'sm'
img = mpimg.imread(file+'.png')[:,:,0:3] # 只选取rgb三个维度
fig = plt.figure(figsize=[10, 10])
if img.dtype == np.float32: # if you want to convert values to [0;255] integer    
    img = (img * 255).astype(np.uint8) 
plt.imshow(img)
plt.title(file+' OriginImg')

exec_time = int(round(time.time() * 1000))
filename = './' + str(exec_time) + '.png'
plt.savefig(filename)
plt.show()

#%% Binarization 
# black 0,0,0  white 255,255,255  gray x,x,x

th = 250
img_bin = (img[:, :, 0] > th ) * (img[:, :, 1] > th) * (img[:, :, 2] > th)
fig = plt.figure(figsize=[10, 10])
img_bin = 1 - img_bin  #反转0 1
plt.imshow(img_bin, cmap='ocean', vmin=-3.5, vmax=1)
#plt.colorbar()
plt.title(file+' Binarized')

exec_time = int(round(time.time() * 1000))
filename = './' + str(exec_time) + '.png'
plt.savefig(filename)
plt.show()

#%% Dilation operator 

w, h = img_bin.shape
half_ker = 1 # the half side length of conv kernal
bidiimg = np.ones((w,h)) 
for i in range(half_ker, w - half_ker + 1):
    for j in range(half_ker, h - half_ker + 1):
        bidiimg[i,j] = np.max(img_bin[i-half_ker:i+half_ker, j-half_ker:j+half_ker])
mymap = img_bin+bidiimg

fig = plt.figure(figsize=[10, 10])
plt.imshow(mymap, cmap='ocean', vmin=-3.5, vmax=1)
plt.title(file+' Dilated')

exec_time = int(round(time.time() * 1000))
filename = './' + str(exec_time) + '.png'
plt.savefig(filename)
plt.show()

#%%  Define start and end point

#np.where(bidiimg==False)
# 地铁站
#s = [50, 136]
#p_start = point.Point(50, 136)
#f = [32, 65]
#p_end = point.Point(32, 65)

s = [21, 6]
p_start = point.Point(21, 6)
f = [33, 30]
p_end = point.Point(33, 30)


fig = plt.figure(figsize=[10, 10])
plt.imshow(mymap, cmap='ocean', vmin=-3.5, vmax=1)
plt.plot(s[1], s[0], color='orange', marker='o', markersize=20)
plt.plot(f[1], f[0], color='r', marker='x', markersize=30)
plt.title('Start End Points Difineded')

exec_time = int(round(time.time() * 1000))
filename = './' + str(exec_time) + '.png'
plt.savefig(filename)
plt.show()

#%%  List of obstacles
obs = np.where(bidiimg==True)
obs1 = np.vstack(obs).T
obs_points = []
for obs_point in obs1:
    obs_points.append(point.Point(obs_point[0],obs_point[1]))

size = [80, 147]
a_star = A_star.AStar(size, obs_points, p_start, p_end)
print("OK")


#%%

start_time = time.time()

# 初始化起点以及起点的优先值，并加入open_set
a_star.p_start.cost = 0
a_star.open_set.append(a_star.p_start)

while True:
    index = a_star.SelectPointInOpenList() # 返回open_set中优先级最高的索引
    if index < 0:
        print('No path found, algorithm failed!!!')
        break
    
    p = a_star.open_set[index] # 返回open_set中优先级最高的点
    if a_star.IsEndPoint(p): # 判断是否为终点，如果是的话，绘制找到的最短路径
        end_time = time.time()
        print('===== Algorithm finish in %.8f seconds'%(end_time-start_time))
        break
    del a_star.open_set[index] #将该p点从open_set中删除，加入close_set中
    a_star.close_set.append(p)

    # Process all neighbors 处理以p为父节点的所有相邻点（8个）
    x = p.x
    y = p.y
    a_star.ProcessPoint(x-1, y+1, p)
    a_star.ProcessPoint(x-1, y, p)
    a_star.ProcessPoint(x-1, y-1, p)
    a_star.ProcessPoint(x, y-1, p)
    a_star.ProcessPoint(x+1, y-1, p)
    a_star.ProcessPoint(x+1, y, p)
    a_star.ProcessPoint(x+1, y+1, p)
    a_star.ProcessPoint(x, y+1, p)


#%%

plt.imshow(img_bin, cmap='ocean', vmin=-3.5, vmax=1)
plt.plot(s[1], s[0], color='orange', marker='o', markersize=20)
plt.plot(f[1], f[0], color='r', marker='^', markersize=20)

pt = a_star.close_set[-1]
while(True):
    if  a_star.IsStartPoint(pt):
        break
    else:
        plt.plot(pt.parent.y, pt.parent.x, 'g*')
        pt = pt.parent
a_star.SaveImage(plt)        
        
        