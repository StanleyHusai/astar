#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import math
import csv
import numpy
from numpy import matrix

SAFETY_OFFSET = 4   # number of grid away from the wall

address = csv.reader(open('BinaryOccupancyGrid.csv',
                                        'r'))
grid_map = []
for rowCsv in address:
    grid_map.append(rowCsv[0:]) # build grid_map matrix from csv file

class Node_Elem:

    def __init__(self, parent, x, y, F):
        self.parent = parent
        self.x = x
        self.y = y
        self.F = F
        
class A_Star:

    def __init__(self, s_x, s_y, e_x, e_y, w=400, h=400):
        self.s_x = s_x
        self.s_y = s_y
        self.e_x = e_x
        self.e_y = e_y
        
        self.width = w
        self.height = h
        
        self.open = []
        self.close = []
        self.path = []
        
    def find_path(self):
        p = Node_Elem(None, self.s_x, self.s_y, 0.0) #build initial node
        while True:
            self.extend_round(p) 
            if not self.open: # if open list is empty
                return
            idx, p = self.get_best() #find node with smallest F
            if self.is_target(p):    #find path
                self.make_path(p)    #construst path
                return
            self.close.append(p)     #put into close lost
            del self.open[idx]       #delete it from open list
            
    def make_path(self,p):
        while p:
            self.path.append((p.x, p.y))
            p = p.parent
        
    def is_target(self, i):
        return i.x == self.e_x and i.y == self.e_y
        
    def get_best(self):

        best = None
        bv = 10000000000 
        bi = -1
        for idx, i in enumerate(self.open):
            value = self.get_F(i)
            if value < bv:
                best = i
                bv = value
                bi = idx
        return bi, best
        
    def get_F(self, i):

        return i.F + self.get_H(i) # F = G + H
        
    def extend_round(self, p): # search neighbors

        xs = (-1, 0, 1, -1, 1, -1, 0, 1)
        ys = (-1,-1,-1,  0, 0,  1, 1, 1)

        for x, y in zip(xs, ys):
            new_x, new_y = x + p.x, y + p.y    # new are neighbor 8 nodes, p is current parent node
            if not self.is_valid_coord(new_x, new_y): # check if the neighbor node it is valid
                continue
            node = Node_Elem(p, new_x, new_y, p.F+self.get_G(
                        p.x, p.y, new_x, new_y)) # construct new neighbor node of current parent node 
            if self.node_in_close(node): # ignore if it is already in close list
                continue
            i = self.node_in_open(node) # it is already in open list
            if i != -1:
                if self.open[i].F > node.F: # has a smaller F
                    self.open[i].parent = p
                    self.open[i].F = node.F # update its parent node and F
                continue
            self.open.append(node)
            
    def get_G(self, x1, y1, x2, y2): # calculate G
        if x1 == x2 or y1 == y2:
            return 1.0
        return 1.4
        
    def get_H(self,i): # calculate H
        return math.sqrt((self.e_x-i.x)*(self.e_x-i.x) + (self.e_y-i.y)*(self.e_y-i.y))*1.2

    def node_in_close(self, node):
        for i in self.close:
            if node.x == i.x and node.y == i.y:
                return True
        return False
        
    def node_in_open(self, node):
        for i, n in enumerate(self.open):
            if node.x == n.x and node.y == n.y:
                return i
        return -1
        
    def is_valid_coord(self, x, y):

        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False

        if grid_map[y-SAFETY_OFFSET][x-SAFETY_OFFSET] == '0':
            return False
        if grid_map[y-SAFETY_OFFSET][x] == '0':
            return False
        if grid_map[y-SAFETY_OFFSET][x+SAFETY_OFFSET] == '0':
            return False
        if grid_map[y][x-SAFETY_OFFSET] == '0':
            return False
        if grid_map[y][x+SAFETY_OFFSET] == '0':
            return False
        if grid_map[y+SAFETY_OFFSET][x-SAFETY_OFFSET] == '0':
            return False
        if grid_map[y+SAFETY_OFFSET][x] == '0':
            return False
        if grid_map[y+SAFETY_OFFSET][x+SAFETY_OFFSET] == '0':
            return False

        if grid_map[y][x] == '1':
            return True
 
    def get_searched(self):
        l = []
        for i in self.open:
            l.append((i.x, i.y))
        for i in self.close:
            l.append((i.x, i.y))
        return l
        
def print_grid_map():
    for line in grid_map:
        print ''.join(line)
        
def path_planning():
    s_x=300    # (300,100)in python_matrix frame
    s_y=100    # (0,0) in point_cloud map frame
    print 'start', s_x,s_y
    e_x=163
    e_y=150    
    print 'end', e_x,e_y
    a_star = A_Star(s_x, s_y, e_x, e_y)
    a_star.find_path()
    searched = a_star.get_searched()
    path = a_star.path
    myFile = open('test_path.csv', 'w')  #write path to csv file
    with myFile:
    	writer = csv.writer(myFile)
    	writer.writerows(path)
    print path
    print "path length is %d"%(len(path))
    print "searched squares count is %d"%(len(searched))
    
def ndtPoseCallback(self, data):  # callback ndt pose
    self.curPose = data  
    self.update = 1

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/ndt_pose', PoseStamped, self.ndtPoseCallback)  
    rospy.spin()


if __name__ == "__main__":
    #listener()
    path_planning()
    #print_grid_map()