#--------------------------------------Libraries------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import random
import numpy as np
#--------------------------------------Classes--------------------------------------------
#Environment and Obstacles
class env3d:

#environment class is defined by obstacle vertices and boundaries
	def __init__(self, xmin,xmax,ymin,ymax,zmin,zmax):
		# self.x = x
		# self.y = y
		# self.z = z
		self.xmin=xmin
		self.xmax=xmax
		self.ymin=ymin
		self.ymax=ymax
		self.zmin=zmin
		self.zmax=zmax 
    		
#Collision checking for a path
	def inobstacle(self,x1,y1,z1,x2,y2,z2):
		c=1 #assume no collision
		for j in range(0,51):
			u=j/50
			x = x1*u + x2*(1-u)
			y = y1*u + y2*(1-u)
			z = z1*u + z2*(1-u)

			# first barrier avoidance 
			if x >= (1-0.061) and x<= (1+0.061):
				if y < 0.486 or y > 0.514:
					c=0
					break
				elif z > 0.814 or z < 0.786:
					c=0
					break

			#second barrier avoidance 
			if x >= (1.25-0.061) and x<= (1.25+0.061):
				if y < 0.486 or y > 0.514:
					c=0
					break
				elif z > 0.214 or z < 0.186:
					c=0
					break
			if c==0: break	
		return c

#check if newly added sample is in the free configuration space
	def isfree(self):
		n= G.number_of_nodes()-1
		(x,y,z)= (G.x[n], G.y[n], G.z[n])

		if x<=0 or y<=0 or z<=0 or z==1 or y==1:
			G.remove_node(n)
			return 0

		#I have taken margin of 0.061 to play safe	
		# first barrier avoidance 
		if x >= (1-0.061) and x<= (1+0.061):
			if y < 0.486 or y > 0.514:
				G.remove_node(n)
				return 0
			elif z > 0.814 or z < 0.786:
				G.remove_node(n)
				return 0

		#second barrier avoidance 
		if x >= (1.25-0.061) and x<= (1.25+0.061):
			if y < 0.486 or y > 0.514:
				G.remove_node(n)
				return 0
			elif z > 0.214 or z < 0.186:
				G.remove_node(n)
				return 0
				
#check if current node is in goal region
	def ingoal(self):
		n= G.number_of_nodes()-1
		(x,y,z)= (G.x[n], G.y[n],G.z[n]) 
		if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax) and (z>=zgmin) and (z<=zgmax) :
			return 1
		else:
			return 0
			
#check for a specific node
	def isfree_xy(self,x,y): 
		obs_num = len(self.x)/4 #four vertices for each rectangular obstacle
		for i in range(1,obs_num+1):
			xomin=self.x[4*(i-1)]
			xomax=self.x[4*(i-1)+2]
			yomin=self.y[4*(i-1)]
			yomax=self.y[4*(i-1)+1]
			if (x>=xomin) and (x<=xomax) and (y>=yomin) and (y<=yomax):
				return 0
				break
				
#draw the edges of a 3d cuboid			
	def cubedraw(self,obsx,obsy,obzl,obzh,k):
		x = obsx
		y = obsy
		zl = [obzl,obzl,obzl,obzl,obzl]
		zh = [obzh,obzh,obzh,obzh,obzh]
		
		ax.plot(x, y, zl,k)
		ax.plot(x,y,zh,k)
		for i in range (0,len(x)-1):
			obx = [x[i],x[i]]
			oby = [y[i],y[i]]
			obz = [zl[i],zh[i]]
			ax.plot(obx,oby,obz,k)
					
#-----------------------------------------------------------------------------------------
class RRT3d:
	def __init__(self,nstart):
		(x,y,z)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		#first node is the only node whose parent is itself
		self.parent.append(0)
	
	#get metric value (current metric is euclidean distance)
	def metric(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		x1=float(x1)
		y1=float(y1)
		x2=float(x2)
		y2=float(y2)
		z1=float(z1)
		z2=float(z2)
		px=(x1-x2)**(2)
		py=(y1-y2)**(2)
		pz=(z1-z2)**(2)
		metric = (px+py+pz)**(0.5)
		return metric
			
	#expand a random point
	#calls subroutines to find nearest node and connect it
	def expand (self):
		#add random node
		x = random.uniform (0, 2)
		y = random.uniform (0, 1)
		z = random.uniform (0, 1)

		n= self.number_of_nodes() #new node number

		self.add_node(n,x,y,z)
		
		if E.isfree()!=0:
			#find nearest node
			nnear = self.near(n)
			#find new node based on step size
			self.step(nnear,n)
			#connect the random node with its nearest node
			self.connect(nnear,n)
		
	#nearest node
	def near(self,n):
		#find a near node
		dmin = self.metric(0,n)
		nnear = 0
		for i in range(0,n):
			if self.metric(i,n) < dmin:
				dmin=self.metric(i,n)
				nnear = i
		return nnear
		
#step size
	def step(self,nnear,nrand):
		d = self.metric(nnear,nrand)
		if d>dmax:
			u=dmax/d
			(xnear,ynear,znear)= (self.x[nnear],self.y[nnear],self.z[nnear])
			(xrand,yrand,zrand)= (self.x[nrand],self.y[nrand],self.z[nrand]) 
			(px,py,pz)=(xrand-xnear,yrand-ynear,zrand-znear)
			theta = math.atan2(py,px)
			x=xnear+dmax*math.cos(theta)
			y=ynear+dmax*math.sin(theta)
			alpha = math.atan2(pz,y)
			z=znear+dmax*math.sin(alpha)
			self.remove_node(nrand)
			self.add_node(nrand,x,y,z) #this is a new node between rand and near

#connect two nodes (local planner)
	def connect(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		n= G.number_of_nodes()-1
		#subdivide path into 100 small segments and ensure each segment is collision free
		if E.inobstacle(x1,y1,z1,x2,y2,z2)==0:
			self.remove_node(n2)
		else:
			self.add_edge(n1,n2)
			
#connect two trees (Boundary Valued Problem)
	def BVP_to(self,A):
		#attempt to connect this node
		n1=self.number_of_nodes()-1
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		c=0 #assume no connection
		num=A.number_of_nodes()
		for i in range (0,num-1):
			(x2,y2,z2)= (A.x[i],A.y[i],A.z[i])
			if E.inobstacle(x1,y1,z1,x2,y2,z2)==1:
				self.add_node(n1+1,x2,y2,z2)
				self.add_edge(n1,n1+1)
				self.BVPnode=n1+1
				A.BVPnode=i
				c=1
				break
		return c					

#add node
	def add_node(self,n,x,y,z):
		self.x.insert(n, x)
		self.y.insert(n, y)
		self.z.insert(n, z)

#remove node
	def remove_node(self,n):
		self.x.pop(n)
		self.y.pop(n)
		self.z.pop(n)

#add edge
	def add_edge(self,parent,child):
		self.parent.insert(child,parent)
		
#remove node		
	def remove_edge(self,n):
		self.parent.pop(n)
		
#clear
	def clear(self,nstart):
		(x,y,z)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		#first node is the only node whose parent is itself
		self.parent.append(0)
		
#number of nodes
	def number_of_nodes(self):
		return len(self.x)
		
#path to goal
	def path_to_goal(self):
		i=self.BVPnode
		#add goal state to and its parent node to the path	
		self.path=[]
		self.path.append(i)
		
		i = len(self.parent) -1

		newpos=self.parent[i]
		#keep adding parents	
		while (newpos!=0):
			self.path.append(newpos)
			newpos=self.parent[newpos]	
		#add start state
		self.path.append(0)
					
	#draw tree
	def showtree(self,k):
		for i in range (0,self.number_of_nodes()):
			par=self.parent[i]
			x=[self.x[i],self.x[par]]
			y=[self.y[i],self.y[par]]
			z=[self.z[i],self.z[par]]
			ax.plot(x,y,z,k,lw=0.5)
			
	#draw path 
	def showpath(self,k):
		for i in range (len(self.path)-1):
			n1=self.path[i]
			n2=self.path[i+1]
			x=[self.x[n1],self.x[n2]]
			y=[self.y[n1],self.y[n2]]
			z=[self.z[n1],self.z[n2]]
			ax.plot(x,y,z,k,lw=1,markersize=3)
			 
			
#--------------------------------------Global Definitions---------------------------------
#node limit
nmax = 5000

#goal region
xg = 1.750
yg = 0.5
zg = 0.061
epsilon = 0.05

xgmin, xgmax = xg-epsilon, xg+epsilon
ygmin, ygmax = yg-epsilon, yg+epsilon
zgmin, zgmax = zg-epsilon, zg+epsilon


#extend step size
dmax = 0.05
#start the root of the tree
nstart =(0.2,0.5,0.026) 

#specify vertices for rectangular obstacles (each object has four vertices)
#obstacles known a priori
#
# vx= [18,18,38,38,   60,60,80,80, 40,40,60,60]
# vy= [48,100,100,48, 40,90,90,40, 0,48,48, 0]
# vz = [0,100]

#create an RRT tree with a start node
G=RRT3d(nstart)
S=RRT3d((xg,yg,zg))

#environment instance
E=env3d(0,2,0,1,0,1)
	
#draw setup
fig = plt.figure()
ax = fig.gca(projection='3d')

#--------------------------------------Functions------------------------------------------
#draw trees and environment
def draw ():
	plt.show()
				
				
#--------------------------------------RRT Implementation---------------------------------
def main():
	#balance between extending and biasing	
	for i in range(0,nmax):
		G.expand()
		if G.BVP_to(S)==1: break
		S.expand()
		if S.BVP_to(G)==1: break
		
	G.path_to_goal()
	S.path_to_goal()

	#display initial plan under limited sensing
	draw()
	
# run main when RRT is called
if __name__ == '__main__':
    main()
