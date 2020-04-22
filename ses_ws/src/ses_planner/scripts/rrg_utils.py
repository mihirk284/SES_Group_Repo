#!/usr/bin/env python
import numpy as np
import networkx as nx
import random
import rospy
import std_msgs.msg
import math
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
import matplotlib.pyplot as plt

def init_graph(p):
	G=nx.Graph()
	G.add_node((p.x,p.y,p.z))
	return G

def choose_random_node(G):
	l=list(G.nodes)
	n=random.randint(0,len(l)-1)
	return l[n]

def cartesian_distance(p1,p2):
	dx=p1[0]-p2[0]
	dy=p1[1]-p2[1]
	dz=p1[2]-p2[2]
	return math.sqrt(dx**2+dy**2+dz**2)

def spawn_new_node(G,space,radius,granularity):
	n=choose_random_node(G)
	l=[i for i in np.arange(0,radius,granularity)]
	p=Point()
	dx=l[random.randint(0,len(l)-1)]
	dy=l[random.randint(0,len(l)-1)]
	dz=l[random.randint(0,len(l)-1)]
	p.x=min(n[0]+dx,space[1][0])
	p.y=min(n[1]+dy,space[1][1])
	p.z=min(n[2]+dz,space[1][2])
	nds=[]
	for i in list(G.nodes):
		if cartesian_distance((p.x,p.y,p.z),i)<=radius:
			nds.append(i)
	for i in nds:
		G.add_edge((p.x,p.y,p.z),i,weight=cartesian_distance((p.x,p.y,p.z),i))
	return p

#arguments
#G: nx.Graph
#space: tuple ((startx,starty,startz),(endx,endy,endz))
#source_pt: Point
#target: ((centrex,centrey,centrez),radius)
#granularity: float, minimum interval between pts. default 0.1
#n_pts: number of nodes generated. default 10000
#radius: radius of sphere within which new points are spawned. default: 0.5
def run_RRG(G, space, source_pt, target, granularity=0.1, n_pts=10000, radius=0.5):
	if G==None:
		G=init_graph(source_pt)
	pa=PoseArray()
	pa.poses=[]
	occ=None
	s=(source_pt.x,source_pt.y,source_pt.z)
	for i in range(n_pts):
		pt=spawn_new_node(G,space,radius,granularity)
		p=Pose()
		p.position=pt
		p.orientation=Quaternion()
		p.orientation.x=0
		p.orientation.y=0
		p.orientation.z=0
		p.orientation.w=0
		pa.poses.append(p)
	rospy.wait_for_service('check boxes')
	try:
		chk = rospy.ServiceProxy('check boxes', checkBoxesService)	
		occ=chk(pa)
	except rospy.ServiceException as e:
		print "Service call failed: %s"%e
	for i,j in pa.poses,occ.data:
		if j==0:
			G.remove_node((i.position.x,i.position.y,i.position.z))
	tgt=[]
	dp=[]
	dpl=[]
	shortest_path=[]
	for i in list(G.nodes):
		if cartesian_distance(i,target[0])<=target[1]:
			tgt.append(i)

	for i in tgt:
		dpl.append(nx.dijkstra_path_length(G,s,i))
		dp.append(nx.dijkstra_path(G,s,i))
	if dpl!=[]:
		i=dpl.index(min(dpl))
		shortest_path=dp[i]
	waypts=[]
	for i in shortest_path:
		p1=Point()
		p1.x=i[0]
		p1.y=i[1]
		p1.z=i[2]
		waypts.append(p1)
	return G,waypts

def main():
	rospy.init_node('rrg',anonymous=True)
	p=Point()
	p.x=1
	p.y=1
	p.z=1
	target=((3,4,4),1)
	radius=0.5
	G=None
	G,waypts=run_RRG(G,((0,0,0),(10,10,10)),p,target,0.1,1000,radius)
	print waypts
	nx.draw(G)
	plt.show()
if __name__ == '__main__':
    main()
