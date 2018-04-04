#!/usr/bin/env python

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *
import time 

#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 8000 
RADIUS = 30.0
TARGET_RADIUS = 200.0
STEP = 10.0
OBS=[(50,0,50,300), (200,100,50,400), (350,0,50,300), (500,100,50,400)]

class Node:
	def __init__(self,xcoord=0, ycoord=0, cost=0, parent=None):
		self.x = xcoord
		self.y = ycoord
		self.cost = cost
		self.parent = parent

def obsDraw(pygame,screen):
	blue=(0,0,255)
	for o in OBS: 
		pygame.draw.rect(screen,blue,o)

def dist(p1,p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
	if dist(p1,p2) < EPSILON:
		return p2
	else:
		theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
		return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def chooseParent(nn,newnode,nodes):
	for p in nodes:
		if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) < RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
			nn = p
		newnode.cost = nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
		newnode.parent = nn
	return newnode,nn

def nearest_neighbor(nodes, q_target):
	q_near = nodes[0]
	for p in nodes:
		if dist([p.x,p.y],[q_target.x,q_target.y]) < dist([q_near.x,q_near.y],[q_target.x,q_target.y]):
			q_near = p
	return q_near

def extend(nodes, screen, black):
	rand = Node(random.random()*XDIM, random.random()*YDIM)
	nn = nearest_neighbor(nodes, rand)
	interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
	newnode = Node(interpolatedNode[0], interpolatedNode[1])

	if checkIntersect(nn,newnode,OBS):
		[newnode,nn] = chooseParent(nn,newnode,nodes)
		nodes.append(newnode)
		pygame.draw.line(screen, black, [nn.x,nn.y],[newnode.x,newnode.y])
		pygame.display.update()
	
	for e in pygame.event.get():
		if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
			sys.exit("Leaving because you requested it.")
	
	return nodes

def connect(start_nodes, q_near, q_target, screen, black):
	
	theta = atan2(q_target.y - q_near.y, q_target.x - q_near.x)
	q_new_x = q_near.x + STEP*cos(theta)
	q_new_y = q_near.y + STEP*sin(theta)
	q_new_cost = q_near.cost + dist([q_near.x, q_near.y], [q_new_x, q_new_y])
	q_new_parent = q_near
	q_new = Node(q_new_x, q_new_y, q_new_cost, q_new_parent)
	flag = False

	while checkIntersect(q_near, q_new, OBS):
		[q_new, q_near] = chooseParent(q_near, q_new, start_nodes)
		start_nodes.append(q_new)
		pygame.draw.line(screen, black, [q_near.x,q_near.y],[q_new.x,q_new.y])
		pygame.display.update()

		if q_new.x == q_target.x and q_new.y == q_target.y:
			flag = True
			break

		q_near = q_new
		theta = atan2(q_target.y - q_near.y, q_target.x - q_near.x)
		if dist([q_target.x, q_target.y], [q_near.x, q_near.y]) > STEP:
			q_new_x = q_near.x + STEP*cos(theta)
			q_new_y = q_near.y + STEP*sin(theta)
		else:
			q_new_x = q_target.x
			q_new_y = q_target.y
		
		q_new_cost = q_near.cost + dist([q_near.x, q_near.y], [q_new_x, q_new_y])
		q_new_parent = q_near
		q_new = Node(q_new_x, q_new_y, q_new_cost, q_new_parent)
				
	return start_nodes, flag

def drawPath(nodes, pygame, screen):
	last_node = nodes[-1]
	start = nodes[0]
	red = 255, 10, 10
	while last_node != start:
		pygame.draw.line(screen, red, [last_node.x, last_node.y], [last_node.parent.x, last_node.parent.y], 5)  
		last_node = last_node.parent

def main():
	pygame.init()
	screen = pygame.display.set_mode(WINSIZE)
	pygame.display.set_caption('Bi-directional_RRT_star')
	white = 255, 255, 255
	black = 20, 20, 40
	screen.fill(white)
	obsDraw(pygame,screen)
	
	start = Node(0.0, 0.0)     # Start in the corner
	goal  = Node(630.0, 470.0)

	start_nodes = []
	goal_nodes  = []

	start_nodes.append(start)
	goal_nodes.append(goal)

	i = 0
	flag = False
	start_time = time.time()
	while i < NUMNODES and flag != True:

		rand = Node(random.random()*XDIM, random.random()*YDIM)
		nn = nearest_neighbor(start_nodes, rand)
		start_nodes, flag_dummy = connect(start_nodes, nn, rand, screen, black)
		
		rand = Node(random.random()*XDIM, random.random()*YDIM)
		nn = nearest_neighbor(goal_nodes, rand)
		goal_nodes, flag_dummy = connect(goal_nodes, nn, rand, screen, black)
		q_target = goal_nodes[-1]
		
		# try to connect q_near and q_target if distance is less than a target radius
		q_near = nearest_neighbor(start_nodes, q_target)	
		if (dist([q_target.x, q_target.y], [q_near.x, q_near.y]) < TARGET_RADIUS):
			start_nodes, flag = connect(start_nodes, q_near, q_target, screen, black)

		i += 1
		for e in pygame.event.get():
			if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
				sys.exit("Leaving because you requested it.")

	if flag == True:
		end_time = time.time()
		time_taken = end_time - start_time
		total_cost = start_nodes[-1].cost + goal_nodes[-1].cost
		print "Path Found"
		print ""
		print "Bi-directional RRT* Connect Stats"
		print ""
		print "Cost       : " + str(total_cost) + ' units'
		print "Time Taken : " + str(time_taken) + ' sec'

		drawPath(start_nodes, pygame, screen)
		drawPath(goal_nodes, pygame, screen)
		pygame.display.update()
		pygame.image.save(screen, "bi_rrt_connect_both.jpg")
	else:
		print "Path not found. Try increasing the number of iterations"


if __name__ == '__main__':
	main()
	running = True
	while running:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False



