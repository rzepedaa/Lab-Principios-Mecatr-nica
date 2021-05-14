#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
theta = 0
x2 = 0
y2 = 0
x2Ant = 0
y2Ant = 0
theta2= 0
x3 = 0
y3 = 0
x3Ant = 0
y3Ant = 0
theta3 = 0
x4 = 0
y4 = 0
x4Ant = 0
y4Ant = 0
theta4 = 0
x5 = 0
y5 = 0
x5Ant = 0
y5Ant = 0
theta5 = 0
x6 = 0
y6 = 0
x6Ant = 0
y6Ant = 0
theta6 = 0

contador = 0


xm = 0
ym = 0
xg = 7
yg = 10

def poseCallback1(pose_message):
	global x
	global y
	global theta
	   
	x = pose_message.x
	y = pose_message.y
	theta = pose_message.theta

def poseCallback2(pose_message):
	global x2
	global y2
	global theta2
	    
	x2 = pose_message.x
	y2 = pose_message.y
	theta2 = pose_message.theta
	#print ('x2=', x2, 'y2=', y2)

def poseCallback3(pose_message):
	global x3
	global y3
	global theta3
	    
	x3 = pose_message.x
	y3 = pose_message.y
	theta3 = pose_message.theta
	#print ('x3=', x3, 'y3=', y3)

def poseCallback4(pose_message):
	global x4
	global y4
	global theta4
	    
	x4 = pose_message.x
	y4 = pose_message.y
	theta4 = pose_message.theta
	#print ('x4=', x4, 'y4=', y4)
	
def poseCallback5(pose_message):
	global x5
	global y5
	global theta5
	    
	x5 = pose_message.x
	y5 = pose_message.y
	theta5 = pose_message.theta
	#print ('x5=', x5, 'y5=', y5)
	
def poseCallback6(pose_message):
	global x6
	global y6
	global theta6
	    
	x6 = pose_message.x
	y6 = pose_message.y
	theta6 = pose_message.theta
	#print ('x6=', x6, 'y6=', y6)  


def orientate (xgoal, ygoal):
	global x
	global y
	global theta

	velocity_message = Twist()
	cmd_vel_topic = '/turtle1/cmd_vel'

	while(True):
		ka = 0.6
		desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
		if desired_angle_goal < 0:
			desired_angle_goal = desired_angle_goal + 2*math.pi
		else:
			desired_angle_goal = desired_angle_goal
		if theta<0:
			theta=2*math.pi+theta
		dtheta = desired_angle_goal-theta        
		angular_speed = ka * (dtheta)

		velocity_message.linear.x = 0.0
		velocity_message.angular.z = angular_speed
		velocity_publisher.publish(velocity_message)
		#print ('x=', x, 'y=', y)

		if (abs(dtheta) < 0.005):
			time.sleep(0.3)
			break

# Given three colinear points p, q, r, the function checks if 
# point q lies on line segment 'pr' 
def onSegment(p, q, r):
	if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and 
		(q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
		return True
	return False
  
def orientation(p, q, r):
	# to find the orientation of an ordered triplet (p,q,r)
	# function returns the following values:
	# 0 : Colinear points
	# 1 : Clockwise points
	# 2 : Counterclockwise
      
	val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
	if(val>0):
          
		# Clockwise orientation
		return 1
	elif(val<0):
          
		# Counterclockwise orientation
		return 2
	else:
          
		# Colinear orientation
		return 0
  
# The main function that returns true if 
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):
      
	# Find the 4 orientations required for 
	# the general and special cases
	o1 = orientation(p1, q1, p2)
	o2 = orientation(p1, q1, q2)
	o3 = orientation(p2, q2, p1)
	o4 =orientation(p2, q2, q1)
  
	# General case
	if ((o1 != o2) and (o3 != o4)):
		return True
  
	# Special Cases
  
	# p1 , q1 and p2 are colinear and p2 lies on segment p1q1
	if ((o1 == 0) and onSegment(p1, p2, q1)):
		return True
  
	# p1, q1 and q2 are colinear and q2 lies on segment p1q1
	if ((o2 == 0) and onSegment(p1, q2, q1)):
		return True
  
	# p2 , q2 and p1 are colinear and p1 lies on segment p2q2
	if ((o3 == 0) and onSegment(p2, p1, q2)):
		return True
  
	# p2 , q2 and q1 are colinear and q1 lies on segment p2q2	
	if ((o4 == 0) and onSegment(p2, q1, q2)):
		return True
		# If none of the cases
		return False
		
def area(x1, y1, x2, y2, x3, y3):
 
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)
				
def isInside(x1, y1, x2, y2, x3, y3, x, y):
 
    # Calculate area of triangle ABC
    A = area (x1, y1, x2, y2, x3, y3)
 
    # Calculate area of triangle PBC
    A1 = area (x, y, x2, y2, x3, y3)
     
    # Calculate area of triangle PAC
    A2 = area (x1, y1, x, y, x3, y3)
     
    # Calculate area of triangle PAB
    A3 = area (x1, y1, x2, y2, x, y)
     
    # Check if sum of A1, A2 and A3
    # is same as A
    if(A == A1 + A2 + A3):
        return True
    else:
        return False

def desvio2 (conflicto,xG,yG):
	global x
	global y
	global theta
	global x2
	global y2
	global x3
	global y3
	global x4
	global y4
	global x5
	global y5
	global x6
	global y6
	global xm
	global ym
	
	print(conflicto)
	print('x=',xG)
	print('y=',yG)
	
	caso=[0,0,0,0]
	
	m=(yG-y)/(xG-x)
	
	b=y-m*x
	
	mPerp=(-1)*1/m
	
	bPerp=y-mPerp*x

		
	if theta<0:
		angComp=theta+2*math.pi
	else:
		angComp=theta
	
	for i in conflicto:
		
		if i=='2':
			yAux1=m*x2+b
			yAux2=mPerp*x2+bPerp
			yComp=y2
		else:
			if i=='3':
				yAux1=m*x3+b
				yAux2=mPerp*x3+bPerp
				yComp=y3
			else:
				if i=='4':
					yAux1=m*x4+b
					yAux2=mPerp*x4+bPerp
					yComp=y4
				else:
					if i=='5':
						yAux1=m*x5+b
						yAux2=mPerp*x5+bPerp
						yComp=y5
					else:
						yAux1=m*x6+b
						yAux2=mPerp*x6+bPerp
						yComp=y6	
		
		if angComp<math.pi/2 and angComp>0:
			if yComp>yAux1:
				if yComp>yAux2:
					caso[0]=1
				else:
					caso[2]=1
			else:
				
				if yComp>yAux2:
					caso[1]=1
				else:
					caso[3]=1
		else:
			if angComp<math.pi and angComp>math.pi/2:
				if yComp>yAux1:
					
					if yComp>yAux2:
						caso[1]=1
					else:
						caso[3]=1
				else:
								
					if yComp>yAux2:
						caso[0]=1
					else:
						caso[2]=1
			else:
				if angComp<3/2*math.pi and angComp>math.pi:
					if yComp>yAux1:
						
						if yComp>yAux2:
							caso[3]=1
						else:
							caso[1]=1
					else:
									
						if yComp>yAux2:
							caso[2]=1
						else:
							caso[0]=1
				else:
					if yComp>yAux1:
						
						if yComp>yAux2:
							caso[2]=1
						else:
							caso[0]=1
					else:
									
						if yComp>yAux2:
							caso[3]=1
						else:
							caso[1]=1
													
	print(caso)
	
	if caso[0]==1 and caso[1]==1: 
		if caso[2]==1:
			ang=theta-math.pi/4-math.pi/16;
			if ang<0:
				ang=2*math.pi+ang
			xS=1.2*math.cos(ang);
			yS=1.2*math.sin(ang);
		else:
			ang=theta+math.pi/4+math.pi/16;
			if ang<0:
				ang=2*math.pi+ang
			xS=1.2*math.cos(ang);
			yS=1.2*math.sin(ang);
	else:	
		if caso[0]==1:
			ang=theta-7*math.pi/16;
			if ang<0:
				ang=2*math.pi+ang
			xS=1.2*math.cos(ang);
			yS=1.2*math.sin(ang);
		else:
			if caso[1]==1:
				ang=theta+7*math.pi/16;
				if ang<0:
					ang=2*math.pi+ang
				xS=1.2*math.cos(ang);
				yS=1.2*math.sin(ang);
			else:
				xS=1.2*math.cos(theta);
				yS=1.2*math.sin(theta);
				
	if (x+xS)<0.3 or (x+xS)>10.7:
		xS=(-0.5)*xS;
	
	if (y+yS)<0.3 or (y+yS)>10.7:
		yS=(-0.5)*yS;				

	orientate(x+xS,y+yS)
	time.sleep(0.3)
	go_to_goal2(x+xS,y+yS,conflicto)
	
	if xm!=xG or ym!=yG:
		return True
		
	time.sleep(0.3)
	orientate(xG,yG)
	time.sleep(0.3)
	
def desvio (conflicto,xG,yG):
	global x
	global y
	global theta
	global x2
	global y2
	global theta2
	global x3
	global y3
	global theta3
	global x4
	global y4
	global theta4
	global x5
	global y5
	global theta5
	global x6
	global y6
	global theta6
	global xm
	global ym
	
	print(conflicto)
	print('x=',xG)
	print('y=',yG)
	
	xS=-1.0*math.cos(theta);
	yS=-1.0*math.sin(theta);
	
	if x+xS<0.3 or x+xS>10.7:
		xS=(-0.5)*xS;
	
	if y+yS<0.3 or y+yS>10.7:
		yS=(-0.5)*yS;
	orientate(x+xS,y+yS)
	time.sleep(0.3)
	go_to_goal(x+xS,y+yS)
	time.sleep(2.0)
	
	if xm!=xG or ym!=yG:
		return True
	orientate(xG,yG)
	
	p1=[x,y]
	q1=[xG,yG]
	p=False
	if (math.sqrt(((x-x2)**2)+((y-y2)**2))<2.0):
		p2=[x2-0.5*math.cos(theta2),y2-0.5*math.sin(theta2)]
		if theta2<0:
			theta2=2*math.pi+theta2
		q2=[x2+1.5*math.cos(theta2),y2+1.5*math.sin(theta2)]
		if (doIntersect(p1,q1,p2,q2)):
			p=True
	if math.sqrt(((x-x3)**2)+((y-y3)**2))<2.0:
		p2=[x3-0.5*math.cos(theta3),y3-0.5*math.sin(theta3)]
		if theta3<0:
			theta3=2*math.pi+theta3
		q2=[x3+1.5*math.cos(theta3),y3+1.5*math.sin(theta3)]
		if doIntersect(p1,q1,p2,q2):
			p=True
	if math.sqrt(((x-x4)**2)+((y-y4)**2))<2.0:
		p2=[x4-0.5*math.cos(theta4),y4-0.5*math.sin(theta4)]
		if theta4<0:
			theta4=2*math.pi+theta4
		q2=[x4+1.5*math.cos(theta4),y4+1.5*math.sin(theta4)]
		if doIntersect(p1,q1,p2,q2):
			p=True
	if math.sqrt(((x-x5)**2)+((y-y5)**2))<2.0:
		p2=[x5-0.5*math.cos(theta5),y5-0.5*math.sin(theta5)]
		if theta5<0:
			theta5=2*math.pi+theta5
		q2=[x5+1.5*math.cos(theta5),y5+1.5*math.sin(theta5)]
		if doIntersect(p1,q1,p2,q2):
			p=True
	if math.sqrt(((x-x6)**2)+((y-y6)**2))<2.0:	
		p2=[x6-0.5*math.cos(theta6),y6-0.5*math.sin(theta6)]
		if theta6<0:
			theta6=2*math.pi+theta6
		q2=[x6+1.5*math.cos(theta6),y6+1.5*math.sin(theta6)]
		if doIntersect(p1,q1,p2,q2):
			p=True
	if p:

		caso=[0,0,0,0]
		
		m=(yG-y)/(xG-x)
		
		b=y-m*x
		
		mPerp=(-1)*1/m
		
		bPerp=y-mPerp*x

		if theta<0:
			angComp=theta+2*math.pi
		else:
			angComp=theta
		
		for i in conflicto:
			
			if i=='2':
				yAux1=m*x2+b
				yAux2=mPerp*x2+bPerp
				yComp=y2
			else:
				if i=='3':
					yAux1=m*x3+b
					yAux2=mPerp*x3+bPerp
					yComp=y3
				else:
					if i=='4':
						yAux1=m*x4+b
						yAux2=mPerp*x4+bPerp
						yComp=y4
					else:
						if i=='5':
							yAux1=m*x5+b
							yAux2=mPerp*x5+bPerp
							yComp=y5
						else:
							yAux1=m*x6+b
							yAux2=mPerp*x6+bPerp
							yComp=y6	
			
			if angComp<math.pi/2 and angComp>0:
				if yComp>yAux1:
					if yComp>yAux2:
						caso[0]=1
					else:
						caso[2]=1
				else:
					
					if yComp>yAux2:
						caso[1]=1
					else:
						caso[3]=1
			else:
				if angComp<math.pi and angComp>math.pi/2:
					if yComp>yAux1:
						
						if yComp>yAux2:
							caso[1]=1
						else:
							caso[3]=1
					else:
									
						if yComp>yAux2:
							caso[0]=1
						else:
							caso[2]=1
				else:
					if angComp<3/2*math.pi and angComp>math.pi:
						if yComp>yAux1:
							
							if yComp>yAux2:
								caso[3]=1
							else:
								caso[1]=1
						else:
										
							if yComp>yAux2:
								caso[2]=1
							else:
								caso[0]=1
					else:
						if yComp>yAux1:
							
							if yComp>yAux2:
								caso[2]=1
							else:
								caso[0]=1
						else:
										
							if yComp>yAux2:
								caso[3]=1
							else:
								caso[1]=1
														
		print(caso)
	 	
		
		if caso[0]==1 and caso[1]==1: 
			if caso[2]==1:
				ang=theta-math.pi-math.pi/16;
				if ang<0:
					ang=2*math.pi+ang
				xS=1.2*math.cos(ang);
				yS=1.2*math.sin(ang);
			else:
				ang=theta+math.pi+math.pi/16;
				if ang<0:
					ang=2*math.pi+ang
				xS=1.2*math.cos(ang);
				yS=1.2*math.sin(ang);
		else:	
			if caso[0]==1:
				ang=theta-7*math.pi/16;
				if ang<0:
					ang=2*math.pi+ang
				xS=1.2*math.cos(ang);
				yS=1.2*math.sin(ang);
			else:
				if caso[1]==1:
					ang=theta+7*math.pi/16;
					if ang<0:
						ang=2*math.pi+ang
					xS=1.2*math.cos(ang);
					yS=1.2*math.sin(ang);
				else:
					xS=1.2*math.cos(theta);
					yS=1.2*math.sin(theta);
				
			
					
		if x+xS<0.3 or x+xS>10.7:
			xS=(-0.5)*xS;
		
		if y+yS<0.3 or y+yS>10.7:
			yS=(-0.5)*yS;
		

		orientate(x+xS,y+yS)
		time.sleep(0.3)
		go_to_goal(x+xS,y+yS)
		
	if xm!=xG or ym!=yG:
		return True	
	
	time.sleep(0.3)
	orientate(xG,yG)
	time.sleep(0.3)
	
def go_to_goal2 (xgoal, ygoal, ex):
	global x
	global y
	global theta
	global x2
	global y2
	global x2Ant
	global y2Ant
	global theta2
	global x3
	global y3
	global x3Ant
	global y3Ant
	global theta3
	global x4
	global y4
	global x4Ant
	global y4Ant
	global theta4
	global x5
	global y5
	global x5Ant
	global y5Ant
	global theta5
	global x6
	global y6
	global x6Ant
	global y6Ant
	global theta6
	global contador
	global xm
	global ym
	
	
	velocity_message = Twist()
	cmd_vel_topic = '/turtle1/cmd_vel'

	while(True):
		
		conflicto=[]
		choque=[]
		
		p=False
		pC=False
		
		p1=[x,y]
		q1=[xgoal,ygoal]

		kv = 0.8				
		distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
		
		pTx1=x+1.56*math.cos(theta+0.2897)
		pTx2=x+1.56*math.cos(theta-0.2897)
		pTy1=y+1.56*math.sin(theta+0.2897)
		pTy2=y+1.56*math.sin(theta-0.2897)
		
		if (not '2' in ex):
			if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x2,y2) or abs(math.sqrt(((x2-x)**2)+((y2-y)**2)))<0.95):
				choque.append('2')
				pC=True
		if (not '3' in ex):
			if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x3,y3) or abs(math.sqrt(((x3-x)**2)+((y3-y)**2)))<0.95):
				choque.append('3')
				pC=True
		if (not '4' in ex):
			if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x4,y4) or abs(math.sqrt(((x4-x)**2)+((y4-y)**2)))<0.95):
				choque.append('4')
				pC=True
		if (not '5' in ex):
			if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x5,y5) or abs(math.sqrt(((x5-x)**2)+((y5-y)**2)))<0.95):
				choque.append('5')
				pC=True
		if (not '6' in ex):
			if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x6,y6) or abs(math.sqrt(((x6-x)**2)+((y6-y)**2)))<0.95):
				choque.append('6')
				pC=True
		
		if (x2!=x2Ant or y2!=y2Ant):
			if (math.sqrt(((x-x2)**2)+((y-y2)**2))<1.8):
				p2=[x2-0.5*math.cos(theta2),y2-0.5*math.sin(theta2)]
				if theta2<0:
					theta2=2*math.pi+theta2
				q2=[x2+1.5*math.cos(theta2),y2+1.5*math.sin(theta2)]
				if (doIntersect(p1,q1,p2,q2)):
					conflicto.append('2')
					p=True
		if (x3!=x3Ant or y3!=y3Ant):
			if math.sqrt(((x-x3)**2)+((y-y3)**2))<1.8:
				p2=[x3-0.5*math.cos(theta3),y3-0.5*math.sin(theta3)]
				if theta3<0:
					theta3=2*math.pi+theta3
				q2=[x3+1.5*math.cos(theta3),y3+1.5*math.sin(theta3)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('3')
					p=True
		if (x4!=x4Ant or y4!=y4Ant):
			if math.sqrt(((x-x4)**2)+((y-y4)**2))<1.8:
				p2=[x4-0.5*math.cos(theta4),y4-0.5*math.sin(theta4)]
				if theta4<0:
					theta4=2*math.pi+theta4
				q2=[x4+1.5*math.cos(theta4),y4+1.5*math.sin(theta4)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('4')
					p=True
		if (x5!=x5Ant or y5!=y5Ant):
			if math.sqrt(((x-x5)**2)+((y-y5)**2))<1.8:
				p2=[x5-0.5*math.cos(theta5),y5-0.5*math.sin(theta5)]
				if theta5<0:
					theta5=2*math.pi+theta5
				q2=[x5+1.5*math.cos(theta5),y5+1.5*math.sin(theta5)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('5')
					p=True
		if (x6!=x6Ant or y6!=y6Ant):
			if math.sqrt(((x-x6)**2)+((y-y6)**2))<1.8:	
				p2=[x6-0.5*math.cos(theta6),y6-0.5*math.sin(theta6)]
				if theta6<0:
					theta6=2*math.pi+theta6
				q2=[x6+1.5*math.cos(theta6),y6+1.5*math.sin(theta6)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('6')
					p=True
			
		if pC:
			desvio2(choque,xgoal,ygoal)
			if xm!=xgoal or ym!=ygoal:
				return True	
		else:
			if p:
				desvio(conflicto,xgoal,ygoal)
				if xm!=xgoal or ym!=ygoal:
					return True	
		
		
		#print ('distance = ', distance)        
		linear_speed = kv * distance/3

		#ka = 4.0
		ka=0
		desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
		dtheta = desired_angle_goal-theta        
		#print ('dtheta = ', dtheta)
		angular_speed = ka * (dtheta)

		velocity_message.linear.x = linear_speed
		velocity_message.angular.z = angular_speed
		velocity_publisher.publish(velocity_message)
		#print ('x=', x, 'y=', y)
		
		if (contador<8):
			contador=contador+1
		else:
			contador=0
			x2Ant = x2
			y2Ant = y2
			x3Ant = x3
			y3Ant = y3
			x4Ant = x4
			y4Ant = y4
			x5Ant = x5
			y5Ant = y5
			x6Ant = x6
			y6Ant = y6

		if (distance < 0.1):
			time.sleep(0.3)
			break



def go_to_goal (xgoal, ygoal):
	global x
	global y
	global theta
	global x2
	global y2
	global x2Ant
	global y2Ant
	global theta2
	global x3
	global y3
	global x3Ant
	global y3Ant
	global theta3
	global x4
	global y4
	global x4Ant
	global y4Ant
	global theta4
	global x5
	global y5
	global x5Ant
	global y5Ant
	global theta5
	global x6
	global y6
	global x6Ant
	global y6Ant
	global theta6
	global contador
	
	
	velocity_message = Twist()
	cmd_vel_topic = '/turtle1/cmd_vel'

	while(True):
		
		conflicto=[]
		choque=[]
		p=False
		pC=False
		p1=[x,y]
		q1=[xgoal,ygoal]

		kv = 0.8		
		distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
		
		pTx1=x+1.56*math.cos(theta+0.2897)
		pTx2=x+1.56*math.cos(theta-0.2897)
		pTy1=y+1.56*math.sin(theta+0.2897)
		pTy2=y+1.56*math.sin(theta-0.2897)
		
		
		if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x2,y2) or abs(math.sqrt(((x2-x)**2)+((y2-y)**2)))<0.95):
			choque.append('2')
			pC=True
		if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x3,y3) or abs(math.sqrt(((x3-x)**2)+((y3-y)**2)))<0.95):
			choque.append('3')
			pC=True
		if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x4,y4) or abs(math.sqrt(((x4-x)**2)+((y4-y)**2)))<0.95):
			choque.append('4')
			pC=True
		if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x5,y5) or abs(math.sqrt(((x5-x)**2)+((y5-y)**2)))<0.95):
			choque.append('5')
			pC=True
		if (isInside(x,y,pTx1,pTy1,pTx2,pTy2,x6,y6) or abs(math.sqrt(((x6-x)**2)+((y6-y)**2)))<0.95):
			choque.append('6')
			pC=True
		
		if (x2!=x2Ant or y2!=y2Ant):
			if (math.sqrt(((x-x2)**2)+((y-y2)**2))<1.8):
				p2=[x2-0.5*math.cos(theta2),y2-0.5*math.sin(theta2)]
				if theta2<0:
					theta2=2*math.pi+theta2
				q2=[x2+1.5*math.cos(theta2),y2+1.5*math.sin(theta2)]
				if (doIntersect(p1,q1,p2,q2)):
					conflicto.append('2')
					p=True
		if (x3!=x3Ant or y3!=y3Ant):
			if math.sqrt(((x-x3)**2)+((y-y3)**2))<1.8:
				p2=[x3-0.5*math.cos(theta3),y3-0.5*math.sin(theta3)]
				if theta3<0:
					theta3=2*math.pi+theta3
				q2=[x3+1.5*math.cos(theta3),y3+1.5*math.sin(theta3)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('3')
					p=True
		if (x4!=x4Ant or y4!=y4Ant):
			if math.sqrt(((x-x4)**2)+((y-y4)**2))<1.8:
				p2=[x4-0.5*math.cos(theta4),y4-0.5*math.sin(theta4)]
				if theta4<0:
					theta4=2*math.pi+theta4
				q2=[x4+1.5*math.cos(theta4),y4+1.5*math.sin(theta4)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('4')
					p=True
		if (x5!=x5Ant or y5!=y5Ant):
			if math.sqrt(((x-x5)**2)+((y-y5)**2))<1.8:
				p2=[x5-0.5*math.cos(theta5),y5-0.5*math.sin(theta5)]
				if theta5<0:
					theta5=2*math.pi+theta5
				q2=[x5+1.5*math.cos(theta5),y5+1.5*math.sin(theta5)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('5')
					p=True
		if (x6!=x6Ant or y6!=y6Ant):
			if math.sqrt(((x-x6)**2)+((y-y6)**2))<1.8:	
				p2=[x6-0.5*math.cos(theta6),y6-0.5*math.sin(theta6)]
				if theta6<0:
					theta6=2*math.pi+theta6
				q2=[x6+1.5*math.cos(theta6),y6+1.5*math.sin(theta6)]
				if doIntersect(p1,q1,p2,q2):
					conflicto.append('6')
					p=True
		if pC:	
			desvio2(choque,xgoal,ygoal)
			if xm!=xgoal or ym!=ygoal:
				return True	
		else:
			if p:
				desvio(conflicto,xgoal,ygoal)
				if xm!=xgoal or ym!=ygoal:
					return True	
		
		#print ('distance = ', distance)        
		linear_speed = kv * distance/3

		#ka = 4.0
		ka=0
		desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
		dtheta = desired_angle_goal-theta        
		#print ('dtheta = ', dtheta)
		angular_speed = ka * (dtheta)

		velocity_message.linear.x = linear_speed
		velocity_message.angular.z = angular_speed
		velocity_publisher.publish(velocity_message)
		#print ('x=', x, 'y=', y)
		
		if (contador<8):
			contador=contador+1
		else:
			contador=0
			x2Ant = x2
			y2Ant = y2
			x3Ant = x3
			y3Ant = y3
			x4Ant = x4
			y4Ant = y4
			x5Ant = x5
			y5Ant = y5
			x6Ant = x6
			y6Ant = y6

		if (distance < 0.1):
			time.sleep(0.3)
			break

if __name__ == '__main__':
	try:

		rospy.init_node('turtlesim_motion_pose', anonymous = True)

		#Mi tortuga
		cmd_vel_topic = '/turtle1/cmd_vel' 
		velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
		position_topic1 = "/turtle1/pose"
		pose_subscriber1 = rospy.Subscriber(position_topic1, Pose, poseCallback1)
		#
		position_topic2 = "/turtle2/pose"
		pose_subscriber2 = rospy.Subscriber(position_topic2, Pose, poseCallback2)

		position_topic3 = "/turtle3/pose"
		pose_subscriber3 = rospy.Subscriber(position_topic3, Pose, poseCallback3)
		
		position_topic4 = "/turtle4/pose"
		pose_subscriber4 = rospy.Subscriber(position_topic4, Pose, poseCallback4)
		
		position_topic5 = "/turtle5/pose"
		pose_subscriber5 = rospy.Subscriber(position_topic5, Pose, poseCallback5)
		
		position_topic6 = "/turtle6/pose"
		pose_subscriber6 = rospy.Subscriber(position_topic6, Pose, poseCallback6)
			
		time.sleep(0.2)
		
		xm=3.0
		ym=10.0
		
		orientate(3.0,10.0)
		time.sleep(0.3)
		go_to_goal(3.0,10.0)
		time.sleep(0.3)	
		
		xm=10.0
		ym=3.0
		
		orientate(10.0,3.0)
		time.sleep(0.3)
		go_to_goal(10.0,3.0)
		time.sleep(0.3)
		
		xm=7.50
		ym=7.0
		
		orientate(7.5,7.0)
		time.sleep(0.3)
		go_to_goal(7.5,7.0)
		time.sleep(0.3)

	except rospy.ROSInterruptException:        
		pass
