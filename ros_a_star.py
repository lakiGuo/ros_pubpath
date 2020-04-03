import math
import matplotlib.pyplot as plt
import numpy as np
#reference: https://github.com/AtsushiSakai/PythonRobotics
#via Topic in ROS
# sub:"/map"
#       "/initialpose"
#       "/move_base_simple/goal"
# pub: ("/path_my_A_star")
#reso:0.1
#origin: 
class Astar():
    #start,goal,input,World_2_Map
    def __init__(self,map1,reso,rr,start,goal,minx,miny):
        #self.map1=self.map_extend(map1)     #ros unknown:-1,obstacle:100,path:1
	self.map1=map1
        self.state_map = np.zeros([len(map1) + 2, len(map1[0]) + 2])#
	self.xwidth=len(map1[0])
	print self.xwidth
	self.height=len(map1)
	print self.height
        self.reso=reso 
        self.rr=rr # real_size/reso 
        self.start=start  
        self.goal=goal
        self.motion=self.get_motion_model()
        #origin
        self.minx=minx
        self.miny=miny
        self.maxx=1000
        self.maxy=1000
        
    def map_extend(self,map1):
        new_row = np.ones(len(map1[0])) 
        new_col = np.ones(len(map1) + 2)
        x = np.insert(map1, 0, new_row, axis=0)
        x = np.insert(x, len(map1) + 1, new_row, axis=0)
        x = np.insert(x, 0 , new_col, axis=1)
        x = np.insert(x, len(map1[0]) + 1 , new_col, axis=1)
        return x
        
    def get_motion_model(self):
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion
    
    class Map_Node():
        #pind record the parent node
        def __init__(self,x,y,cost,pind):
            self.x=x
            self.y=y
            self.cost=cost
            self.pind=pind

        def  __str__(self):
            return  str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)            
    
    
    
    def find_path(self):
        #in the map cordinate
        nstart=self.Map_Node(self.World_2_Map(self.start[0],self.minx),self.World_2_Map(self.start[1],self.miny),0.0,-1)
	print 'nstart in map is: ',nstart
        ngoal=self.Map_Node(self.World_2_Map(self.goal[0],self.minx),self.World_2_Map(self.goal[1],self.miny),0.0,-1)
	print 'ngoal in map is: ',ngoal
        open_set={}
        closed_set={}
        open_set[self.calc_grid_index(nstart)]=nstart
        
        
        while True:
            if len(open_set)==0:
                print('open_set is empty')
                break
                
            c_id=min(open_set,key=lambda o: open_set[o].cost+ self.calc_heuristic(ngoal,open_set[o]))
            current=open_set[c_id]         
            #print "c_id is:",c_id
	    #print "current node is :",current

            if current.x==ngoal.x and current.y==ngoal.y:
                print("find final-path")
                ngoal.pind=current.pind
                ngoal.cost=current.cost
                break
            
            
            del open_set[c_id]
	    #print "del ok"
            
            closed_set[c_id]=current
            
            for i,_ in enumerate(self.motion):
                node=self.Map_Node(current.x+self.motion[i][0],
                              current.y+self.motion[i][1],
                              current.cost+self.motion[i][2],
                              c_id)
            	print self.map1[int(node.y)][int(node.x)]
		#print "node in loop is :",node
                n_id=self.calc_grid_index(node)
                
                if not self.verify_node(node):
                    continue
                
                if n_id in closed_set:
                    continue
                
                if n_id not in open_set:
                    open_set[n_id]=node
                else:
                    if node.cost<open_set[n_id].cost:
                        open_set[n_id]=node
        path=self.calc_final_path(ngoal,closed_set)
	print "path ok"
	return path
                    
    def calc_final_path(self, ngoal, closedset):
        #rx,ry=[calc_map2world(ngoal.x,self.minx)],[calc_map2world(ngoal.y,self.miny)]  
        path=[[self.calc_map2world(ngoal.x,self.minx),self.calc_map2world(ngoal.y,self.miny)]]
        pind=ngoal.pind
	print "ngoal.pind is:",ngoal.pind
        while pind != -1:
            n=closedset[pind]
	    print 'now n is:',n
            #rx.append(calc_map2world(closedset.x,self.minx)) 
            #ry.append(calc_map2world(closedset.y,self.miny)) 
            path.append([self.calc_map2world(n.x,self.minx),self.calc_map2world(n.y,self.miny)])
            pind=n.pind
    	print "already find!"
        return path
    
    def calc_map2world(self,m_p,minp):
        w_p=m_p*self.reso+minp
        
        return w_p
    
             
    def calc_heuristic(self,ngoal,map_node):
        w=10
        h= w*math.hypot(map_node.x-ngoal.x,map_node.y-ngoal.y)
        return h
        
            
    def calc_grid_index(self,node):
        return node.y*self.xwidth+node.x
    
    def World_2_Map(self,pos,min_p):
        return round((pos-min_p)/self.reso)
   
    def calc_omap(self):
	ox,oy=[],[]
	self.omap=[[False for i in range(self.xwidth)] for i in range(self.height)]
	for i in range(self.height):
		for j in range(self.xwidth):
			if self.map1[i][j]==100:
				self.omap[i][j]=True
				ox.append(j)
				oy.append(i)
	print zip(ox,oy)
	
	
	#for i in range(self.height):
	#	for j in range(self.xwidth):
	#		for iox,ioy in zip(ox,oy):
	#			d=math.hypot(iox-i,ioy-j)
	#			if d<=self.rr:
	#				self.omap[i][j]=True

    def verify_node(self, node):
        #px = self.calc_map2world(node.x, self.minx)
        #py = self.calc_map2world(node.y, self.miny)
	#self.maxx=self.World_2_Map(self.xwidth,self.minx)
	#self.maxy=self.World_2_Map(self.height,self.miny)
	print self.map1[int(node.y)][int(node.x)]
        #if px < self.minx:
        #    return False
        #elif py < self.miny:
        #    return False
        #elif node.y >= self.height:
        #    return False
        #elif node.x >= self.xwidth:
        #    return False  
	#self.calc_omap()
	if self.map1[int(node.y)][int(node.x)]!=0:
	    return False 
	if self.map1[int(node.y)-self.rr][int(node.x)]!=0:
	    return False
	if self.map1[int(node.y)+self.rr][int(node.x)]!=0:
	    return False
	if self.map1[int(node.y)][int(node.x)-self.rr]!=0:
	    return False
	if self.map1[int(node.y)][int(node.x)+self.rr]!=0:
	    return False
	return True

    #def calc_omap(self):
    #self.map=
    
    
        
