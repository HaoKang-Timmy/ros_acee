#!/usr/bin/python3
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan, PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt


## TODO import your own planner

class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_grid_size = 0.3
        self.plan_robot_radius = 0.6
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []
        self.map_data = []
        # count to update map
        self.map_count = 0

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber('/course_agv/goal', PoseStamped, self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan', Plan, self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path', Path, queue_size=1)
        self.map_sub = rospy.Subscriber('/slam_map', OccupancyGrid, self.mapCallback)
        self.updateMap()
        # self.updateGlobalPose()

        pass

    def goalCallback(self, msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0)
        pass

    def collisionCallback(self, msg):
        self.replan(0)

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans, self.rot) = self.tf.lookupTransform('/map', '/robot_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def get_mypath(self, x, y, mygx, mygy, map_data, exists):
        lenx = np.size(map_data, 0)
        leny = np.size(map_data, 1)
        print((x, y, mygx, mygy))
        maph=np.zeros((lenx,leny))
        mapg=np.zeros((lenx,leny))
        mapf=np.zeros((lenx,leny))
        mapo=np.zeros((lenx,leny))
        mappx=np.zeros((lenx,leny))
        mappy=np.zeros((lenx,leny))
        openlistheap=[]
        openlistlocal=[]    
        mapo[int(x),int(y)]=1
        openlistheap.append(((x-mygx)**2+(y-mygy)**2)**0.5)
        openlistlocal.append((x,y))
        k=0
        def findmin(l):
            loc=0
            for i in range(1,len(l)):
                if(l[i]<l[loc]):
                    loc=i
            return loc
        def mfind(l,o):
            for i in range(0,len(l)):
                if(l[i][0]==o[0] and l[i][1]==o[1]):
                    return i
            return -1
        while(1):
            #print(1)
            loc=findmin(openlistheap)
            print(loc)
            if(len(openlistheap)!=len(openlistlocal)):
                print("eroooooooooooooooooooooooooooooor")
                break        
            nowx,nowy=openlistlocal[loc]
            mapo[int(nowx),int(nowy)]=1    
            #print((nowx,nowy))
            #print(openlistlocal)        
            del openlistheap[loc]
            del openlistlocal[loc]
            can=[-1,0,1]
            find=0
            print(k)
            k=k+1    
            for i in range(0,3):
                for j in range(0,3):
                    
                    if (nowx + can[i] >= 0 and nowx + can[i] <= lenx and nowy + can[j] >= 0 and nowy + can[
                                j] <= leny and map_data[int(nowx + can[i]), int(nowy + can[j])] <= 99 and exists[
                                int(nowx + can[i]), int(nowy + can[j])] == 0 and (i!=0 or j!=0)):
                        if(mapo[int(nowx + can[i]),int(nowy + can[j])]==0):
                            index=mfind(openlistlocal,(nowx + can[i],nowy + can[j]) )
                            if(index==-1):

                                openlistlocal.append((int(nowx + can[i]),int(nowy + can[j])))
                                
                                mappx[int(nowx + can[i]),int(nowy + can[j])]=nowx
                                mappy[int(nowx + can[i]),int(nowy + can[j])]=nowy
                                
                                if(int(nowx + can[i])==mygx and int(nowy + can[j])==mygy):
                                    find=1
                                    break
                                                        
                                if(can[i]*can[j]==0):                            
                                    mapg[int(nowx + can[i]),int(nowy + can[j])]=mapg[int(nowx),int(nowy)]+1
                                else:
                                    mapg[int(nowx + can[i]),int(nowy + can[j])]=mapg[int(nowx),int(nowy)]+2**0.5

                                maph[int(nowx + can[i]),int(nowy + can[j])]=((int(nowx + can[i])-mygx)**2+(int(nowy + can[j])-mygy)**2)**0.5
                                mapf[int(nowx + can[i]),int(nowy + can[j])]=maph[int(nowx + can[i]),int(nowy + can[j])]+mapg[int(nowx + can[i]),int(nowy + can[j])]
                                openlistheap.append(mapf[int(nowx + can[i]),int(nowy + can[j])])
                            else:
                                
                                mappx[int(nowx + can[i]),int(nowy + can[j])]=nowx
                                mappy[int(nowx + can[i]),int(nowy + can[j])]=nowy
                                
                                if(int(nowx + can[i])==mygx and int(nowy + can[j])==mygy):
                                    find=1
                                    break
                                                        
                                if(can[i]*can[j]==0):                            
                                    mapg[int(nowx + can[i]),int(nowy + can[j])]=mapg[int(nowx),int(nowy)]+1
                                else:
                                    mapg[int(nowx + can[i]),int(nowy + can[j])]=mapg[int(nowx),int(nowy)]+2**0.5

                                maph[int(nowx + can[i]),int(nowy + can[j])]=((int(nowx + can[i])-mygx)**2+(int(nowy + can[j])-mygy)**2)**0.5
                                mapf[int(nowx + can[i]),int(nowy + can[j])]=maph[int(nowx + can[i]),int(nowy + can[j])]+mapg[int(nowx + can[i]),int(nowy + can[j])]
                                openlistheap[index]=mapf[int(nowx + can[i]),int(nowy + can[j])]
                if(find==1):
                    break
            if(find==1):
                break
        print("success!")
        self.plan_rx.append(mygx)
        self.plan_ry.append(mygy)
        nowx=mygx
        nowy=mygy
        
        while(1):
            
            print((nowx,nowy))
            temnowx=mappx[int(nowx),int(nowy)]
            temnowy=mappy[int(nowx),int(nowy)]
            nowx=temnowx
            nowy=temnowy        
            self.plan_rx.append(nowx)
            self.plan_ry.append(nowy)
            if(nowx==x and nowy==y):
                break
            print(self.plan_rx)
            print(self.plan_ry)

    def replan(self, req):
        print('get request for replan!!!!!!!!')
        map_data = np.array(self.map.data).reshape((-1, self.map.info.height)).transpose()
        ran=int((self.plan_robot_radius)//self.map.info.resolution)
        print(ran)
        lenx = np.size(map_data, 0)
        leny = np.size(map_data, 1)
        temarry=np.zeros((lenx,leny))
        for i in range(lenx):
            for j in range(leny):
                temarry[i,j]=map_data[i,j]
        for i in range(lenx):
            for j in range(leny):
                for k in range(-ran,ran+1):
                    for l in range(-ran,ran+1):
                        if(i+k>=0 and i+k<lenx and j+l>=0 and j+l<leny):
                            if(map_data[i+k,j+l]>=50):
                                temarry[i,j]=100
        for i in range(lenx):
            for j in range(leny):
                map_data[i,j]=temarry[i,j]
        self.updateGlobalPose()
        ## TODO get planner result
        ## e.g. self.plan_rx,self.plan_ry = self.planner.planning(self.plan_sx,self.plan_sy,self.plan_gx,self.plan_gy)
        self.plan_rx = []
        self.plan_ry = []
        exists = np.zeros((lenx, leny), dtype=float)
        mygx = (self.plan_gx - self.map.info.origin.position.x) // self.map.info.resolution
        mygy = (self.plan_gy - self.map.info.origin.position.y) // self.map.info.resolution
        mysx = (self.plan_sx - self.map.info.origin.position.x) // self.map.info.resolution
        mysy = (self.plan_sy - self.map.info.origin.position.y) // self.map.info.resolution
        self.get_mypath(mysx, mysy, mygx, mygy, map_data, exists)
        for i in range(len(self.plan_rx)):
            self.plan_rx[i] = self.plan_rx[i] * self.map.info.resolution + self.map.info.origin.position.x
            self.plan_ry[i] = self.plan_ry[i] * self.map.info.resolution + self.map.info.origin.position.y
        ##
        self.publishPath()
        res = True
        return PlanResponse(res)

    def mapCallback(self, msg):

        self.map = msg
        self.map_data = np.array(self.map.data).reshape((-1, self.map.info.height)).transpose()
        print(self.map_data)
        pass

    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map', GetMap)
            msg = getMap().map
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s' % e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx) - 1 - i]
            pose.pose.position.y = self.plan_ry[len(self.plan_rx) - 1 - i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass


if __name__ == '__main__':
    main()
