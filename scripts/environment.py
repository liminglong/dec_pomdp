#!/usr/bin/env python
# coding=utf-8
# license removed for brevity

import rospy
from std_msgs.msg import String
import random
import matplotlib
import matplotlib.pyplot as plt
import sys
from dec_pomdp.msg import *
#10 seconds one step
#NODE_DICT = {}# a global variable, key is the node_ID in the graph, value is a node object

node_list = []
edge_list = []
jam_edge_list = []

#info_state_ID = [0, 1, 2]
INFOR_STATE_VALUE = [10, 20, 30]
INFOR_CHANGE_MAT = [[0.3, 0.3, 0.4],
                   [0.3, 0.3, 0.4],
                   [0.2, 0.2, 0.6]
                  ]
#TODO:把矩阵改成一个真正的马尔科夫随机变换矩阵，先不加ｔｈｒｅａｔ和ｊａｍ
#threat_state_ID = [0, 1, 2]
THREAT_STATE_VALUE = [5, 10, 15]
THREAT_CHANGE_MAT = [[0.2, 0.2, 0.6],
                     [0.3, 0.3, 0.4],
                     [0.5, 0.2, 0.3]
                    ]


#edge_jam_state = [0, 1, 2]
EDGE_JAM_STATE_VALUE = [1, 2, 3]
EDGE_JAM_CHANGE_MAT = [[0.2, 0.2, 0.6],
                       [0.3, 0.3, 0.4],
                       [0.5, 0.2, 0.3]
                      ]

class EDGE:
    def __init__(self, node_ID_0, node_ID_1, edge_ID, is_jam_edge, jam_state_ID=None, jam_edge_ID=None):
        global EDGE_JAM_STATE_VALUE
        if node_ID_0 > node_ID_1:
            print "Error! node_ID_0 should < node_ID_1."
            sys.exit(1)
        self._node_ID_0 = node_ID_0
        self._node_ID_1 = node_ID_1
        self._edge_ID = edge_ID
        self._is_jam_edge = is_jam_edge
        if jam_state_ID == None:
            self._jam_state_ID = -1
        else:
            self._jam_state_ID = jam_state_ID       
        if jam_edge_ID == None:
            self._jam_edge_ID = -1
        else:
           self._jam_edge_ID = jam_edge_ID
        
    def jam_change(self):
        global EDGE_JAM_STATE_VALUE
        global EDGE_JAM_CHANGE_MAT
        temp_ID = self._jam_state_ID
        a = random.uniform(0,1)
        b0 = EDGE_JAM_CHANGE_MAT[temp_ID][0]
        b1 = EDGE_JAM_CHANGE_MAT[temp_ID][1]
        #b2 = INFOR_CHANGE_MAT[temp_ID][2]
        #b0+b1+b2 =1
        if (a == 0 or (a > 0 and a < b0)):
            self._jam_state_ID=0
        elif (a == b0 or (a > b0 and a < (b0+b1))):
            self._jam_state_ID = 1
        else:
            self._jam_state_ID = 2
        return self._jam_state_ID  
    
    def get_jam(self):
        global EDGE_JAM_STATE_VALUE
        if self._is_jam_edge == False:
            print "Error, this is not a jam edge."
            sys.exit(1)
        else:
            return EDGE_JAM_STATE_VALUE[self._jam_state_ID]
        


class Node:
    def __init__(self, is_task, is_threat, pose_ID, infor_state_ID, threat_state_ID, task_ID = None):      
        global INFO_STATE_VALUE
        global THREAT_STATE_VALUE  
        self._pose_ID = pose_ID#节点的位置
        self._x = -1
        self._y = -1
        self._infor_state_ID = infor_state_ID#信息状态的ID号码,如果想获取信息状态的值，从infor_state_value中查找
        self._threat_state_ID = threat_state_ID#威胁状态的ＩＤ号码，如果想获取威胁状态的值，从threat_state_value中查找
        self._infor_state_value =  INFOR_STATE_VALUE[self._infor_state_ID]
        self._threat_state_value = THREAT_STATE_VALUE[self._threat_state_ID]       
        if task_ID == None:
            self._task_ID = -1
        else:
            self._task_ID = task_ID
        self._is_task = is_task
        
        self._is_threat = is_threat

    def check_is_task(self):
        return self._is_task

    def infor_change(self):
        global INFOR_STATE_VALUE
        global INFOR_CHANGE_MAT
        temp_ID = self._infor_state_ID
        a = random.uniform(0,1)
        b0 = INFOR_CHANGE_MAT[temp_ID][0]
        b1 = INFOR_CHANGE_MAT[temp_ID][1]
        #b2 = INFOR_CHANGE_MAT[temp_ID][2]
        #b0+b1+b2 =1
        if (a == 0 or (a > 0 and a < b0)):
            self._infor_state_ID=0
        elif (a == b0 or (a > b0 and a < (b0+b1))):
            self._infor_state_ID = 1
        else:
            self._infor_state_ID = 2
        return self._infor_state_ID        
    
    def get_infor(self):
        global INFOR_STATE_VALUE
        return INFOR_STATE_VALUE[self._infor_state_ID]
            
    def threat_change(self):
        global THREAT_STATE_VALUE
        global THREAT_CHANGE_MAT
        temp_ID = self._threat_state_ID
        a = random.uniform(0,1)
        b0 = THREAT_CHANGE_MAT[temp_ID][0]
        b1 = THREAT_CHANGE_MAT[temp_ID][1]

        if (a == 0 or (a > 0 and a < b0)):
            self._threat_state_ID=0
        elif (a == b0 or (a > b0 and a < (b0+b1))):
            self._threat_state_ID = 1
        else:
            self._threat_state_ID = 2
        return self._threat_state_ID            

    def get_threat(self):
        global THREAT_STATE_VALUE
        return THREAT_STATE_VALUE[self._threat_state_ID]
        
#牺牲空间来获取时间，地图环境的表示采用邻接矩阵实现
class Graph:
    def __init__(self,mat):   #初始化
        vnum = len(mat)
        for x in mat:
            if len(x) != vnum:
                raise ValueError("Argument for 'Graph'.")
        self._mat = [mat[i][:] for i in range(vnum)]      #使用拷贝的数据,copy line by line
        self._vnum = vnum

    def vertex_num(self):      #返回结点数目
        return self._vnum

    def invalID(self,v):      #检验输入的结点是否合法
        return v < 0 or v >= self._vnum

    def add_edge(self,vi,vj,val=1):   #增加边,此时边的val表示的是边的值。
        if self.invalID(vi) or self.invalID(vj):
            print (str(vi) + ' or' + str(vj) + 'is not a valID vertex.')
        if vi>vj:        
            print "Error! Vi > Vj. Vi should < Vj"
            sys.exit(1)
        self._mat[vi][vj] = val
        #self._mat[vj][vi] = val

    def set_edge(self,vi,vj,val):   #设置边,此时边的val表示的是边的索引号
        if self.invalID(vi) or self.invalID(vj):
            print (str(vi) + ' or' + str(vj) + 'is not a valID vertex.')
        if vi>vj:
            print "Error! Vi > Vj. Vi should < Vj"
            sys.exit(1)
        self._mat[vi][vj] = val
        #self._mat[vj][vi] = val

    def get_edge(self,vi,vj):   #得到边的信息
        if vi>vj:
            print "Error! Vi > Vj. Vi should < Vj"
            sys.exit(1)    
        if self.invalID(vi) or self.invalID(vj):
            raise GraphError(str(vi) + ' or ' + str(vj) + ' is not a valID vertex.')
        return self._mat[vi][vj]



def init():
    pass
'''
def talker():
    pub = rospy.Publisher('/environment', String, queue_size=10)
    rospy.init_node('environment', anonymous=True)
    rate = rospy.Rate(10) # 10hz
        
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
'''

def robot_random_move(current_node_ID):
    global edge_list
    temp_next_node_list = []
    for i in range(len(edge_list)):
        if edge_list[i]._node_ID_0 == current_node_ID:
            temp_next_node_list.append(edge_list[i]._node_ID_1)
        if edge_list[i]._node_ID_1 == current_node_ID:
            temp_next_node_list.append(edge_list[i]._node_ID_0)                
    next_node_ID_list = random.sample(temp_next_node_list, 1)
    next_node_ID = next_node_ID_list[0]
    return next_node_ID

def calc_fit_value(node_ID):
    global node_list
    global edge_list
    print "Come in the function calc_fit_value."
    temp_fit_value = 0
    if node_list[temp_robot_0_node_ID]._is_task == True:
        print "hello1"
        temp_fit_value+= node_list[temp_robot_0_node_ID].get_infor()
    if node_list[temp_robot_0_node_ID]._is_threat == True:
        print "hello2"
        temp_fit_value-= node_list[temp_robot_0_node_ID].get_threat()
    if not random_count == 0:
        a = pre_robot_0_node_ID
        b = temp_robot_0_node_ID
        if a > b:
            t = a
            a = b
            b = t
        temp_edge_ID = env.get_edge(i,j)
        #得到了编号，下面就是判断信息，加减sum_fit_value
        if edge_list[temp_edge_ID]._is_jam_edge == True:
            print "hello3"
            temp_fit_value -= edge_list[temp_edge_ID].get_jam()
    return temp_fit_value

#TODO: calc_fit_value这一块儿，需要做一个重新实现,初步计划的是采用类的继承的方式来实现,下午直接整体的重新写一遍代码。

if __name__ == '__main__':
    #global node_list
    #global edge_list
    #global jam_edge_list
    try:
        #rospy.init_node('environment_node', anonymous=True)        
        robot_0_initial_pose_x = 0
        robot_0_initial_pose_y = 0
        robot_1_initial_pose_x = 0
        robot_1_initial_pose_y = 0
        robot_2_initial_pose_x = 0
        robot_2_initial_pose_y = 0

        robot_0_initial_node_ID = 0
        robot_1_initial_node_ID = 0
        robot_2_initial_node_ID = 0
        
        temp_env = [[-1 for col in range(36)] for row in range(36)]
        env = Graph(temp_env)
        for i in range(36):
            node = Node(is_task=False, is_threat=False, pose_ID=i, infor_state_ID=0, threat_state_ID=0)
            node._x = i%6
            node._y = i/6
            node_list.append(node)
        

        #4个任务节点
        node_list[8]._is_task = True
        node_list[8]._task_ID = 0
        
        node_list[20]._is_task = True
        node_list[20]._task_ID = 1
                
        node_list[27]._is_task = True
        node_list[27]._task_ID = 2
        
        node_list[31]._is_task = True
        node_list[31]._task_ID = 3    
        #6个威胁节点
        node_list[9]._is_threat = True
        node_list[12]._is_threat = True
        node_list[21]._is_threat = True
        node_list[25]._is_threat = True
        node_list[28]._is_threat = True  
        
        

        for i in range(36):            
            if node_list[i]._is_task == True:
                if node_list[i]._infor_state_ID == 0:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=100, c='yellow',marker='o')
                if node_list[i]._infor_state_ID == 1:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=200, c='yellow',marker='o')
                if node_list[i]._infor_state_ID == 2:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=300, c='yellow',marker='o')                    
            elif node_list[i]._is_threat == True:
                if node_list[i]._threat_state_ID == 0:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=100, c = 'red', marker='o')
                if node_list[i]._threat_state_ID == 1:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=200, c = 'red', marker='o')
                if node_list[i]._threat_state_ID == 2:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=300, c = 'red', marker='o')                
                    
            else:
                plt.scatter(node_list[i]._x, node_list[i]._y, s=40, c='black',marker='o')
        
        #先添加横边
        for i in range(36):
            if not (i+1)%6==0:
                env.add_edge(i,i+1,1)
            else:
                continue        
        
        #再添加竖边
        for i in range(30):
            env.add_edge(i,i+6,1)
                               
        #去掉空缺边,把边的值置为０
        env.set_edge(4,10,-1)
        env.set_edge(9,15,-1)
        env.set_edge(13,14,-1)
        env.set_edge(15,21,-1)
        env.set_edge(18,24,-1)
        env.set_edge(20,26,-1)
        env.set_edge(21,27,-1)
        env.set_edge(22,28,-1)
        env.set_edge(27,33,-1)
        env.set_edge(28,34,-1)        
        
        #def __init__(self, node_ID_0, node_ID_1, edge_ID, jam_state_ID):
        #TODO:应该可以让ｍａｔ里面的有边的这一项存一个ｅｄｇｅ的编号，无边的这一项统一是（－１），这个ｅｄｇｅ的编号，对应的是ｅｄｇｅ_list里面的索引号。
        #
        #TODO:先把所有的边都添加进来，如果ｅｎｖ_mat中是１，从０开始分配编号，编号对应edge_list当中的
        #def __init__(self, node_ID_0, node_ID_1, edge_ID, is_jam_edge, jam_state_ID=None):
        temp_jam_edge_ID_list = []
        edge_ID_count = 0
        for i in range(36):
            for j in range(36):
                if env._mat[i][j] ==1:
                    temp_edge = EDGE(node_ID_0=i, node_ID_1 = j, edge_ID = edge_ID_count, is_jam_edge=False)
                    edge_list.append(temp_edge)
                    #TODO:把env_mat中对应的边的编号加进来
                    env.set_edge(i,j,edge_ID_count)
                    #env._mat[i][j] = edge_ID_count
                    edge_ID_count +=1
                    pass#TODO:先把所有的边都添加到ＥｄｇｅＬｉｓｔ当中来，如果ｅｎｖ_mat中是１，从０开始分配编号，编号对应edge_list当中的，如果是－１就不管，如果－１都不是，就垃圾了，报错。
        for i in range(len(edge_list)):
            if edge_list[i]._node_ID_0 == 3 and edge_list[i]._node_ID_1 == 4:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 0
                jam_edge_list.append(edge_list[i])
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)
            if edge_list[i]._node_ID_0 == 6 and edge_list[i]._node_ID_1 == 7:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 1 
                jam_edge_list.append(edge_list[i])
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)                
            if edge_list[i]._node_ID_0 == 14 and edge_list[i]._node_ID_1 == 15:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 2
                jam_edge_list.append(edge_list[i])
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)                
            if edge_list[i]._node_ID_0 == 18 and edge_list[i]._node_ID_1 == 19:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 3
                jam_edge_list.append(edge_list[i])
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)                
            if edge_list[i]._node_ID_0 == 21 and edge_list[i]._node_ID_1 == 22:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 4
                jam_edge_list.append(edge_list[i])
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)                
            if edge_list[i]._node_ID_0 == 32 and edge_list[i]._node_ID_1 == 33:
                edge_list[i]._is_jam_edge = True
                edge_list[i]._jam_state_ID = 0
                edge_list[i]._jam_edge_ID = 5
                jam_edge_list.append(edge_list[i])                                                                                     
                temp_jam_edge_ID_list.append(edge_list[i]._edge_ID)        
        '''        
        edge_0 = JAM_EDGE(node_ID_0 = 3, node_ID_1=4, edge_ID=0, jam_state_ID=0)
        edge_1 = JAM_EDGE(node_ID_0 = 6, node_ID_1=7, edge_ID=1, jam_state_ID=0)
        edge_2 = JAM_EDGE(node_ID_0 = 14, node_ID_1=15, edge_ID=2, jam_state_ID=0)
        edge_3 = JAM_EDGE(node_ID_0 = 18, node_ID_1=19, edge_ID=3, jam_state_ID=0)     
        edge_4 = JAM_EDGE(node_ID_0 = 21, node_ID_1=22, edge_ID=4, jam_state_ID=0)             
        edge_5 = JAM_EDGE(node_ID_0 = 32, node_ID_1=33, edge_ID=5, jam_state_ID=0)             
        
        jam_edge_list.append(edge_0)
        jam_edge_list.append(edge_1)
        jam_edge_list.append(edge_2)
        jam_edge_list.append(edge_3)
        jam_edge_list.append(edge_4)
        jam_edge_list.append(edge_5)        
        '''
                
        for i in range(len(edge_list)):
            if edge_list[i]._is_jam_edge == True:
                a = edge_list[i]._node_ID_0
                b = edge_list[i]._node_ID_1
                plt.plot([node_list[a]._x, node_list[b]._x], [node_list[a]._y, node_list[b]._y], color='red')
            else:
                a = edge_list[i]._node_ID_0
                b = edge_list[i]._node_ID_1
                plt.plot([node_list[a]._x, node_list[b]._x], [node_list[a]._y, node_list[b]._y], color='blue')
        '''
        for i in range(36):
            for j in range(36):
                if env._mat[i][j] ==1:
                    plt.plot([node_list[i]._x,node_list[j]._x] , [node_list[i]._y, node_list[j]._y], color='blue')
        '''                    
        #plt.axis('off') 
        plt.savefig("examples.jpg")
        
        #准备ROS环境
        pub_env = rospy.Publisher('/environment', Environment, queue_size=10)
        rospy.init_node('environment', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        #开始环境信息的动态变换和发布
        #以每十秒为一个消息发送的周期
        env_msg = Environment()
        
        state_cycle_count = 0
        move_cycle_count = 0
        unit_move_cycle = 10
        
        temp_robot_0_node_ID = robot_0_initial_node_ID
        temp_robot_1_node_ID = robot_1_initial_node_ID
        temp_robot_2_node_ID = robot_2_initial_node_ID
        
        fit_value_sum = 0
        random_count = 0#只是为了计算机器人随机游走的效应值而设定的。
        pre_robot_0_node_ID = 0
        pre_robot_1_node_ID = 1
        pre_robot_2_node_ID = 2
        while not rospy.is_shutdown():
            rate.sleep()    
            state_cycle_count +=1
            move_cycle_count +=1
            #10 seconds
            #print "hello3"
            if state_cycle_count == unit_move_cycle:
                #这部分会先进行随机算法的ｆｉｔｖａｌｕｅ值的计算
                #TODO:后面这段代码作为环境的时候，就可以把贪婪的部分注释掉
                #print "x and y of node_list[6]: "
                #print node_list[8].get_infor()
                #print node_list[9].get_threat()
                fit_value_sum += calc_fit_value(temp_robot_0_node_ID)
                fit_value_sum += calc_fit_value(temp_robot_1_node_ID)
                fit_value_sum += calc_fit_value(temp_robot_2_node_ID)
                print "temp_robot_0_node_ID: ",
                print temp_robot_0_node_ID
                print "temp_robot_1_node_ID: ",
                print temp_robot_1_node_ID
                print "temp_robot_2_node_ID: ",
                print temp_robot_2_node_ID
                
                
                random_count+=1    
                pre_robot_0_node_ID = temp_robot_0_node_ID
                pre_robot_1_node_ID = temp_robot_1_node_ID
                pre_robot_2_node_ID = temp_robot_2_node_ID
                
                temp_robot_0_node_ID = robot_random_move(temp_robot_0_node_ID)
                temp_robot_1_node_ID = robot_random_move(temp_robot_1_node_ID)
                temp_robot_2_node_ID = robot_random_move(temp_robot_2_node_ID)
                print "fit_value_sum: ",
                print fit_value_sum
                #TODO:做信息状态和威胁状态变换的操作
                t1 = rospy.get_time()
                for i in range(len(node_list)):
                    if node_list[i]._is_task:
                        node_list[i].infor_change()
                    if node_list[i]._is_threat:
                        node_list[i].threat_change()
                for i in range(len(jam_edge_list)):
                    if jam_edge_list[i]._is_jam_edge == False:
                        print "Error! There is normal edge in jam_edge_list!"
                        sys.exit(1)
                    else:
                        jam_edge_list[i].jam_change()
                #TODO 把消息存储并且发送出去，那边解析，
                t2 = rospy.get_time()
                env_msg.task_0_pose_ID = 8
                env_msg.task_0_infor_value = node_list[8].get_infor()
                env_msg.task_1_pose_ID = 20
                env_msg.task_1_infor_value = node_list[20].get_infor()   
                env_msg.task_2_pose_ID = 27
                env_msg.task_2_infor_value = node_list[27].get_infor()    
                env_msg.task_3_pose_ID = 31
                env_msg.task_3_infor_value = node_list[31].get_infor()
                
                env_msg.threat_0_pose_ID = 9
                env_msg.threat_0_value = node_list[9].get_threat()
                env_msg.threat_1_pose_ID = 12
                env_msg.threat_1_value = node_list[12].get_threat()
                env_msg.threat_2_pose_ID = 21
                env_msg.threat_2_value = node_list[21].get_threat()
                env_msg.threat_3_pose_ID = 25
                env_msg.threat_3_value = node_list[25].get_threat()       
                env_msg.threat_4_pose_ID = 28                         
                env_msg.threat_4_value = node_list[28].get_threat()
                
                env_msg.jam_edge_0_pose_ID = temp_jam_edge_ID_list[0]
                env_msg.jam_edge_0_value = edge_list[temp_jam_edge_ID_list[0]].get_jam()
                env_msg.jam_edge_1_pose_ID = temp_jam_edge_ID_list[1]
                env_msg.jam_edge_1_value = edge_list[temp_jam_edge_ID_list[1]].get_jam()                
                env_msg.jam_edge_2_pose_ID = temp_jam_edge_ID_list[2]
                env_msg.jam_edge_2_value = edge_list[temp_jam_edge_ID_list[2]].get_jam()                
                env_msg.jam_edge_3_pose_ID = temp_jam_edge_ID_list[3]
                env_msg.jam_edge_3_value = edge_list[temp_jam_edge_ID_list[3]].get_jam()
                env_msg.jam_edge_4_pose_ID = temp_jam_edge_ID_list[4]
                env_msg.jam_edge_4_value = edge_list[temp_jam_edge_ID_list[4]].get_jam()                
                env_msg.jam_edge_5_pose_ID = temp_jam_edge_ID_list[5]
                env_msg.jam_edge_5_value = edge_list[temp_jam_edge_ID_list[5]].get_jam()    
                           
                pub_env.publish(env_msg)                                                             
                #需要发送的环境内容：
                #1.任务节点的位置，一个任务ＩＤ对应着一个pose_ID就可以，四个任务节点，位置可以变化
                #2.任务节点的信息状态，应该会对应着任务节点的ＩＤ，以及任务节点的信息状态。
                #3.威胁节点的威胁状态，五个威胁节点，位置不可以变化
                #4.某些边的阻塞状态，边用两个node_ID来组成，对应一个阻塞状态的值。不是所有的边都是有阻塞状态的，一些边是比较畅通的，一些边是有阻塞状态的。

                #TODO:下一步的工作内容，直接把文章打开看；把需要观察的模型定好；到底动态模型是如何动态的，需要定好；把对环境已知的信息范围加入进来；
                #TODO:(1)可以开始写的代码，把每一帧的机器人位置也画出来，然后为每一帧的图片标明不同的编号，保存起来。
                #TODO:(2)可以开始写的代码，采取串行贪婪算法 
                
                
                

                
                
                print "time: ",
                print t2-t1                                                       
                state_cycle_count = 0 
            if move_cycle_count == 10*unit_move_cycle:
                #TODO：做目标动态移动一个单位的操作,这边预留了接口，但是初期设置的是目标不动态移动。
                pass
            
            #TODO:这里面还要干一件事，主要就是关于Baseline的事情。把随机的Ｂａｓｅｌｉｎｅ和贪心的Ｂａｓｅｌｉｎｅ都计算出来。这两个Ｂａｓｅｌｉｎｅ，可以指定机器人的数量和各自的初始位置，可以指定出执行任务的总时间，规划时间不用指定，贪心和随机几乎都是纯实时的规划时间。
            #TODO:随机就是每个机器人都随机地走一步；贪心就是算最近点的期望最大的点；
            #TODO:观测，如何进行观测呢?到某个点之后会观测到这个点的状态，以及与这个点相邻的边的状态。
            #TODO:之后，写出正式的POMDP的形式化建模。
            
        #TODO:先把随机的Baseline做出来，在每一个时间步，都标记好机器人的位置，然后每个时间步，机器人都会从当前的节点，随机地移动到一个相邻的边当中去，然后根据这几个机器人移动所产生的各个结果，算出最后的效应值，机器人之前没有信息共享。
        #plt.show()#这个函数是一个死循环                
        #talker()
    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
