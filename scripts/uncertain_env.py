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
'''
INFOR_STATE_VALUE = [10, 20, 30]
INFOR_CHANGE_MAT = [[0.3, 0.3, 0.4],
                   [0.3, 0.3, 0.4],
                   [0.2, 0.2, 0.6]
                  ]
'''    

INFOR_STATE_VALUE = [1, 10, 100, 1000]
INFOR_CHANGE_MAT = [[0.8, 0.1, 0.1, 0.0],
                    [0.4, 0.5, 0.0, 0.1],
                    [0.2, 0.1, 0.6, 0.1],
                    [0.0, 0.0, 0.4, 0.6]]

class EDGE:
    def __init__(self, node_ID_0, node_ID_1, edge_ID):
        global EDGE_JAM_STATE_VALUE
        if node_ID_0 > node_ID_1:
            print "Error! node_ID_0 should < node_ID_1."
            sys.exit(1)
        self._node_ID_0 = node_ID_0
        self._node_ID_1 = node_ID_1
        self._edge_ID = edge_ID    

class Node:
    def __init__(self, is_task, pose_ID, infor_state_ID, task_ID = None):      
        global INFO_STATE_VALUE
        self._pose_ID = pose_ID#节点的位置
        self._x = -1
        self._y = -1
        self._infor_state_ID = infor_state_ID#信息状态的ID号码,如果想获取信息状态的值，从infor_state_value中查找
        self._infor_state_value =  INFOR_STATE_VALUE[self._infor_state_ID]   
        if task_ID == None:
            self._task_ID = -1
        else:
            self._task_ID = task_ID
        self._is_task = is_task

    def check_is_task(self):
        return self._is_task

    def infor_change(self):
        global INFOR_STATE_VALUE
        global INFOR_CHANGE_MAT
        temp_ID = self._infor_state_ID
        a = random.uniform(0,1)
        b0 = INFOR_CHANGE_MAT[temp_ID][0]
        b1 = INFOR_CHANGE_MAT[temp_ID][1]
        b2 = INFOR_CHANGE_MAT[temp_ID][2]
        #b2 = INFOR_CHANGE_MAT[temp_ID][2]
        #b0+b1+b2 =1
        if (a == 0 or (a > 0 and a < b0)):
            self._infor_state_ID=0
        elif (a == b0 or (a > b0 and a < (b0+b1))):
            self._infor_state_ID = 1
        elif (a == b0+b1 or (a > (b0+b1) and a < (b0+ b1+ b2))):
            self._infor_state_ID = 2
        else:
            self._infor_state_ID = 3
        return self._infor_state_ID
    
    def get_infor(self):
        global INFOR_STATE_VALUE
        return INFOR_STATE_VALUE[self._infor_state_ID]

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

if __name__ == '__main__':
    try:
        robot_0_initial_node_ID = 0
        robot_1_initial_node_ID = 0
        robot_2_initial_node_ID = 0

        temp_env = [[-1 for col in range(36)] for row in range(36)]
        env = Graph(temp_env)
        for i in range(36):
            node = Node(is_task=False, pose_ID=i, infor_state_ID=0)
            node._x = i%6
            node._y = i/6
            node_list.append(node)

        #4个任务节点,在这段代码里可以随意地加入任务节点。
        node_list[8]._is_task = True
        node_list[8]._task_ID = 0
        
        node_list[20]._is_task = True
        node_list[20]._task_ID = 1
                
        node_list[27]._is_task = True
        node_list[27]._task_ID = 2
        
        node_list[31]._is_task = True
        node_list[31]._task_ID = 3                   

        #先在ｍａｔ矩阵中添加横边
        for i in range(36):
            if not (i+1)%6==0:
                env.add_edge(i,i+1,1)
            else:
                continue        
        
        #再在ｍａｔ矩阵中添加竖边
        for i in range(30):
            env.add_edge(i,i+6,1)#把已经添加在ｍａｔ矩阵当中的边（也就是矩阵中值为１的边），都添加到edge_list当中。
        edge_ID_count = 0
        for i in range(36):
            for j in range(36):
                if env._mat[i][j] ==1:
                    temp_edge = EDGE(node_ID_0=i, node_ID_1 = j, edge_ID = edge_ID_count)
                    edge_list.append(temp_edge)
                    env.set_edge(i,j,edge_ID_count)
                    edge_ID_count +=1
                    pass
        
        #准备ROS环境
        pub_env = rospy.Publisher('/environment', Environment, queue_size=10)
        rospy.init_node('environment', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        env_msg = Environment()
        
        state_cycle_count = 0
        #unit_move_cycle = 10
        
        temp_robot_0_node_ID = robot_0_initial_node_ID
        temp_robot_1_node_ID = robot_1_initial_node_ID
        temp_robot_2_node_ID = robot_2_initial_node_ID
        
        fit_value_sum = 0
        while not rospy.is_shutdown():
            rate.sleep()
            state_cycle_count += 1
            #做信息状态变换的操作
            t1 = rospy.get_time()
            for i in range(len(node_list)):
                if node_list[i]._is_task:
                    node_list[i].infor_change()
            t2 = rospy.get_time()
            print "hello 1"
            for i in range(36):
                if node_list[i]._is_task == True:
                    if node_list[i]._infor_state_ID == 0:
                        plt.scatter(node_list[i]._x, node_list[i]._y, s=40, c='yellow',marker='o')
                    if node_list[i]._infor_state_ID == 1:
                        plt.scatter(node_list[i]._x, node_list[i]._y, s=200, c='yellow',marker='o')
                    if node_list[i]._infor_state_ID == 2:
                        plt.scatter(node_list[i]._x, node_list[i]._y, s=400, c='yellow',marker='o')
                    if node_list[i]._infor_state_ID == 3:
                        plt.scatter(node_list[i]._x, node_list[i]._y, s=800, c='yellow',marker='o')
                else:
                    plt.scatter(node_list[i]._x, node_list[i]._y, s=40, c='black',marker='o')            
            for i in range(len(edge_list)):
                a = edge_list[i]._node_ID_0
                b = edge_list[i]._node_ID_1
                plt.plot([node_list[a]._x, node_list[b]._x], [node_list[a]._y, node_list[b]._y], color='blue')
            plt.savefig("examples.jpg")
            print "hello 2"
            env_msg.task_0_pose_ID = 8
            env_msg.task_0_infor_value = node_list[8].get_infor()
            env_msg.task_1_pose_ID = 20
            env_msg.task_1_infor_value = node_list[20].get_infor()   
            env_msg.task_2_pose_ID = 27
            env_msg.task_2_infor_value = node_list[27].get_infor()    
            env_msg.task_3_pose_ID = 31
            env_msg.task_3_infor_value = node_list[31].get_infor()

            pub_env.publish(env_msg)
            #break
            print "time: ",
            print t2-t1
    except rospy.ROSInterruptException:
        pass
