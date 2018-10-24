#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from dec_pomdp.msg import *
                  
INFOR_STATE_VALUE = [1, 10, 100, 1000]
INFOR_CHANGE_MAT = [[0.8, 0.1, 0.1, 0.0],
                    [0.4, 0.5, 0.0, 0.1],
                    [0.2, 0.1, 0.6, 0.1],
                    [0.0, 0.0, 0.4, 0.6]]                  
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

def env_callback(env_msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print "env_callback start!"
    task_0_pose_ID = env_msg.task_0_pose_ID
    task_0_infor_value = env_msg.task_0_infor_value
    task_1_pose_ID = env_msg.task_1_pose_ID
    task_1_infor_value = env_msg.task_1_infor_value
    task_2_pose_ID = env_msg.task_2_pose_ID
    task_2_infor_value = env_msg.task_2_infor_value   
    task_3_pose_ID = env_msg.task_3_pose_ID
    task_3_infor_value = env_msg.task_3_infor_value

    #TODO:把数据解析出来
    node_list = []
    edge_list = []
    #jam_edge_list = []

    #以下的代码是把环境建立起来，包括把边建立起来
    temp_env = [[-1 for col in range(36)] for row in range(36)]
    env = Graph(temp_env)
    for i in range(36):
        node = Node(is_task=False, pose_ID=i, infor_state_ID=0)
        node._x = i%6
        node._y = i/6
        node_list.append(node)

    #多个任务节点的位置，对于所有的机器人都是提前知道的，4个任务节点,在这段代码里可以随意地加入任务节点。
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
        env.add_edge(i,i+6,1)
    #把已经添加在ｍａｔ矩阵当中的边（也就是矩阵中值为１的边），都添加到edge_list当中。
    edge_ID_count = 0
    for i in range(36):
        for j in range(36):
            if env._mat[i][j] ==1:
                temp_edge = EDGE(node_ID_0=i, node_ID_1 = j, edge_ID = edge_ID_count)
                edge_list.append(temp_edge)
                env.set_edge(i,j,edge_ID_count)
                edge_ID_count +=1
                pass
    print "hello 2"
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/environment", Environment, env_callback)
    rospy.spin()
if __name__ == '__main__':
    listener()
