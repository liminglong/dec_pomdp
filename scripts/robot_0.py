#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from dec_pomdp.msg import *

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

    def set_edge(self,vi,vj,val):   #设置边,此时边的val表示的是边的值。
        if self.invalID(vi) or self.invalID(vj):
            print (str(vi) + ' or' + str(vj) + 'is not a valID vertex.')
        if vi>vj:
            print "Error! Vi > Vj. Vi should < Vj"
            sys.exit(1)
        self._mat[vi][vj] = val
        #self._mat[vj][vi] = val

    def get_edge(self,vi,vj):   #得到边的信息
        if self._invalID(vi) or self._invalID(vj):
            raise GraphError(str(vi) + ' or ' + str(vj) + ' is not a valID vertex.')
        return self._mat[vi][vj]

def env_callback(env_msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    task_0_pose_ID = env_msg.task_0_pose_ID
    task_0_infor_value = env_msg.task_0_infor_value
    task_1_pose_ID = env_msg.task_1_pose_ID
    task_1_infor_value = env_msg.task_1_infor_value
    task_2_pose_ID = env_msg.task_2_pose_ID
    task_2_infor_value = env_msg.task_2_infor_value   
    task_3_pose_ID = env_msg.task_3_pose_ID
    task_3_infor_value = env_msg.task_3_infor_value
    
    threat_0_pose_ID = env_msg.threat_0_pose_ID
    threat_0_value = env_msg.threat_0_value
    threat_1_pose_ID = env_msg.threat_1_pose_ID
    threat_1_value = env_msg.threat_1_value
    threat_2_pose_ID = env_msg.threat_2_pose_ID
    threat_2_value = env_msg.threat_2_value
    threat_3_pose_ID = env_msg.threat_3_pose_ID
    threat_3_value = env_msg.threat_3_value
    threat_4_pose_ID = env_msg.threat_4_pose_ID
    threat_4_value = env_msg.threat_4_value

    jam_edge_0_pose_ID = env_msg.jam_edge_0_pose_ID
    jam_edge_0_value = env_msg.jam_edge_0_value
    jam_edge_1_pose_ID = env_msg.jam_edge_1_pose_ID
    jam_edge_1_value = env_msg.jam_edge_1_value
    jam_edge_2_pose_ID = env_msg.jam_edge_2_pose_ID
    jam_edge_2_value = env_msg.jam_edge_2_value
    jam_edge_3_pose_ID = env_msg.jam_edge_3_pose_ID
    jam_edge_3_value = env_msg.jam_edge_3_value
    jam_edge_4_pose_ID = env_msg.jam_edge_4_pose_ID
    jam_edge_4_value = env_msg.jam_edge_4_value
    jam_edge_5_pose_ID = env_msg.jam_edge_5_pose_ID
    jam_edge_5_value = env_msg.jam_edge_5_value

    #TODO:把数据解析出来
    node_list = []
    edge_list = []
    #jam_edge_list = []

    #以下的代码是把环境建立起来，包括把边建立起来
    temp_env = [[-1 for col in range(36)] for row in range(36)]
    env = Graph(temp_env)#把邻接矩阵的环境建立起来
    #把点加到node_list当中来
    for i in range(36):
        node = Node(is_task=False, is_threat=False, pose_ID=i, infor_state_ID=0, threat_state_ID=0)
        node._x = i%6
        node._y = i/6            
        node_list.append(node)
    #把边加到邻接矩阵的环境当中来
    #先添加横边
    for i in range(36):
        if not (i+1)%6==0:
            env.add_edge(i,i+1,1)                
        else:
            continue        

    #再添加竖边
    for i in range(30):
        env.add_edge(i,i+6,1)
        
    #去掉空缺边,把边的值置为-1，这样这个的env地图就完整了
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

    #把所有的边都添加到edge_list当中来
    edge_ID_count = 0
    for i in range(36):
        for j in range(36):
            if env._mat[i][j] ==1:
                temp_edge = EDGE(node_ID_0=i, node_ID_1 = j, edge_ID = edge_ID_count, is_jam_edge=False)
                edge_list.append(temp_edge)
                edge_ID_count +=1
    
                
    #TODO:接下来通过ROS收到的消息，把中间有价值的内容解析出来
    #TODO:中间有价值的内容，可以通过按照索引号访问等方式，把有价值的内容提取出来。
                
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/environment", Environment, env_callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    #TODO:关于代码流程。
    #1.把消息编译成功,并且成功的import进来;2.Subscriber函数可以成功地解析消息，并且可以根据对环境的统一认知，那个矩阵，知道各个机器人的位置等等；完成上面的两步暂时算是完成了第一步。
    rospy.spin()

if __name__ == '__main__':
    listener()
