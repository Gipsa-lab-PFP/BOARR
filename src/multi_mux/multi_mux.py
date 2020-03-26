#!/usr/bin/env python  
import rospy
from topic_tools.srv import MuxSelect
import copy
import subprocess

class MultiMux:
    def __init__(self):
        self.out_topics = rospy.get_param("~output_topics")
        self.out_prefix = rospy.get_param("~output_prefix", "")
        self.input1_prefix = rospy.get_param("~input1_prefix", "input1")
        self.input2_prefix = rospy.get_param("~input2_prefix", "input2")
        self.service_clients=[]
        for t in self.out_topics:
            print('rosrun topic_tools mux '+self.out_prefix+t+' '+self.input1_prefix+t+' '+self.input2_prefix+t+' mux:=mux_'+t)
            subprocess.Popen(['rosrun',
                              'topic_tools',
                              'mux',
                              self.out_prefix+t,
                              self.input1_prefix+'/'+t,
                              self.input2_prefix+'/'+t,
                              'mux:=mux_'+t])
            self.service_clients.append(rospy.ServiceProxy('mux_'+t+'/select', MuxSelect))
        
        self.cur_input = self.input1_prefix
        
        self.mux_select = rospy.Service("select", MuxSelect, self.select)
        
        
    def select(self, req):
        prev_input = copy.copy(self.cur_input)
        for i in range(len(self.service_clients)):
            try:
                self.service_clients[i](req.topic+'/'+self.out_topics[i])
            except rospy.ServiceException as exc:
                pass
        
            
        return prev_input;
    
if __name__ == '__main__':
    rospy.init_node('multi_mux')
    multi_mux = MultiMux()
    rospy.spin()

