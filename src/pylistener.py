#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

# Import required Python code.
import pdb 
import sys
import rospy
import ratslam.posecell_networkv
from nav_msgs.msg import Odometry
from node_example.msg import TopologicalAction
from node_example.msg import ViewTemplate
# Import custom message data.
from node_example.msg import NodeExampleData
counter = 0
timo=0
prev_time=0
pc_output=TopologicalAction()
topic_root='irat_red'
pose_pc_robot=open("poses_pc_robot.txt","w")
def callback(odom_data,pc_args_1):
    #pdb.set_trace()
    pc=pc_args_1[0]
    pub_pc=pc_args_1[1]
    '''
    Callback function for the subscriber.
    '''
    # Simply print out values in our custom message
    #rospy.loginfo(rospy.get_name() + " I heard %s", data.message +
     #             ", a + b = %d" % (data.a + data.b))
    global prev_time
    if (prev_time>0):
        global counter
        pc_output.src_id=pc.get_current_exp_id()
        time_diff_robot=odom_data.header.stamp.to_sec()-prev_time
        pc.on_odo(odom_data.twist.twist.linear.x, odom_data.twist.twist.angular.z, time_diff_robot);
        print "kkkkkkkkkkkkk",time_diff_robot
        temp_action=pc.get_action()
        pc_output.action=temp_action.value
        print "eeeeeeeeeeee",pc_output.action
        if pc_output.action!=0:
            
            print "000000000000000000000000",pc_output.dest_id
            pc_output.header.stamp=rospy.Time.now()
            pc_output.header.seq+=1
            pc_output.dest_id=pc.get_current_exp_id()
            pc_output.relative_rad=pc.get_relative_rad()
            pub_pc.publish(pc_output)
            print(rospy.Time.now().to_sec(),pc_output.header.seq,pc_output.action, pc_output.src_id,pc_output.dest_id)
    prev_time=odom_data.header.stamp.to_sec()
    print "999999999999999999",pc.best_x

    pose_pc_robot.write(str(pc.best_x)+" "+str(pc.best_y)+" "+str(pc.best_th)+"\n")
def template_callback(vt,arg_template):
    pc=arg_template[0]
    pub_pc=arg_template[1]
    pc.on_view_template(vt.current_id,vt.relative_rad)
        

def listener():
    '''
    Main function.
    '''
    # Create a subscriber with appropriate topic, custom message and name of
    # callback function.
    
    pc=ratslam.posecell_networkv.PosecellNetwork()
    #slam = ratslam.Ratslam()
    pub_pc=rospy.Publisher("/irat_red/PoseCell/TopologicalAction",TopologicalAction,queue_size=0)
    rospy.Subscriber("/irat_red/odom",Odometry,callback,(pc,pub_pc))
    
    rospy.Subscriber('/irat_red/LocalView/Template',ViewTemplate,template_callback,(pc,pub_pc))
    
      #ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, pc, &pub_pc), ros::VoidConstPtr(),
       #                                                             ros::TransportHints().tcpNoDelay());

    
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('RatSLAMPoseCells')
    # Go to the main loop.
    listener()
