#!/usr/bin/env python
import roslib; roslib.load_manifest('awesomeo_controller')
import rospy
import wx
import kinematics_msgs
import kinematics_msgs.srv
import orrosplanning.srv
from sensor_msgs.msg import JointState
from ax12_driver_core.msg import *
from threading import Thread
#from arm_kinematics.srv import *

RANGE=10000

indices = None
seed = None
joint_names = None
pub = None

hack_joint_names = ["1.0:base_to_right_shoulder",
                    "2.0:base_to_left_shoulder",
                    "3.0:right_shoulder_to_upper",
                    "4.0:left_shoulder_to_upper",
                    "5.0:right_elbow",
                    "6.0:left_elbow",
                    "7.0:base_to_right_hip",
                    "8.0:base_to_left_hip",
                    "9.0:right_hip_to_split",
                    "10.0:left_hip_to_split",
                    "11.0:right_hip_split_to_upper",
                    "12.0:left_hip_split_to_upper",
                    "13.0:right_knee",
                    "14.0:left_knee",
                    "15.0:right_ankle",
                    "16.0:left_ankle",
                    "17.0:right_ankle_roll",
                    "18.0:left_ankle_roll"]


initial = {'x':0.03, 'y':0.0, 'z':-0.215}

def process_motor_states(msg):
    joint_state = kinematics_msgs.srv.GetPositionFKRequest().robot_state.joint_state

    if(joint_names is None):
        return
    if pub is None:
        return

    joint_state.name = hack_joint_names

    # Prefill with dummy values incase we miss a packet
    vals = [0.0 for x in range(len(hack_joint_names))]

    # Process all the states given to us by the ax12 node
    for m in msg.motor_states:
        angle = m.position/float(1024)
        angle -= 0.5
        angle *= 3.14159
        vals[m.id-1] = angle

    joint_state.position = vals
        
    pub.publish(joint_state)
        
from std_msgs.msg import String
def controller(x, y, z):
    global seed
    global indices
    global joint_names
    global pub
    pub = rospy.Publisher('tom_states', JointState)
    rospy.init_node('controller')
	
    get_ik_solver_info = rospy.ServiceProxy(
                            'arm_kinematics/get_ik_solver_info', 
                            kinematics_msgs.srv.GetKinematicSolverInfo)
    #get_position_ik = rospy.ServiceProxy('arm_kinematics/get_ik', 
    #                        kinematics_msgs.srv.GetPositionIK)
    rospy.wait_for_service("GetPositionIK")
    get_position_ik = rospy.ServiceProxy('GetPositionIK', 
                            kinematics_msgs.srv.GetPositionIK)
    get_position_fk = rospy.ServiceProxy('arm_kinematics/get_fk', 
                            kinematics_msgs.srv.GetPositionFK)

    solverInfo = get_ik_solver_info().kinematic_solver_info

    # Setup the FK request
    fkr = kinematics_msgs.srv.GetPositionFKRequest()
    fkr.header.frame_id = "base_link"
    fkr.fk_link_names = ["right_foot_pad"]
    if joint_names is None:
        joint_names = solverInfo.joint_names
    fkr.robot_state.joint_state.name = solverInfo.joint_names
    fkr.robot_state.joint_state.position = [0 for i in solverInfo.joint_names]

    # Make the FK call
    response = get_position_fk(fkr)

    #rospy.logwarn(response.__repr__())

    # Setup the IK request
    ikr = kinematics_msgs.srv.GetPositionIKRequest()
    ikr.timeout = rospy.Duration(5)
    ikr.ik_request.ik_link_name = "right_foot_pad"
    ikr.ik_request.pose_stamped.header.frame_id = "base_link"
    ikr.ik_request.pose_stamped.pose.position.x = x
    ikr.ik_request.pose_stamped.pose.position.y = y
    ikr.ik_request.pose_stamped.pose.position.z = z
    ikr.ik_request.pose_stamped.pose.orientation.x = 0.0
    ikr.ik_request.pose_stamped.pose.orientation.y = 0.0
    ikr.ik_request.pose_stamped.pose.orientation.z = 0.0
    ikr.ik_request.pose_stamped.pose.orientation.w = 1.0

    ikr.ik_request.ik_seed_state.joint_state.name = solverInfo.joint_names
    if seed is None:
        seed = [0 for i in solverInfo.joint_names]
    ikr.ik_request.ik_seed_state.joint_state.position = seed

    before = rospy.get_time();
    response = get_position_ik(ikr)
    after = rospy.get_time()

    if response.error_code.val == 1:
        rospy.logwarn("publishing!")
        if(indices is None):
            indices = []
            a = response.solution.joint_state.name
            for name in solverInfo.joint_names:
                for (i,s) in zip(range(len(a)),a):
                    if s == name:
                        indices.append(i)
        else:
            seed = [response.solution.joint_state.position[x] for x in indices]
        pub.publish(response.solution.joint_state)
        return 1
    elif response.error_code.val != -31:
        rospy.logwarn("other error code! %d" % response.error_code.val)

    return 0
    
    #rospy.logwarn(response.__repr__())

# extended natural pos = 0.03, 0.0, -0.215

def controller_loop():
    mins = {}
    maxs = {}
    incr = {}
    mins['x'] = 0.0
    maxs['x'] = 0.06
    mins['y'] = -0.05
    maxs['y'] = 0.05
    mins['z'] = -0.215
    maxs['z'] = -0.150
    incr['x'] = 0.001
    incr['y'] = 0.001
    incr['z'] = 0.001
    key = "xyz"
    while True:
        a = mins[key[0]]
        while a < maxs[key[0]]:
            b = mins[key[1]]
            while b < maxs[key[1]]:
                c = mins[key[2]]
                while c < maxs[key[2]]:
                    vals = [a,b,c]
                    if(controller(vals[key.rfind('x')],
                                  vals[key.rfind('y')],
                                  vals[key.rfind('z')])):
                        pass #rospy.sleep(0.010)
                    c += incr[key[2]]
                b += incr[key[1]]
            a += incr[key[0]]

class ControllerGui(wx.Frame):
    def __init__(self, title):
        wx.Frame.__init__(self, None, -1, title, (-1, -1));
        panel = wx.Panel(self, wx.ID_ANY);
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
        self.joint_map = {}
        
        ### Sliders ###
        for name in "xyz":
            rospy.logwarn("initing name "+name);

            row = wx.GridSizer(1,2)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (panel, value=str(0), 
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)

            row.Add(display, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE, 
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.joint_map[name] = {'slidervalue':0, 'display':display, 
                                    'slider':slider}

            display.SetValue("%.2f"%initial[name])
            slider.SetValue(int((initial[name] + 0.5)*float(RANGE)))

        ### Buttons ###
        self.ctrbutton = wx.Button(panel, 1, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        
        box.Add(self.ctrbutton, 0, wx.EXPAND)
        
        panel.SetSizer(box)
        box.Fit(self)
        self.update_values()


    def update_values(self):
        d = {}
        for k in "xyz":
            d[k] = self.joint_map[k]['slider'].GetValue()
        for (k, j) in self.joint_map.items():
            j['slider'].SetValue(d[k])
            d[k] = d[k]/float(RANGE) - 0.5
            j['display'].SetValue("%.2f"%d[k])

        x = d['x']
        y = d['y']
        z = d['z']
        
        #rospy.logwarn("update_values "+str(x)+str(y)+str(z))
        controller(x,y,z)


    def sliderUpdate(self, event):
        #rospy.logwarn("got a slider update!")
        self.update_values()


if __name__ == '__main__':
    try:
        rospy.Subscriber("/motor_states/ttyUSB0", MotorStateList, process_motor_states)
        #controller(0,0,0)
        # Enable this if you want to search through a rectangular prism of
        # poses for the right foot

        #controller_loop()
        app = wx.App()
        x = ControllerGui("Controller")
        x.Show()
        ## FIXME for some reason killing roslaunch doesn't always kill this app
        ## but roslaunch is smart enough to send sigkill, it just takes a while
        Thread(target=app.MainLoop).start()
        while not rospy.is_shutdown():
            #rospy.logwarn("looping...");
            rospy.sleep(1)
	
	rospy.logwarn("shutdown!")

	exit()
            
    except rospy.ROSInterruptException: pass
