import rospy
from pr2_mechanism_msgs.srv import ListControllers, ListControllersRequest, SwitchController, SwitchControllerRequest


class ControllerClient:
    """
    Switch to either Cartesian or joint controllers for one arm

    controller_type is 'joint' or 'cartesian'
    arm is 'right' or 'left'

    Outcomes:
    * 'succeeded':  The controllers are switched.
    * 'failed':  The controllers are not switched.
    """

    def __init__(self, input_keys=['controller_type', 'arm']):
        
        switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
        list_controllers_serv_name = 'pr2_controller_manager/list_controllers'

        rospy.wait_for_service(list_controllers_serv_name)
        rospy.wait_for_service(switch_controller_serv_name)

        self.switch_controller_service = \
            rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
        self.list_controllers_service = \
            rospy.ServiceProxy(list_controllers_serv_name, ListControllers)

        #names set each time in set_controller_names for the appropriate arm
        self.joint_controller = 'r_arm_controller'
        self.cartesian_controller = 'r_cart'

        #current state of the controllers, set by check_controller_states
        self.joint_controller_state = None
        self.cartesian_controller_state = None

    ##set the controller names appropriately for the arm being switched
    def set_controller_names(self, arm):
        if arm == "right": 
            self.joint_controller = 'r_arm_controller'
            self.cartesian_controller = 'r_cart'
        else:
            self.joint_controller = 'l_arm_controller'
            self.cartesian_controller = 'l_cart'


    ##figure out which controllers are loaded/started (fills in self.joint_controller_state and 
    # self.cartesian_controllers_state with 'not loaded', 'stopped', or 'running'
    def check_controller_states(self):
        resp = self.list_controllers_service()

        if self.joint_controller in resp.controllers:
            self.joint_controller_state = resp.state[resp.controllers.index(self.joint_controller)]
        else:
            self.joint_controller_state = 'not loaded'

        if self.cartesian_controller in resp.controllers:
            self.cartesian_controller_state = resp.state[resp.controllers.index(self.cartesian_controller)]
        else:
            self.cartesian_controller_state = 'not loaded'


    ##stops the joint controllers, starts the Cartesian ones (both need to be loaded already)
    def switch_to_cartesian_mode(self):
        self.check_controller_states()

        if self.cartesian_controller_state == 'not loaded':
            rospy.logerr("Cartesian controller is not loaded!")
            return 'failed'

        if not (self.joint_controller_state == 'stopped' and self.cartesian_controller_state == 'running'):  
            success = self.switch_controller_service([self.cartesian_controller,], [self.joint_controller,], 2)
            if success:
                rospy.loginfo("switched joint to Cartesian successfully")
                return 'succeeded'
            else:
                rospy.logerr("switching joint to Cartesian failed")
                return 'failed'
            
        # already in Cartesian mode
        return 'succeeded'


    ##stops the Cartesian controllers, starts the joint ones (both need to be loaded already)
    def switch_to_joint_mode(self):
        self.check_controller_states()

        if self.joint_controller_state == 'not loaded':
            rospy.logerr("joint controller is not loaded!")
            return 'failed'

        if not (self.joint_controller_state == 'running' and self.cartesian_controller_state == 'stopped'):
            success = self.switch_controller_service([self.joint_controller,], [self.cartesian_controller,], 2)
            if success:
                rospy.loginfo("switched Cartesian to joint successfully")
                return 'succeeded'
            else:
                rospy.logerr("switching Cartesian to joint failed")
                return 'failed'

        # already in joint mode
        return 'succeeded'
        

