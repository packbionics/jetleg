#!/usr/bin/python3
import sys
from array import array as array
from packbionics_interfaces.srv._update_impedance import UpdateImpedance
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(UpdateImpedance, '/jetleg_controller/impedance_params')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdateImpedance.Request()

    def send_request(self, stiffness, damping, equilibrium):
        self.req.stiffness = stiffness
        self.req.damping = damping
        self.req.equilibrium = equilibrium
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

    def GaitMode(Littlephase):
        gait_phases = ["IC", "LR", "Ms", "TS", "Ps", "Is", "MS", "LS"]
        gait_values = [[-1.0,0.0,1.0,- 1.0,0.0,1.0],[-1.0,-1.0,1.0, -1.0,0.0,1.0],[-1.0, 1.0,0.0, -1.0,0.0,1.0],[1.0,0.0,-1.0, -1.0,0.0,1.0],
                       [-1.0,0.0,1.0, -1.0,0.0,1.0],[-1.0,-1.0,1.0], -1.0,0.0,1.0, [-1.0, 1.0,0.0, -1.0,0.0,1.0],[1.0,0.0,-1.0, -1.0,0.0,1.0]]
        gp_orientation = dict(zip(gait_phases,gait_values))

        if GaitPhase(Littlephase) == "Stance":

    



    def GaitPhase(Bigphase):
        if Bigphase == "IC" or phase == "LR" or phase == "Ms" or phase == "TS":
            return "Stance"
        else:
            return "Swing"

        print("blobk")
        return 5



    



def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    stiffness = [1.0,1.0,0.0]
    damping = [1.0 , 1.0 , 0.0]
    equilibrium = [0.0 , 0.0, 0.0] 
    

    stiffness = array('d', stiffness)
    damping = array('d', damping)
    equilibrium = array('d', equilibrium)

    response = minimal_client.send_request(stiffness, damping, equilibrium)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for'  
        )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
class GaitMode():
        def __init__(self, BigPhase, LittlePhases, PhaseChosen):
            self.BigPhase = BigPhase
            self.LittlePhase = LittlePhases
            self.PhaseChosen = PhaseChosen
            self.gait_phase_list = ["IC", "LR", "Ms", "TS", "Ps", "Is", "MS", "LS"]
        def Transition(self, Number):
            self.PhaseChosen = self.gait_phase_list[Number]
        


        
        


class GaitPhase():
    #contains a set of parameters (numbers) associated with each phase
    #will be read via a yaml file
    #take in an index corresponding to the index of the phase we want
    # 3 lists, 1 for each parameter

