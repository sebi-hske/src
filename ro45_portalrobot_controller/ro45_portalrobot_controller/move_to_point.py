
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ro45_action_interfaces.action import MovetoPos


class MoveToPoint:
    def __init__(self, x, y, z):
        self.position_action_server = ActionServer(
            self,
            MovetoPos,
            'move_to_position',
            execute_callback=self.position_execute_callback,
            goal_callback=self.position_goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        self.corr_val_x, self.corr_val_y, self.corr_val_z = x,y,z
        
    

    def position_execute_callback(self, goal_handle):
        try:
            self.get_logger().info("Executing goal...")
            self.timer_period = 0.01
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            goal_handle.succeed()    
            result = MovetoPos.Result()    
            return result
        except Exception as e:
            self.get_logger().error(f"Error during goal execution: {e}")
            goal_handle.abort()
            self.failsafe_hold_pos()
            return None
        
    def position_goal_callback(self, goal_request):
        self.get_logger().info("Received goal request to move to position: " + str(goal_request))
        self.desired_pos_x = (goal_request.position_x - self.corr_val_x) * -1.0
        self.desired_pos_y = (goal_request.position_y - self.corr_val_y) * -1.0
        self.desired_pos_z = (goal_request.position_z - self.corr_val_z) # * -1.0
        self.get_logger().info("corrected positions for robot "+  str(self.desired_pos_x)+str(self.desired_pos_y)+str(self.desired_pos_z))
        return GoalResponse.ACCEPT
        
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Replacing active goal with new goal.')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()    

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancelled. Stopping execution")
        self.failsafe_hold_pos()
        goal_handle.canceled()
        return CancelResponse.ACCEPT