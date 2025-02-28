#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from waypoints import waypoints

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['orders_received', 'wait'], 
                             output_keys=['sequence', 'active_orders'])
        self.order_sub = rospy.Subscriber('/orders', String, self.order_cb)
        self.sequence = []
        self.active_orders = set()

    def order_cb(self, msg):
        tables = msg.data.split()
        self.sequence = tables
        self.active_orders = set(tables)

    def execute(self, userdata):
        rospy.sleep(1.0)  # Wait for orders
        if self.sequence:
            userdata.sequence = self.sequence
            userdata.active_orders = self.active_orders
            return 'orders_received'
        return 'wait'

class MovingToKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'canceled'], 
                             input_keys=['active_orders'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        if not userdata.active_orders:
            return 'canceled'
        goal = MoveBaseGoal()
        goal.target_pose = waypoints['kitchen']
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'success'

class WaitingAtKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['confirmed', 'timeout'], 
                             input_keys=['active_orders'])
    
    def execute(self, userdata):
        if not userdata.active_orders:
            return 'timeout'
        try:
            rospy.wait_for_message('/kitchen/confirmation', String, timeout=10.0)
            return 'confirmed'
        except rospy.ROSException:
            return 'timeout'

class SelectNextTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['table_selected', 'done'], 
                             input_keys=['sequence', 'active_orders'], 
                             output_keys=['current_table'])
        self.index = 0

    def execute(self, userdata):
        while self.index < len(userdata.sequence):
            table = userdata.sequence[self.index]
            self.index += 1
            if table in userdata.active_orders:
                userdata.current_table = table
                return 'table_selected'
        return 'done'

class MovingToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], 
                             input_keys=['current_table'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = waypoints[userdata.current_table]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'success'

class WaitingAtTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'], 
                             input_keys=['current_table'])
    
    def execute(self, userdata):
        try:
            rospy.wait_for_message('/' + userdata.current_table + '/confirmation', String, timeout=10.0)
        except rospy.ROSException:
            pass  # Proceed regardless of confirmation
        return 'next'

class ReturningToKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = waypoints['kitchen']
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'success'

class ReturningToHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = waypoints['home']
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'success'

def main():
    rospy.init_node('cafe_robot_smach')
    
    # Handle cancellations
    active_orders = set()
    def cancel_cb(msg):
        table = msg.data
        if table in active_orders:
            active_orders.remove(table)
    rospy.Subscriber('/cancellations', String, cancel_cb)

    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'orders_received': 'MOVING_TO_KITCHEN', 'wait': 'IDLE'})
        smach.StateMachine.add('MOVING_TO_KITCHEN', MovingToKitchen(), 
                               transitions={'success': 'WAITING_AT_KITCHEN', 'canceled': 'RETURNING_TO_HOME'})
        smach.StateMachine.add('WAITING_AT_KITCHEN', WaitingAtKitchen(), 
                               transitions={'confirmed': 'SELECT_NEXT_TABLE', 'timeout': 'RETURNING_TO_HOME'})
        smach.StateMachine.add('SELECT_NEXT_TABLE', SelectNextTable(), 
                               transitions={'table_selected': 'MOVING_TO_TABLE', 'done': 'RETURNING_TO_KITCHEN'})
        smach.StateMachine.add('MOVING_TO_TABLE', MovingToTable(), 
                               transitions={'success': 'WAITING_AT_TABLE'})
        smach.StateMachine.add('WAITING_AT_TABLE', WaitingAtTable(), 
                               transitions={'next': 'SELECT_NEXT_TABLE'})
        smach.StateMachine.add('RETURNING_TO_KITCHEN', ReturningToKitchen(), 
                               transitions={'success': 'RETURNING_TO_HOME'})
        smach.StateMachine.add('RETURNING_TO_HOME', ReturningToHome(), 
                               transitions={'success': 'IDLE'})
    
    # Add introspection for debugging
    sis = smach_ros.IntrospectionServer('server', sm, '/SM_ROOT')
    sis.start()
    
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
