#!/usr/bin python3

# Import necessary ROS 2 modules and libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from fleet_management_msgs.action import Allocation

class FleetManagementClient(Node):
    def __init__(self):
        super().__init__('fleet_management_client')
        self.action_client = ActionClient(self, Allocation, 'allocate_vehicle')
        self.request_allocation()

    def request_allocation(self):
        vehicles_to_allocate = list(range(1, 11))  # Request routes for all vehicles
        goal_msg = Allocation.Goal(vehicles=vehicles_to_allocate)
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            goal_result = future.result().result
            for i, route in enumerate(goal_result.routes):
                print(f"Vehicle {i + 1} Route: {route.route}")
        else:
            print("Goal failed!")

def main(args=None):
    rclpy.init(args=args)
    fleet_management_client = FleetManagementClient()
    rclpy.spin(fleet_management_client)
    fleet_management_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

