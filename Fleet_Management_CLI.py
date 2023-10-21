#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
from your_project_msgs.action import FleetManagementAction  # Replace with your actual message structure

class FleetManagementCLI(Node):
    def __init__(self):
        super().__init__('fleet_management_cli')
        self.action_client = self.create_action_client(FleetManagementAction, 'fleet_management_action')

    async def allocate_vehicles(self, vehicles):
        goal_msg = FleetManagementAction.Goal(order=vehicles)
        await self.action_client.wait_for_server()
        self.action_client.send_goal(goal_msg)
        await self.action_client.wait_for_result()
        result = self.action_client.get_result()
        self.get_logger().info(f'Allocated routes: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    cli = FleetManagementCLI()

    parser = argparse.ArgumentParser(description='Fleet Management CLI')
    parser.add_argument('vehicles', metavar='vehicle', type=int, nargs='+', help='Vehicle IDs to allocate')

    args = parser.parse_args()
    try:
        rclpy.spin(cli.allocate_vehicles(args.vehicles))
    except KeyboardInterrupt:
        pass

    cli.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

