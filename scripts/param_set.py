#!/usr/bin/env python3
import os
import sys
import time
import rclpy
from rclpy.node import Node
from tianbot_core.srv import DebugCmd

robot_name = os.getenv("ROBOT_NAME", "tianbot")

class ParamSetClient(Node):
    def __init__(self):
        super().__init__('param_set_client')
        service_name = robot_name + '/' + 'debug_cmd_srv'
        self.get_logger().warn("service_name: {0}".format(service_name))
        self.client = self.create_client(DebugCmd, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, param):
        req = DebugCmd.Request()
        req.cmd = param
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
 
def param_set_client(param):
    rclpy.init()
    client = ParamSetClient()
    result = client.send_request(param)
    client.destroy_node()
    rclpy.shutdown()
    return result.result

def param_get_client():
    rclpy.init()
    client = ParamSetClient()
    result = client.send_request("param get")
    client.destroy_node()
    rclpy.shutdown()
    return result.result

def param_save_client():
    rclpy.init()
    client = ParamSetClient()
    result = client.send_request("param save")
    client.destroy_node()
    rclpy.shutdown()
    return result.result

def reset_client():
    rclpy.init()
    client = ParamSetClient()
    result = client.send_request("reset")
    client.destroy_node()
    rclpy.shutdown()
    return result.result

def usage():
    return "%s [param_file_name]" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        filename = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print("param file [%s]\n" % filename)

    with open(filename, 'r') as file:
        for line in file.readlines():
            if line.strip() == '' or line.strip().startswith('#'):
                continue
            print("param set " + line.strip())
            # print(param_set_client(line.strip()))

    print("param save ...")
    print(param_save_client())
    time.sleep(0.1)

    reset_client()
    print("wait 7 sec for reset ...\n")
    time.sleep(7)

    print("param get:")
    print(param_get_client())