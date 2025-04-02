    def turn(self):
        ticks_per_revolution = 6

        start_time = self.get_clock().now()
        #self.send_camera_request()
        print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")

        for tick in range(ticks_per_revolution - 1):
            request_msg = MoveTo.Request()
            goal_list = Path()
            goal_list.header.frame_id = "map"
            goal_list.header.stamp = self.get_clock().now().to_msg()
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0
            qw = np.cos(np.pi/ticks_per_revolution) 
            qz = np.sin(np.pi/ticks_per_revolution)
            goal_msg.pose.orientation.z = float(qz)
            goal_msg.pose.orientation.w = float(qw)
            goal_msg.pose = self.transforme_to_map(goal_msg.pose)
            goal_list.poses.append(goal_msg)
            request_msg.path = goal_list
            request_msg.max_speed = 0.3
            request_msg.max_turn_speed = 0.2
            request_msg.enforce_orientation = True
            request_msg.stop_at_goal = True
            self.get_logger().info(f'Giving the camra time to detect objects, tick nr: {tick}')
            self.send_move_goal(request_msg)
            start_time = self.get_clock().now()
            #self.send_camera_request()
            #print(f"Camera request sent. Time taken: {self.get_clock().now() - start_time} seconds.")
            while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=0.1):
                # rclpy.spin_once(self, timeout_sec=0.01)
                self.executor.spin_once(timeout_sec=0.01)
