self.camera_client = self.create_client(YoloImageDetect, 'final_detect', callback_group=group2)


self.next_obj[3]

    def send_camera_request_and_return_closest_objxy_with_class(self, x, y, obj_class):
        request = YoloImageDetect.Request()
        request.camera_name = "rgbd_camera"
        request.target_frame = "map"


        future = self.camera_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        while not future.done():
            self.executor.spin_once(timeout_sec=0.1) 

        if future.result() is not None:
            response = future.result()
            closest_distance = 1
            find_aligned_obj = False
            rx, ry = -1, -1

            for i, obj in enumerate(response.objects):
                pose = obj.center_point
                category = obj.category
                if category != "no_detection":
                    self.get_logger().warning("trying to make sure accurate obj position, but no obj detected")
                    return 0, 0 
                print(f"Object {i + 1}:")
                if category == obj_class:
                    x_real = (pose.pose.position.x)
                    y_real = (pose.pose.position.y)
                    distance = math.sqrt((x_real - x)**2 + (y_real - y)**2)
                    if distance < closest_distance:
                        closest_distance = distance
                        find_aligned_obj = True
                        rx, ry = x_real, y_real

            if find_aligned_obj == True:
                return rx, ry
            else:
                self.get_logger().warning('Failed to find obj with predicted class.')
                return 0, 0
                
        else:
            self.get_logger().error('Failed to receive response from camera.')
            return 0, 0