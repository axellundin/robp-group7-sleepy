import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='arm_camera',
            namespace='arm_camera',
            parameters=[{
                'video_device': '/dev/arm_camera',
                'frame_id': 'arm_camera_link',
                'camera_name': 'arm_camera',
                'pixel_format': 'yuyv',
                'framerate': 5.0,
                'image_width': 640,
                'image_height': 480,
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1
            }]
        )
    ])