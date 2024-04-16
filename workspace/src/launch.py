
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # # publishes images in a loop
        Node(
            package='test_package',
            executable='test_image_publisher',
            output='screen',
            namespace='/',
            name='test_image_publisher',
            # remappings=[
            #      ('/image_topic', '/camera/image_raw')
            # ]
        ),

        # shows received image
        # Node(
        #     package='test_package',
        #     executable='test_image_display',
        #     output='screen',
        #     namespace='/',
        #     name='test_image_display',
        #     remappings=[
        #          ('/image_topic', '/camera/image_raw')
        #     ]
        # ),
        
        # webcam node
        # Node(
        #     package='webcam_node',
        #     executable='webcam_node',
        #     # output='screen',
        #     namespace='/',
        #     name='webcam'
        # ),
        # object detection node
        Node(
            package='object_detection',
            executable='object_detection',
            # output='screen',
            namespace='/',
            name='object_detection',
            output="screen"
        ),

        # # Tello driver node
        # Node(
        #     package='tello',
        #     executable='tello',
        #     output='screen',
        #     namespace='/',
        #     name='tello',
        #     parameters=[
        #         {'connect_timeout': 10.0},
        #         {'tello_ip': '192.168.10.1'},
        #         {'tf_base': 'map'},
        #         {'tf_drone': 'drone'}
        #     ],
        #     # remappings=[
        #     #     ('/image_raw', '/camera')
        #     # ],
        #     respawn=True
        # ),

        # # Tello control node
        # Node(
        #     package='tello_control',
        #     executable='tello_control',
        #     namespace='/',
        #     name='control',
        #     output='screen',
        #     respawn=False
        # ),

        # # RQT topic debug tool
        # Node(
        #     package='rqt_gui',
        #     executable='rqt_gui',
        #     output='screen',
        #     namespace='/',
        #     name='rqt',
        #     respawn=False
        # ),

        # RViz data visualization tool
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     namespace='/',
        #     name='rviz2',
        #     respawn=True,
        #     arguments=['-d', '/home/jon/tello-slam-and-recognition/workspace/src/rviz.rviz']
        # ),

        # # Static TF publisher
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     namespace='/',
        #     name='tf',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
        #     respawn=True
        # ),


#############################old nodes don't use
        # # ORB SLAM
        # Node(
        #     package='orbslam2',
        #     executable='mono',
        #     output='screen',
        #     namespace='/',
        #     name='orbslam',
        #     respawn=True,
        #     remappings=[
        #         ('/camera', '/image_raw')
        #     ],
        #     arguments=['home/jon/tello-slam-and-recognition/libs/ORB_SLAM2/Vocabulary/ORBvoc.txt', '~/home/jon/tello-slam-and-recognition/workspace/src/orbslam2/config.yaml']
        # ),

        # Camera calibration node
        # Node(
        #     package='camera_calibration',
        #     executable='cameracalibrator',
        #     output='screen',
        #     respawn=True,
        #     namespace='/',
        #     name='calibration',
        #     arguments=['--size', '7x9', '--square', '0.20'],
        #     parameters=[
        #         {'image': '/image_raw'},
        #         {'camera': '/camera_info'}
        #     ]
        # )
    ]


    return LaunchDescription(nodes)