import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_dir = get_package_share_directory('yoloddx')
    model_path = os.path.join(pkg_dir, 'models')
    
    # Define arguments
    cam_arg = DeclareLaunchArgument(
        'camera',
        default_value='/color/image_raw',
        description='Camera topic to use')
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8m-pose.pt',
        description='Model to use',
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace', 
        default_value='', 
        description='Namespace to use (e.g., 1 for /r1)')
    
    # Base namespace with r-prefix when a value is provided
    base_namespace = PythonExpression([
        "'r' + '",
        LaunchConfiguration('namespace'),
        "' if '",
        LaunchConfiguration('namespace'),
        "' != '' else ''"
    ])
    
    # Full namespace for yolo node (base + /yolo)
    yolo_namespace = PythonExpression([
        "'",
        base_namespace,
        "/yolo' if '",
        LaunchConfiguration('namespace'),
        "' != '' else 'yolo'"
    ])
    
    # First determine the camera topic based on camera arg (without namespace)
    base_camera_topic = PythonExpression([
        "'/rgb/image_raw' if '",
        LaunchConfiguration('camera'),
        "' == 'xtion' else ",
        "'/rgbd_camera/image' if '",
        LaunchConfiguration('camera'),
        "' == 'simulator' else ",
        "'/image_raw' if '",
        LaunchConfiguration('camera'),
        "' == 'laptop' else '",
        LaunchConfiguration('camera'),
        "'"
    ])
    
    # Then add the namespace prefix if provided
    camera_topic = PythonExpression([
        "'/'+ '",
        base_namespace,
        "' + '",
        base_camera_topic,
        "' if '",
        LaunchConfiguration('namespace'),
        "' != '' else '",
        base_camera_topic,
        "'"
    ])
    
    # Model selection with proper quotes
    model_file = PythonExpression([
        "'" + model_path + "/yolov8m.pt' if '",
        LaunchConfiguration('model'),
        "' == 'v8' else ",
        "'" + model_path + "/yolov8m-seg.pt' if '",
        LaunchConfiguration('model'),
        "' == 'seg' else '",
        model_path + "/",
        LaunchConfiguration('model'),
        "'"
    ])
    
    return LaunchDescription(
        [
            cam_arg,
            model_arg,
            namespace_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('yolo_bringup'),
                        'launch',
                        'yolo.launch.py',
                    )
                ),
                launch_arguments={
                    'model': model_file,
                    'input_image_topic': camera_topic,
                    'namespace': yolo_namespace,  # Using namespace with /yolo suffix
                }.items(),
            ),
        ]
    )