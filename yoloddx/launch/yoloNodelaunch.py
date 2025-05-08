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
        default_value='color/image_raw',
        description='Camera topic to use')
    
    model_arg = DeclareLaunchArgument(
        'model',
        # default_value=model_path + '/yolov8m-pose.pt',
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
    
    # Camera topic with base namespace but not /yolo part
    camera_topic = PythonExpression([
        "'/",
        base_namespace,
        "/rgb/image_raw' if '",
        LaunchConfiguration('namespace'),
        "' != '' else '/rgb/image_raw'"
    ])
    
    # Model selection with proper quotes
    model_file = PythonExpression([
        "'" + model_path + "/yolov8m.pt' if '",
        LaunchConfiguration('model'),
        "' == 'v8' else ",
        "'" + model_path + "/yolov8m-seg.pt' if '",
        LaunchConfiguration('model'),
        "' == 'seg' else '",
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