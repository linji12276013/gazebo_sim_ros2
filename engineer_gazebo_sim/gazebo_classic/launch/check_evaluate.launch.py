from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
      
    # 声明参数（可选，若需要从命令行动态传入）
    name = DeclareLaunchArgument(
        'name', default_value='my_eih_calib',
        description='Unique identifier for the calibration'
    )
 
    handeye_calibration_evaluate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch/evaluate.launch.py'
            ])
        ]),
        launch_arguments={
            'name': LaunchConfiguration('name'),
        }.items()
    )
    
    return LaunchDescription([
        name,
        handeye_calibration_evaluate
    ])
