#!/usr/bin/env python3
"""
Launch file for Enhanced Dual GPS Converter
Part of MoMo_SAM Georeferencing Implementation Plan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for Enhanced Dual GPS Converter"""
    
    # Declare launch arguments
    input_gps_topic_arg = DeclareLaunchArgument(
        'input_gps_topic',
        default_value='/gps/fix',
        description='Input GPS topic (NavSatFix messages)'
    )
    
    output_local_topic_arg = DeclareLaunchArgument(
        'output_local_topic',
        default_value='/gnss/local/odom',
        description='Output local coordinate topic'
    )
    
    output_global_topic_arg = DeclareLaunchArgument(
        'output_global_topic',
        default_value='/gnss/global/odom',
        description='Output global coordinate topic'
    )
    
    utm_zone_arg = DeclareLaunchArgument(
        'utm_zone',
        default_value='29',
        description='UTM zone number'
    )
    
    utm_hemisphere_arg = DeclareLaunchArgument(
        'utm_hemisphere',
        default_value='N',
        description='UTM hemisphere (N or S)'
    )
    
    gps_quality_threshold_arg = DeclareLaunchArgument(
        'gps_quality_threshold',
        default_value='8.0',
        description='GPS quality threshold in m² (consumer GPS compatible)'
    )
    
    enable_quality_filtering_arg = DeclareLaunchArgument(
        'enable_quality_filtering',
        default_value='true',
        description='Enable GPS quality filtering'
    )
    
    origin_establishment_samples_arg = DeclareLaunchArgument(
        'origin_establishment_samples',
        default_value='5',
        description='Number of GPS samples required for origin establishment'
    )
    
    origin_variance_threshold_arg = DeclareLaunchArgument(
        'origin_variance_threshold',
        default_value='10.0',
        description='Maximum allowed variance for origin establishment (meters)'
    )
    
    # Enhanced Dual GPS Converter node
    enhanced_dual_gps_converter_node = Node(
        package='momo_sam',
        executable='enhanced_dual_gps_converter.py',
        name='enhanced_dual_gps_converter',
        output='screen',
        parameters=[{
            'input_gps_topic': LaunchConfiguration('input_gps_topic'),
            'output_local_topic': LaunchConfiguration('output_local_topic'),
            'output_global_topic': LaunchConfiguration('output_global_topic'),
            'utm_zone': LaunchConfiguration('utm_zone'),
            'utm_hemisphere': LaunchConfiguration('utm_hemisphere'),
            'gps_quality_threshold': LaunchConfiguration('gps_quality_threshold'),
            'enable_quality_filtering': LaunchConfiguration('enable_quality_filtering'),
            'local_frame_id': 'gps_local',
            'global_frame_id': 'gps_global',
            'origin_establishment_samples': LaunchConfiguration('origin_establishment_samples'),
            'origin_variance_threshold': LaunchConfiguration('origin_variance_threshold'),
        }],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        input_gps_topic_arg,
        output_local_topic_arg,
        output_global_topic_arg,
        utm_zone_arg,
        utm_hemisphere_arg,
        gps_quality_threshold_arg,
        enable_quality_filtering_arg,
        origin_establishment_samples_arg,
        origin_variance_threshold_arg,
        
        # Nodes
        enhanced_dual_gps_converter_node,
    ])