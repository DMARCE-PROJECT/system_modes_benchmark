# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ament_index_python.packages

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions

import launch_system_modes.actions
import launch_system_modes.event_handlers
import launch_system_modes.events

import lifecycle_msgs


def generate_launch_description():
    modelfile = (ament_index_python.packages.get_package_share_directory(
      'system_modes_benchmark') + '/benchmark_modes.yaml')

    mode_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes') + '/launch/mode_manager.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    safety = launch_system_modes.actions.System(
            name='safety',
            namespace='')

    nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_benchmark') + '/launch/nav2.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    manipulator = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_benchmark') + '/launch/manipulator.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    camera = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_benchmark') + '/launch/camera.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    guard = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_benchmark') + '/launch/guard.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    # Startup
    safety_configure = launch.actions.EmitEvent(
        event=launch_system_modes.events.ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(safety),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    safety_activate = launch.actions.EmitEvent(
        event=launch_system_modes.events.ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(safety),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))

    # Handlers
    on_inactive_handler = launch.actions.RegisterEventHandler(
        launch_system_modes.event_handlers.OnStateTransition(
            target_system_part=safety,
            goal_state='inactive',
            entities=[safety_activate]))

    description = launch.LaunchDescription()
    description.add_action(mode_manager)
    description.add_action(safety)
    description.add_action(safety_configure)
    description.add_action(on_inactive_handler)

    description.add_action(nav2)
    description.add_action(manipulator)
    description.add_action(camera)
    description.add_action(guard)

    return description
