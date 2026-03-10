# Copyright 2023 NXROBO
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

from typing import List, Tuple

# Sagittarius arm models
_SGR_MODELS = (
    'sgr532',
)
 
def get_sagittarius_arm_models() -> Tuple[str]:
    """Get the tuple of Sagittarius arm models."""
    return _SGR_MODELS


def get_sagittarius_arm_joints(robot_model: str) -> List[str]:
    """
    Return a list of joints in the robot_model.

    :param robot_model: The robot model to get the joints of
    :return: A list of joint names of the given robot model
    :raises: KeyError if the robot model is not valid
    """
    if robot_model in _SGR_MODELS:
        return [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint_gripper_left', 'joint_gripper_right'
        ]
    else:
        raise KeyError(f'{robot_model} is not a valid robot model.')
