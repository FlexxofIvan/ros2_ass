"""Original 18 DoF teaching humanoid; geometry and joint contract live here.

The pelvis is fixed to the world. No mesh, gait, balance controller or external
robot asset is used. Limits are teaching bounds, not a physical robot's ratings.
"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ARM_SUFFIXES = ('shoulder_pitch', 'shoulder_roll', 'elbow_pitch', 'wrist_roll')
LEG_SUFFIXES = ('hip_yaw', 'hip_roll', 'hip_pitch', 'knee_pitch', 'ankle_pitch')
ARM_JOINT_NAMES = tuple(f'{side}_{joint}' for side in ('left', 'right') for joint in ARM_SUFFIXES)
LEG_JOINT_NAMES = tuple(f'{side}_{joint}' for side in ('left', 'right') for joint in LEG_SUFFIXES)
JOINT_NAMES = ARM_JOINT_NAMES + LEG_JOINT_NAMES
_LIMITS = {'shoulder_pitch': (-1.4, 1.4), 'shoulder_roll': (-1.0, 1.0),
           'elbow_pitch': (-1.5, 0.2), 'wrist_roll': (-1.4, 1.4),
           'hip_yaw': (-0.6, 0.6), 'hip_roll': (-0.5, 0.5), 'hip_pitch': (-1.0, 1.0),
           'knee_pitch': (-0.1, 1.5), 'ankle_pitch': (-0.7, 0.7)}
JOINT_LIMITS = {joint: _LIMITS[joint.split('_', 1)[1]] for joint in JOINT_NAMES}
JOINT_VELOCITY_LIMITS = {joint: 1.5 for joint in JOINT_NAMES}
CONTROLLER_NAME = 'humanoid_controller'


def namespace_name(value):
    value = value.strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*', value):
        raise ValueError('namespace must contain nonempty ROS name components')
    return value


def practice_config():
    """Working public practice preset for the guided exercise."""
    return {
        '/**/controller_manager': {'ros__parameters': {
            'update_rate': 100,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            CONTROLLER_NAME: {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        f'/**/{CONTROLLER_NAME}': {'ros__parameters': {
            'joints': list(JOINT_NAMES), 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'], 'allow_partial_joints_goal': False,
            'action_monitor_rate': 20.0,
            'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': 0.05,
                            **{joint: {'trajectory': 0.15, 'goal': 0.04} for joint in JOINT_NAMES}}}}}


def robot_description(namespace, controller_yaml):
    namespace = namespace_name(namespace)
    robot = ET.Element('robot', name='course_humanoid')
    ET.SubElement(robot, 'link', name='world')

    def link(name, size, mass, xyz='0 0 0', color='0.2 0.45 0.8 1'):
        node = ET.SubElement(robot, 'link', name=name)
        inertial = ET.SubElement(node, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=xyz)
        ET.SubElement(inertial, 'mass', value=str(mass))
        x, y, z = size
        ET.SubElement(inertial, 'inertia', ixx=str(mass*(y*y+z*z)/12),
                      iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                      ixy='0', ixz='0', iyz='0')
        for tag in ('visual', 'collision'):
            shape = ET.SubElement(node, tag)
            ET.SubElement(shape, 'origin', xyz=xyz)
            ET.SubElement(ET.SubElement(shape, 'geometry'), 'box', size=' '.join(map(str,size)))
            if tag == 'visual':
                material = ET.SubElement(shape, 'material', name=f'{name}_color')
                ET.SubElement(material, 'color', rgba=color)
        return node

    def joint(name, parent, child, xyz, axis='0 1 0', fixed=False):
        node = ET.SubElement(robot, 'joint', name=name, type='fixed' if fixed else 'revolute')
        ET.SubElement(node, 'parent', link=parent)
        ET.SubElement(node, 'child', link=child)
        ET.SubElement(node, 'origin', xyz=xyz)
        if not fixed:
            low, high = JOINT_LIMITS[name]
            ET.SubElement(node, 'axis', xyz=axis)
            ET.SubElement(node, 'limit', lower=str(low), upper=str(high),
                          effort='10' if name in ARM_JOINT_NAMES else '30', velocity='1.5')
            ET.SubElement(node, 'dynamics', damping='0.05', friction='0.0')

    link('pelvis', (0.24,0.26,0.16), 3.0, color='0.12 0.2 0.3 1')
    joint('world_to_pelvis', 'world', 'pelvis', '0 0 1.0', fixed=True)
    link('torso', (0.22,0.32,0.35), 4.0, xyz='0 0 0.19')
    joint('pelvis_to_torso', 'pelvis', 'torso', '0 0 0.08', fixed=True)
    link('head', (0.15,0.16,0.16), 0.5, color='0.9 0.75 0.25 1')
    joint('torso_to_head', 'torso', 'head', '0 0 0.47', fixed=True)
    for side, sign in (('left',1),('right',-1)):
        color = '0.15 0.65 0.55 1' if sign == 1 else '0.85 0.35 0.18 1'
        link(f'{side}_shoulder_mount',(0.045,0.045,0.045),0.1,color=color)
        joint(f'{side}_shoulder_pitch','torso',f'{side}_shoulder_mount',f'0 {sign*0.21} 0.31')
        link(f'{side}_upper_arm',(0.065,0.065,0.24),0.5,xyz='0 0 -0.12',color=color)
        joint(f'{side}_shoulder_roll',f'{side}_shoulder_mount',f'{side}_upper_arm','0 0 0',axis='1 0 0')
        link(f'{side}_forearm',(0.055,0.055,0.21),0.35,xyz='0 0 -0.105',color=color)
        joint(f'{side}_elbow_pitch',f'{side}_upper_arm',f'{side}_forearm','0 0 -0.24')
        link(f'{side}_hand',(0.075,0.04,0.10),0.15,xyz='0 0 -0.05',color=color)
        joint(f'{side}_wrist_roll',f'{side}_forearm',f'{side}_hand','0 0 -0.21',axis='0 0 1')
        link(f'{side}_hip_yaw_link',(0.04,0.04,0.04),0.12,color=color)
        joint(f'{side}_hip_yaw','pelvis',f'{side}_hip_yaw_link',f'0 {sign*0.09} -0.1',axis='0 0 1')
        link(f'{side}_hip_roll_link',(0.04,0.04,0.04),0.12,color=color)
        joint(f'{side}_hip_roll',f'{side}_hip_yaw_link',f'{side}_hip_roll_link','0 0 0',axis='1 0 0')
        link(f'{side}_thigh',(0.085,0.085,0.30),1.0,xyz='0 0 -0.15',color=color)
        joint(f'{side}_hip_pitch',f'{side}_hip_roll_link',f'{side}_thigh','0 0 0')
        link(f'{side}_shin',(0.075,0.075,0.30),0.7,xyz='0 0 -0.15',color=color)
        joint(f'{side}_knee_pitch',f'{side}_thigh',f'{side}_shin','0 0 -0.30')
        link(f'{side}_foot',(0.18,0.09,0.055),0.3,xyz='0.04 0 -0.028',color=color)
        joint(f'{side}_ankle_pitch',f'{side}_shin',f'{side}_foot','0 0 -0.30')
    control = ET.SubElement(robot, 'ros2_control', name='GazeboSimSystem', type='system')
    ET.SubElement(ET.SubElement(control,'hardware'),'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for name in JOINT_NAMES:
        node = ET.SubElement(control,'joint',name=name)
        command = ET.SubElement(node,'command_interface',name='position')
        ET.SubElement(command,'param',name='min').text = str(JOINT_LIMITS[name][0])
        ET.SubElement(command,'param',name='max').text = str(JOINT_LIMITS[name][1])
        position = ET.SubElement(node,'state_interface',name='position')
        ET.SubElement(position,'param',name='initial_value').text = '0.0'
        ET.SubElement(node,'state_interface',name='velocity')
        ET.SubElement(node,'state_interface',name='effort')
    plugin = ET.SubElement(ET.SubElement(robot,'gazebo'),'plugin',
                           filename='libgz_ros2_control-system.so',name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text = str(controller_yaml)
    ET.SubElement(plugin,'hold_joints').text = 'true'
    ET.SubElement(plugin,'position_proportional_gain').text = '0.1'
    ET.SubElement(ET.SubElement(plugin,'ros'),'namespace').text = f'/{namespace}'
    ET.indent(robot)
    return ET.tostring(robot,encoding='unicode')+'\n'


WORLD = '''<?xml version="1.0"?>
<sdf version="1.9"><world name="control_lab">
  <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <gravity>0 0 -9.81</gravity>
  <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -1</direction></light>
  <model name="floor"><static>true</static><link name="floor"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision><visual name="floor"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry><material><ambient>0.65 0.65 0.68 1</ambient><diffuse>0.65 0.65 0.68 1</diffuse></material></visual></link></model>
  <model name="pelvis_support"><static>true</static><pose>-0.22 0 0.5 0 0 0</pose><link name="support"><visual name="support"><geometry><box><size>0.07 0.08 1.0</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual><visual name="pelvis_mount"><pose>0.12 0 0.5 0 0 0</pose><geometry><box><size>0.24 0.12 0.04</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>
</world></sdf>'''


def generate(mode, namespace, output_dir, controller_config=None):
    """Create absolute launch resources for the guided practice."""
    import yaml
    if mode != 'practice':
        raise ValueError('mode must be practice')
    namespace = namespace_name(namespace)
    folder = Path(output_dir).resolve()
    folder.mkdir(parents=True,exist_ok=True)
    config = folder/'controllers.yaml'
    if controller_config:
        source = Path(controller_config).expanduser().resolve()
        data = yaml.safe_load(source.read_text())
        if not isinstance(data,dict):
            raise ValueError('controller configuration must be a YAML mapping')
        config.write_text(yaml.safe_dump(data,sort_keys=False))
    else:
        config.write_text(yaml.safe_dump(practice_config(),sort_keys=False))
    urdf = folder/'humanoid.urdf'
    urdf.write_text(robot_description(namespace,config))
    world = folder/'world.sdf'
    world.write_text(WORLD)
    bridge = folder/'bridge.yaml'
    bridge.write_text(yaml.safe_dump([{'ros_topic_name':'/clock','gz_topic_name':'/clock',
        'ros_type_name':'rosgraph_msgs/msg/Clock','gz_type_name':'gz.msgs.Clock','direction':'GZ_TO_ROS'}]))
    rviz = folder/'view.rviz'
    rviz.write_text(f'''Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: {namespace}/world
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/RobotModel
      Name: Robot
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /{namespace}/robot_description
        Durability Policy: Transient Local
      TF Prefix: {namespace}
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Focal Point: {{X: 0.0, Y: 0.0, Z: 0.9}}
      Pitch: 0.2
      Yaw: 0.8
Window Geometry:
  Width: 1100
  Height: 800
''')
    return {key:str(path) for key,path in {'robot_urdf':urdf,'world_sdf':world,
            'controller_yaml':config,'bridge_yaml':bridge,'rviz_config':rviz}.items()}
