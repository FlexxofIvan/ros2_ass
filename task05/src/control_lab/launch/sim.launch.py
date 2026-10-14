"""Launch the original fixed-pelvis robot and standard Jazzy controllers."""
from pathlib import Path
import shutil
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from control_lab.model import CONTROLLER_NAME, generate, namespace_name


def launch(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    mode = get('mode')
    ns = namespace_name(get('namespace'))
    config = get('controller_config')
    if mode != 'practice':
        raise ValueError('mode must be practice')
    folder = Path(tempfile.mkdtemp(prefix='l05-'))
    try:
        paths = generate(mode,ns,folder,config or None)
    except Exception:
        shutil.rmtree(folder,ignore_errors=True)
        raise
    print(f'[L05] {mode} resources: {folder}; namespace: /{ns}',flush=True)
    server = ExecuteProcess(cmd=['gz','sim','-s','-r','--headless-rendering','-v','3',paths['world_sdf']],
        output='screen',additional_env={'DISPLAY':'','EGL_PLATFORM':'surfaceless'})
    rsp = Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=ns,
        parameters=[{'robot_description':Path(paths['robot_urdf']).read_text(),
                     'frame_prefix':f'{ns}/','use_sim_time':True}],output='screen')
    bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',
        parameters=[{'config_file':paths['bridge_yaml']}],output='screen')
    spawn = Node(package='ros_gz_sim',executable='create',name='spawn_robot',
        arguments=['-world','control_lab','-name','course_humanoid','-file',paths['robot_urdf']],output='screen')
    broadcasters = Node(package='controller_manager',executable='spawner',
        arguments=['joint_state_broadcaster','-c',f'/{ns}/controller_manager',
                   '--controller-manager-timeout','60','--switch-timeout','30'],output='screen')
    controller = Node(package='controller_manager',executable='spawner',
        arguments=[CONTROLLER_NAME,'-c',f'/{ns}/controller_manager',
                   '--controller-manager-timeout','60','--switch-timeout','30'],output='screen')
    def after_spawn(event, context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=f'robot spawn failed: {event.returncode}'))]
        return [broadcasters]
    def after_broadcaster(event, context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=f'broadcaster activation failed: {event.returncode}'))]
        return [controller]
    def after_controller(event, context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=f'controller activation failed: {event.returncode}'))]
        print(f'[L05] READY /{ns}/{CONTROLLER_NAME}/follow_joint_trajectory',flush=True)
        return []
    actions = [server,rsp,bridge,spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=broadcasters,on_exit=after_broadcaster)),
        RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=after_controller)),
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: shutil.rmtree(folder,ignore_errors=True) or [])]))]
    if get('gui') == 'true':
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','-v','2'],output='screen'))
    if get('rviz') == 'true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',paths['rviz_config']],
                            parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    defaults = {'mode':'practice','namespace':'humanoid','controller_config':'','gui':'true','rviz':'true'}
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value)
                              for key,value in defaults.items()]+[OpaqueFunction(function=launch)])
