"""Public practice transport commands and observations; never a homework client."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import yaml
from .model import CONTROLLER_NAME, JOINT_NAMES, JOINT_LIMITS, namespace_name


def practice_message(pose, duration):
    """Two supplied static poses for guided CLI use; no wave mission planner."""
    if pose not in ('neutral','reach') or not math.isfinite(duration) or not 0.2<=duration<=30:
        raise ValueError('pose neutral|reach; duration 0.2..30 s')
    positions = {joint:0.0 for joint in JOINT_NAMES}
    if pose == 'reach':
        positions.update(left_shoulder_pitch=-0.35,left_elbow_pitch=-0.45,
                         right_shoulder_pitch=-0.25,right_elbow_pitch=-0.30)
    sec = math.floor(duration)
    nsec = round((duration-sec)*1_000_000_000)
    if nsec == 1_000_000_000: sec,nsec = sec+1,0
    return {'joint_names':list(JOINT_NAMES),'points':[
        {'positions':[positions[joint] for joint in JOINT_NAMES],
         'time_from_start':{'sec':sec,'nanosec':nsec}}]}


def control(args):
    manager = f'/{args.namespace}/controller_manager'
    endpoint = f'/{args.namespace}/{CONTROLLER_NAME}'
    if args.operation == 'list':
        cmd = ['ros2','control','list_controllers','-c',manager]
    elif args.operation == 'interfaces':
        cmd = ['ros2','control','list_hardware_interfaces','-c',manager]
    elif args.operation == 'pose':
        cmd = ['ros2','topic','pub','--once','--wait-matching-subscriptions','1',
               endpoint+'/joint_trajectory','trajectory_msgs/msg/JointTrajectory',
               yaml.safe_dump(practice_message(args.pose,args.duration),default_flow_style=True)]
    elif args.operation == 'goal':
        cmd = ['ros2','action','send_goal',endpoint+'/follow_joint_trajectory',
               'control_msgs/action/FollowJointTrajectory',
               yaml.safe_dump({'trajectory':practice_message(args.pose,args.duration)},default_flow_style=True),
               '--feedback']
    elif args.operation == 'cancel':
        # Standard CancelGoal service with zero UUID/stamp cancels every goal
        # on this one controller, per the ROS action protocol.
        request = {'goal_info':{'goal_id':{'uuid':[0]*16},'stamp':{'sec':0,'nanosec':0}}}
        cmd = ['ros2','service','call',endpoint+'/follow_joint_trajectory/_action/cancel_goal',
               'action_msgs/srv/CancelGoal',yaml.safe_dump(request,default_flow_style=True)]
    else:
        raise ValueError('unsupported control operation')
    return subprocess.run(cmd,check=False,timeout=args.timeout).returncode


def measure(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from control_msgs.msg import JointTrajectoryControllerState
    from rosgraph_msgs.msg import Clock
    rclpy.init(args=[])
    node = Node('l05_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    data = {'schema_version':1,'source':'live_ros_gazebo_observation','namespace':args.namespace,
            'controller':CONTROLLER_NAME,'joint_names':list(JOINT_NAMES),'controller_samples':[],
            'joint_samples':[],'completed':False,'started_unix':time.time(),
            'environment':{'ros_distro':os.environ.get('ROS_DISTRO'),
                'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'gz_partition':os.environ.get('GZ_PARTITION')}}
    clock = None
    state = None
    collecting = False
    def on_clock(msg):
        nonlocal clock
        clock = msg.clock.sec+msg.clock.nanosec*1e-9
    def on_joint(msg):
        if collecting:
            data['joint_samples'].append({'t':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'joint_names':list(msg.name),'positions':list(msg.position),'velocities':list(msg.velocity)})
    def on_state(msg):
        nonlocal state
        state = msg
        if collecting:
            reference = getattr(msg,'reference',getattr(msg,'desired',None))
            feedback = getattr(msg,'feedback',getattr(msg,'actual',None))
            data['controller_samples'].append({'t':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'joint_names':list(msg.joint_names),'reference':list(reference.positions),
                'actual':list(feedback.positions),'error':list(msg.error.positions),
                'velocities':list(feedback.velocities)})
    node.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data)
    node.create_subscription(JointState,f'/{args.namespace}/joint_states',on_joint,qos_profile_sensor_data)
    node.create_subscription(JointTrajectoryControllerState,
        f'/{args.namespace}/{CONTROLLER_NAME}/controller_state',on_state,qos_profile_sensor_data)
    deadline = time.monotonic()+args.timeout
    try:
        while clock is None or state is None:
            if time.monotonic()>=deadline:
                raise RuntimeError('clock/controller state unavailable before wall deadline')
            rclpy.spin_once(node,timeout_sec=0.05)
        start = clock
        data['window_start_sim_s'] = start
        collecting = True
        deadline = time.monotonic()+args.timeout
        while clock-start<args.duration:
            if time.monotonic()>=deadline:
                raise RuntimeError('simulation stalled or too slow before wall deadline')
            if clock<start:
                raise RuntimeError('simulation clock moved backwards during measurement')
            rclpy.spin_once(node,timeout_sec=0.05)
        collecting = False
        data['summary'] = summarize(data)
        data.update(completed=True,window_end_sim_s=clock)
    except (Exception,KeyboardInterrupt) as error:
        data['error'] = f'{type(error).__name__}: {error}'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        data['finished_unix'] = time.time()
        path = Path(args.output).expanduser()
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(path),'completed':data['completed'],
                      'summary':data.get('summary'),'error':data.get('error')},indent=2))
    return 0 if data['completed'] else 1


def summarize(data):
    """Descriptive statistics of measured state, not an authoritative grade."""
    rows = data.get('controller_samples',[])
    if not rows: raise ValueError('no controller samples')
    results = {}
    for joint in JOINT_NAMES:
        errors, actual = [],[]
        for row in rows:
            if joint not in row['joint_names']: continue
            index = row['joint_names'].index(joint)
            if index>=len(row['reference']) or index>=len(row['actual']): continue
            a,b = row['reference'][index],row['actual'][index]
            if not math.isfinite(a) or not math.isfinite(b):
                raise ValueError(f'nonfinite observation for {joint}')
            errors.append(a-b)
            actual.append(b)
        if errors:
            low,high = JOINT_LIMITS[joint]
            results[joint] = {'max_abs_error_rad':max(map(abs,errors)),
                'rms_error_rad':math.sqrt(sum(x*x for x in errors)/len(errors)),
                'final_error_rad':errors[-1],'measured_min_rad':min(actual),'measured_max_rad':max(actual),
                'within_model_limits':min(actual)>=low-0.01 and max(actual)<=high+0.01}
    return {'controller_samples':len(rows),'joint_samples':len(data.get('joint_samples',[])),
            'observed_duration_sim_s':rows[-1]['t']-rows[0]['t'],
            'missing_joints':[name for name in JOINT_NAMES if name not in results],
            'joints':results}


def plot(data,output,joints=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    joints = joints or ['left_shoulder_pitch','left_elbow_pitch','right_shoulder_pitch','right_elbow_pitch']
    rows = data['controller_samples']
    if not rows: raise ValueError('no controller samples to plot')
    fig,axes = plt.subplots(len(joints),1,figsize=(9,2.2*len(joints)),sharex=True,constrained_layout=True)
    if len(joints)==1: axes=[axes]
    start = rows[0]['t']
    for ax,joint in zip(axes,joints):
        good = [row for row in rows if joint in row['joint_names']]
        times,reference,actual = [],[],[]
        for row in good:
            index = row['joint_names'].index(joint)
            if index>=len(row['reference']) or index>=len(row['actual']): continue
            times.append(row['t']-start)
            reference.append(row['reference'][index])
            actual.append(row['actual'][index])
        ax.plot(times,reference,'--',label='controller reference')
        ax.plot(times,actual,label='measured position')
        ax.set_ylabel('Position [rad]')
        ax.set_title(joint)
        ax.grid(True,alpha=0.3)
    axes[0].legend()
    axes[-1].set_xlabel('Elapsed simulation time [s]')
    fig.suptitle(f"Live controller observation /{data['namespace']}/{CONTROLLER_NAME}")
    path=Path(output).expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action',required=True)
    ctrl = subs.add_parser('control')
    ctrl.add_argument('operation',choices=['list','interfaces','pose','goal','cancel'])
    ctrl.add_argument('pose',nargs='?',choices=['neutral','reach'],default='neutral')
    for p in (ctrl,):
        p.add_argument('--namespace',default='humanoid')
        p.add_argument('--duration',type=float,default=3.0)
        p.add_argument('--timeout',type=float,default=60.0)
    obs = subs.add_parser('measure')
    obs.add_argument('--namespace',default='humanoid')
    obs.add_argument('--duration',type=float,default=6.0)
    obs.add_argument('--timeout',type=float,default=60.0)
    obs.add_argument('--output',default='/workspace/evidence/practice.json')
    for operation in ('report','plot'):
        p = subs.add_parser(operation)
        p.add_argument('--input',required=True)
        if operation=='plot':
            p.add_argument('--output',required=True)
            p.add_argument('--joints',nargs='+',choices=JOINT_NAMES)
    args = parser.parse_args(argv)
    if hasattr(args,'namespace'): args.namespace = namespace_name(args.namespace)
    if hasattr(args,'duration') and (not math.isfinite(args.duration) or not 0.2<=args.duration<=60):
        parser.error('duration must be 0.2..60 simulation seconds')
    if hasattr(args,'timeout') and (not math.isfinite(args.timeout) or not 0<args.timeout<=300):
        parser.error('timeout must be 0..300 wall seconds')
    try:
        if args.action=='control': return control(args)
        if args.action=='measure': return measure(args)
        data = json.loads(Path(args.input).expanduser().read_text())
        if args.action=='report': print(json.dumps(summarize(data),indent=2))
        else: plot(data,args.output,args.joints)
        return 0
    except (ValueError,RuntimeError,OSError,subprocess.TimeoutExpired) as error:
        print(f'{type(error).__name__}: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
