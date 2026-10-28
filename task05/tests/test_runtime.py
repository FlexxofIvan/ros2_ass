"""Validate asset/scaffolding contracts; ROS/Gazebo needs separate live QA."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from control_lab.model import JOINT_NAMES, ARM_JOINT_NAMES, LEG_JOINT_NAMES, JOINT_LIMITS, generate
from control_lab.pid import simulate
from control_lab.cli import practice_message, summarize


def test_original_robot_contract(tmp_path):
    paths = generate('practice','group_a/humanoid',tmp_path)
    robot = ET.parse(paths['robot_urdf']).getroot()
    moving = robot.findall("./joint[@type='revolute']")
    assert len(moving)==18
    assert {j.attrib['name'] for j in moving}==set(JOINT_NAMES)
    assert len(ARM_JOINT_NAMES)==8 and len(LEG_JOINT_NAMES)==10
    assert robot.find("./joint[@name='world_to_pelvis']/parent").attrib['link']=='world'
    assert not robot.findall('.//mesh')
    assert {j.attrib['name'] for j in robot.findall('./ros2_control/joint')}==set(JOINT_NAMES)
    for joint in moving:
        limit = joint.find('limit')
        assert tuple(float(limit.attrib[k]) for k in ('lower','upper'))==JOINT_LIMITS[joint.attrib['name']]
    config=yaml.safe_load(Path(paths['controller_yaml']).read_text())
    assert config['/**/humanoid_controller']['ros__parameters']['joints']==list(JOINT_NAMES)
    assert robot.find('./gazebo/plugin/ros/namespace').text=='/group_a/humanoid'
    ET.parse(paths['world_sdf'])


@pytest.mark.parametrize('mode',['homework','other',''])
@pytest.mark.parametrize('explicit_config',[False,True])
def test_runtime_rejects_nonpractice_modes(tmp_path,mode,explicit_config):
    source=tmp_path/'input.yaml'
    source.write_text('/**/controller_manager:\n  ros__parameters:\n    update_rate: 50\n')
    output=tmp_path/'output'
    with pytest.raises(ValueError,match='mode must be practice'):
        generate(mode,'humanoid',output,source if explicit_config else None)
    assert not output.exists()


def test_practice_can_use_explicit_controller_config(tmp_path):
    source=tmp_path/'input.yaml'
    source.write_text('/**/controller_manager:\n  ros__parameters:\n    update_rate: 50\n')
    paths=generate('practice','humanoid',tmp_path/'output',source)
    data=yaml.safe_load(Path(paths['controller_yaml']).read_text())
    assert data==yaml.safe_load(source.read_text())


@pytest.mark.parametrize('namespace',['','../other','bad-name','a//b','a b','1a'])
def test_namespace_rejects_paths(tmp_path,namespace):
    with pytest.raises(ValueError): generate('practice',namespace,tmp_path)


def test_practice_pose_uses_complete_order_and_safe_limits():
    for pose in ('neutral','reach'):
        msg=practice_message(pose,3.25)
        assert tuple(msg['joint_names'])==JOINT_NAMES
        point=msg['points'][0]
        assert point['time_from_start']=={'sec':3,'nanosec':250_000_000}
        for name,value in zip(JOINT_NAMES,point['positions']):
            assert JOINT_LIMITS[name][0]<=value<=JOINT_LIMITS[name][1]
        assert point['positions'][8:]==[0.0]*10


def test_pd_reduces_overshoot_and_integral_rejects_constant_load():
    p,pd,pid=(simulate(profile,duration=12) for profile in ('p','pd','pid'))
    assert pd['metrics']['overshoot_rad'] < p['metrics']['overshoot_rad']
    # With zero integral, equilibrium torque Kp*error balances constant load.
    parameters=pd['parameters']
    assert pd['metrics']['final_error_rad']==pytest.approx(
        parameters['disturbance_Nm']/parameters['kp'],abs=1e-4)
    assert abs(pid['metrics']['final_error_rad']) < abs(pd['metrics']['final_error_rad'])
    assert abs(pid['metrics']['final_error_rad']) < 0.001
    assert pd['source']=='offline_single_axis_effort_model'
    assert all(abs(r['effort'])<=pid['parameters']['torque_limit_Nm'] for r in pid['samples'])
    assert all(math.isfinite(r['position']) for r in pid['samples'])


@pytest.mark.parametrize('kwargs',[{'kp':float('nan')},{'control_rate':99},{'torque_limit':0},{'duration':100}])
def test_pid_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError): simulate(**kwargs)


def test_observation_matches_by_joint_name_not_message_order():
    data={'controller_samples':[{'t':1.0,'joint_names':['left_elbow_pitch','left_shoulder_pitch'],
                                 'reference':[-0.4,-0.2],'actual':[-0.3,-0.1]},
                                {'t':2.0,'joint_names':['left_shoulder_pitch','left_elbow_pitch'],
                                 'reference':[-0.2,-0.4],'actual':[-0.15,-0.38]}]}
    result=summarize(data)
    assert result['observed_duration_sim_s']==1
    assert result['joints']['left_elbow_pitch']['final_error_rad']==pytest.approx(-0.02)
    assert result['joints']['left_shoulder_pitch']['max_abs_error_rad']==pytest.approx(0.1)
