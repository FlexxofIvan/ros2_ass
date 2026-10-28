"""Offline effort-control experiment: J*qddot = u - b*qdot - disturbance.

This deliberately simple, one-axis model is not Gazebo or a humanoid joint.
The plant integrates at 1 kHz; feedback is sampled and held at a configurable
rate. Derivative uses measured velocity, avoiding a reference-step kick.
"""
import argparse
import csv
import json
import math
from pathlib import Path

PROFILES = {'p': (6.0,0.0,0.0), 'pd': (6.0,1.3,0.0), 'pid': (6.0,1.3,3.0)}


def simulate(profile='pd', duration=6.0, kp=None, kd=None, ki=None,
             torque_limit=2.0, control_rate=1000, anti_windup=True):
    if profile not in PROFILES:
        raise ValueError('profile must be p, pd, or pid')
    defaults = PROFILES[profile]
    gains = tuple(default if value is None else float(value)
                  for value,default in zip((kp,kd,ki),defaults))
    kp,kd,ki = gains
    if not all(math.isfinite(v) and v>=0 for v in gains):
        raise ValueError('gains must be finite and nonnegative')
    if not math.isfinite(duration) or not 0.2<=duration<=60:
        raise ValueError('duration must be 0.2..60 s')
    if not math.isfinite(torque_limit) or not 0<torque_limit<=10:
        raise ValueError('torque_limit must be 0..10 Nm')
    if not isinstance(control_rate,int) or not 1<=control_rate<=1000 or 1000%control_rate:
        raise ValueError('control_rate must be a positive divisor of 1000 Hz')
    dt = 0.001
    period = 1000//control_rate
    q, velocity, integral, effort, unsaturated = 0.0,0.0,0.0,0.0,0.0
    reference, inertia, damping, disturbance = 0.6,0.08,0.12,0.3
    rows = []
    saturated = 0
    for step in range(round(duration/dt)+1):
        t = step*dt
        target = reference
        if step%period == 0:
            error = target-q
            candidate = integral+error/control_rate
            unsaturated = kp*error-kd*velocity+ki*candidate
            effort = max(-torque_limit,min(torque_limit,unsaturated))
            # Conditional integration: permit unwinding, block growth farther
            # into saturation. Compare with --windup to expose the difference.
            if not anti_windup or effort == unsaturated or error*unsaturated<=0:
                integral = candidate
            else:
                unsaturated = kp*error-kd*velocity+ki*integral
                effort = max(-torque_limit,min(torque_limit,unsaturated))
            saturated += int(abs(unsaturated)>torque_limit)
            rows.append({'t':t,'reference':target,'position':q,'velocity':velocity,
                         'error':target-q,'effort':effort,'unsaturated_effort':unsaturated,
                         'integral_error':integral})
        acceleration = (effort-damping*velocity-disturbance)/inertia
        velocity += acceleration*dt
        q += velocity*dt
        if not all(math.isfinite(x) for x in (q,velocity,integral)):
            raise ValueError('numerical divergence; reduce gains/rate combination')
    tail = [r for r in rows if r['t']>=max(0,duration-1)]
    metrics = {'final_error_rad':rows[-1]['error'],
               'tail_rms_error_rad':math.sqrt(sum(r['error']**2 for r in tail)/len(tail)),
               'overshoot_rad':max(0,max(r['position'] for r in rows)-reference),
               'saturation_fraction':saturated/len(rows)}
    return {'schema_version':1,'source':'offline_single_axis_effort_model','profile':profile,
            'parameters':{'kp':kp,'kd':kd,'ki':ki,'inertia_kg_m2':inertia,
                'damping_Nm_s_rad':damping,'disturbance_Nm':disturbance,
                'torque_limit_Nm':torque_limit,'control_rate_Hz':control_rate,
                'plant_rate_Hz':1000,'anti_windup':anti_windup,'reference_rad':reference},
            'samples':rows,'metrics':metrics}


def plot(data, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = data['samples']
    fig, axes = plt.subplots(3,1,figsize=(9,7),sharex=True,constrained_layout=True)
    times = [r['t'] for r in rows]
    axes[0].plot(times,[r['reference'] for r in rows],'--',label='reference')
    axes[0].plot(times,[r['position'] for r in rows],label='measured model position')
    axes[0].set_ylabel('Position [rad]')
    axes[0].legend()
    axes[1].plot(times,[r['effort'] for r in rows],label='applied effort')
    axes[1].plot(times,[r['unsaturated_effort'] for r in rows],':',label='unsaturated request')
    axes[1].set_ylabel('Torque [N m]')
    axes[1].legend()
    axes[2].plot(times,[r['error'] for r in rows],label='tracking error')
    axes[2].set_ylabel('Error [rad]')
    axes[2].set_xlabel('Model time [s]')
    for ax in axes: ax.grid(True,alpha=0.3)
    fig.suptitle(f"Offline {data['profile'].upper()} effort control (not humanoid telemetry)")
    output = Path(output).expanduser()
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=PROFILES,default='pd')
    parser.add_argument('--duration',type=float,default=6.0)
    for key in ('kp','kd','ki'): parser.add_argument(f'--{key}',type=float)
    parser.add_argument('--torque-limit',type=float,default=2.0)
    parser.add_argument('--control-rate',type=int,default=1000)
    parser.add_argument('--windup',action='store_true',help='disable conditional anti-windup')
    parser.add_argument('--output',default='/workspace/evidence/pid.json')
    parser.add_argument('--plot')
    args = parser.parse_args(argv)
    try:
        data = simulate(args.profile,args.duration,args.kp,args.kd,args.ki,
                        args.torque_limit,args.control_rate,not args.windup)
    except ValueError as error:
        parser.error(str(error))
    path = Path(args.output).expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    with path.with_suffix('.csv').open('w',newline='') as file:
        writer = csv.DictWriter(file,fieldnames=data['samples'][0])
        writer.writeheader()
        writer.writerows(data['samples'])
    if args.plot: plot(data,args.plot)
    print(json.dumps({'output':str(path),'metrics':data['metrics']},indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
