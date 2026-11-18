"""System, MuJoCo, optional RaiSim, GPU, ROS 2, and SDK contract diagnostics (`./xrobot3 diagnose`)."""
import importlib.util
import json
import os
import platform
import subprocess
import sys
from pathlib import Path


def _check_python_module(name):
    return importlib.util.find_spec(name) is not None


def _probe_mujoco(python_exe, scene_xml):
    if not os.access(python_exe, os.X_OK):
        return {'installed': False, 'ok': False, 'version': None, 'detail': 'python executable not found'}
    code = (
        'import sys, mujoco; '
        'm = mujoco.MjModel.from_xml_path(sys.argv[1]); '
        'print(mujoco.__version__, m.nq, m.nv)'
    )
    try:
        res = subprocess.run([str(python_exe), '-c', code, str(scene_xml)], capture_output=True, text=True, timeout=10)
    except Exception as exc:
        return {'installed': True, 'ok': False, 'version': None, 'detail': str(exc)}
    if res.returncode != 0:
        err = (res.stderr or res.stdout or '').strip().splitlines()
        return {'installed': True, 'ok': False, 'version': None, 'detail': err[-1] if err else 'mujoco import failed'}
    parts = res.stdout.strip().split()
    ver = parts[0] if parts else 'unknown'
    nq_nv = f'nq={parts[1]}, nv={parts[2]}' if len(parts) >= 3 else ''
    return {'installed': True, 'ok': True, 'version': ver, 'detail': nq_nv}


def _probe_gpu():
    try:
        res = subprocess.run(
            [
                'nvidia-smi',
                '--query-gpu=name,enforced.power.limit,power.default_limit,clocks_event_reasons.sw_thermal_slowdown',
                '--format=csv,noheader,nounits',
            ],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if res.returncode != 0 or not res.stdout.strip():
            return {'present': False, 'throttled': False, 'detail': 'no NVIDIA GPU or nvidia-smi unavailable'}
        line = res.stdout.strip().splitlines()[0]
        name, limit, default, thermal = [x.strip() for x in line.split(',')]
        throttled = float(limit) < 0.5 * float(default) or thermal == 'Active'
        return {
            'present': True,
            'name': name,
            'power_limit_w': float(limit),
            'default_power_w': float(default),
            'thermal_slowdown': thermal,
            'throttled': throttled,
            'detail': f'{name} ({float(limit):.0f}/{float(default):.0f} W, thermal={thermal})',
        }
    except Exception as exc:
        return {'present': False, 'throttled': False, 'detail': f'unavailable ({exc})'}


def run_diagnostics(root_dir, strict_raisim=False):
    """Inspect all runtime subsystems without throwing if optional RaiSim is absent."""
    root = Path(root_dir).resolve()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    sdk_a_dir = root / 'sdk' / 'policy' / 'sdk_a_to'
    if str(sdk_a_dir) not in sys.path:
        sys.path.insert(0, str(sdk_a_dir))

    from network import interfaces
    from raisim_bridge import probe_raisim

    # 1. ROS 2 & core Python packages
    ros_pkgs = {pkg: _check_python_module(pkg) for pkg in ('rclpy', 'onnxruntime', 'bodyctrl_msgs')}
    env_info = {
        'ok': all(ros_pkgs.values()),
        'platform': f'{platform.system()} {platform.machine()}',
        'python': sys.version.split()[0],
        'executable': sys.executable,
        'ros_distro': os.environ.get('ROS_DISTRO') or '(not sourced)',
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '(unset)'),
        'display': os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or '(headless)',
        'packages': ros_pkgs,
    }

    # 2. Network adapters
    try:
        nics = interfaces()
        net_info = {'ok': any(i['name'] == 'lo' for i in nics), 'interfaces': nics}
    except Exception as exc:
        net_info = {'ok': False, 'interfaces': [], 'error': str(exc)}

    # 3. Contracts & scenes
    required_files = [
        sdk_a_dir / 'deploy.json',
        sdk_a_dir / 'policy.onnx',
        sdk_a_dir / 'reference.npz',
        sdk_a_dir / 'fixtures.npz',
        sdk_a_dir / 'scene_unitree_visual.xml',
        root / 'sdk' / 'config' / 'sdk_a_sim.yaml',
        root / 'sdk' / 'config' / 'sdk_a_hardware.yaml',
    ]
    missing = [str(p.relative_to(root)) for p in required_files if not p.is_file()]
    joint_count = 0
    if (sdk_a_dir / 'deploy.json').is_file():
        try:
            joint_count = len(json.loads((sdk_a_dir / 'deploy.json').read_text()).get('joint_names', []))
        except Exception:
            joint_count = 0
    contracts_info = {
        'ok': (not missing) and joint_count == 29,
        'missing_files': missing,
        'joint_count': joint_count,
    }

    # 4. MuJoCo default backend (4.0 + 3.3.7)
    scene_xml = sdk_a_dir / 'scene_unitree_visual.xml'
    mj4 = _probe_mujoco(root / '.venv-mj4' / 'bin' / 'python', scene_xml)
    mj3 = _probe_mujoco(root / '.venv' / 'bin' / 'python' if os.access(root / '.venv' / 'bin' / 'python', os.X_OK) else sys.executable, scene_xml)
    mujoco_info = {
        'ok': mj4['ok'] or mj3['ok'],
        'default_version': '4.0' if mj4['ok'] else ('3.3.7' if mj3['ok'] else None),
        'v4_0': mj4,
        'v3_3_7': mj3,
    }

    # 5. Optional RaiSim backend
    raisim_info = probe_raisim(root)

    # 6. GPU render health
    gpu_info = _probe_gpu()

    core_ok = env_info['ok'] and contracts_info['ok'] and mujoco_info['ok']
    overall_ok = core_ok and (not strict_raisim or raisim_info['available'])

    return {
        'overall_ok': overall_ok,
        'core_ok': core_ok,
        'strict_raisim': bool(strict_raisim),
        'environment': env_info,
        'gpu': gpu_info,
        'network': net_info,
        'contracts': contracts_info,
        'mujoco': mujoco_info,
        'raisim': raisim_info,
    }


def format_report(report):
    lines = ['Xrobot3 Diagnostics', '=' * 60]
    env = report['environment']
    pkg_str = ', '.join(f'{k}={"OK" if v else "MISSING"}' for k, v in env['packages'].items())
    lines.append(
        f"[{'PASS' if env['ok'] else 'FAIL'}] Environment : {env['platform']} | Py {env['python']} | ROS={env['ros_distro']} | Display={env['display']}"
    )
    lines.append(f"       Modules     : {pkg_str}")

    gpu = report['gpu']
    gpu_tag = 'WARN' if gpu.get('throttled') else 'INFO'
    lines.append(f"[{gpu_tag}] GPU         : {gpu['detail']}")
    if gpu.get('throttled'):
        lines.append('       Hint        : Power/thermal throttling detected; viewer FPS may drop. Enable Performance mode / nvidia-powerd.')

    net = report['network']
    ifaces = ', '.join(f"{i['name']}({i['kind']})" for i in net['interfaces']) or '(none)'
    lines.append(f"[{'PASS' if net['ok'] else 'WARN'}] Network/DDS : {ifaces}")

    con = report['contracts']
    c_detail = f"{con['joint_count']} joints, policy/scene/configs present" if con['ok'] else f"missing={con['missing_files']} joints={con['joint_count']}"
    lines.append(f"[{'PASS' if con['ok'] else 'FAIL'}] Contracts   : {c_detail}")

    mj = report['mujoco']
    v4 = mj['v4_0']
    v3 = mj['v3_3_7']
    lines.append(
        f"[{'PASS' if mj['ok'] else 'FAIL'}] MuJoCo (default): 4.0={'OK (' + v4['version'] + ')' if v4['ok'] else 'N/A (' + v4['detail'] + ')'}; "
        f"3.3.7={'OK (' + v3['version'] + ')' if v3['ok'] else 'N/A (' + v3['detail'] + ')'}"
    )

    rs = report['raisim']
    rs_tag = 'PASS' if rs['available'] else ('FAIL' if report['strict_raisim'] else 'WARN')
    rs_label = rs['status'].upper()
    lines.append(
        f"[{rs_tag}] RaiSim (optional): {rs_label}" + (f" | {rs['detail']}" if rs['detail'] else '')
    )
    if rs.get('raisim_root'):
        lines.append(f"       Root/Python : {rs['raisim_root']} | {rs.get('python')}")
    if rs.get('activation_key'):
        lic = rs.get('license_info') or {}
        extra = f" (expiry={lic['expiry_date']}, type={lic.get('license_type', '?')})" if lic.get('expiry_date') else ''
        lines.append(f"       License Key : {rs['activation_key']}{extra}")
    if rs.get('hint'):
        lines.append(f"       Hint        : {rs['hint']}")

    lines.append('-' * 60)
    if report['overall_ok']:
        if rs['available']:
            lines.append('Summary: READY (MuJoCo + RaiSim backends both ready).')
        else:
            lines.append('Summary: READY for default MuJoCo & hardware (optional RaiSim not active; `--physics mujoco` unaffected).')
    else:
        lines.append('Summary: NOT READY (fix FAIL items above; run `./xrobot3 check` for deep ONNX/torque check).')
    return '\n'.join(lines)
