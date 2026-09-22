#!/usr/bin/env python3

"""Build a MuJoCo MJCF model from the SolidWorks URDF export."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import struct
import time
import xml.etree.ElementTree as ET

from common import RESULTS_ROOT, default_config_path, default_urdf_path, load_config
import numpy as np
from scipy.spatial.transform import Rotation


MATERIAL_COLORS = (
    '0.76 0.78 0.80 1',
    '0.18 0.42 0.62 1',
    '0.82 0.25 0.22 1',
    '0.32 0.56 0.39 1',
)


def _numbers(value, count):
    result = np.fromstring(value, sep=' ', dtype=float)
    if result.size != count:
        raise ValueError(f'expected {count} numbers, got {value!r}')
    return result


def _origin(element):
    if element is None:
        return np.zeros(3), np.asarray([1.0, 0.0, 0.0, 0.0])
    xyz = _numbers(element.get('xyz', '0 0 0'), 3)
    rpy = _numbers(element.get('rpy', '0 0 0'), 3)
    xyzw = Rotation.from_euler('xyz', rpy).as_quat()
    return xyz, np.roll(xyzw, 1)


def _vector(values):
    return ' '.join(f'{float(value):.12g}' for value in values)


def _mesh_path(filename: str, urdf_path: Path) -> Path:
    prefix = 'package://装配体.SLDASM/'
    if filename.startswith(prefix):
        return (urdf_path.parents[1] / filename[len(prefix):]).resolve()
    candidate = Path(filename)
    if not candidate.is_absolute():
        candidate = urdf_path.parent / candidate
    return candidate.resolve()


def _split_large_binary_stl(path: Path, output_path: Path, max_faces=190000):
    asset_dir = output_path.parent / f'{output_path.stem}_assets'
    asset_dir.mkdir(parents=True, exist_ok=True)
    with path.open('rb') as stream:
        header = stream.read(80)
        count_bytes = stream.read(4)
        if len(count_bytes) != 4:
            raise ValueError(f'invalid binary STL: {path}')
        face_count = struct.unpack('<I', count_bytes)[0]
        faces = stream.read()
    if len(faces) != 50 * face_count:
        raise ValueError(f'only binary STL meshes are supported: {path}')
    if face_count <= max_faces:
        destination = asset_dir / path.name
        shutil.copy2(path, destination)
        return [destination]
    result = []
    for part, start in enumerate(range(0, face_count, max_faces), start=1):
        count = min(max_faces, face_count - start)
        destination = asset_dir / f'{path.stem}_part{part}.stl'
        with destination.open('wb') as stream:
            stream.write(header)
            stream.write(struct.pack('<I', count))
            stream.write(faces[50 * start:50 * (start + count)])
        result.append(destination)
    return result


def convert_urdf_to_mjcf(
    urdf_path: Path,
    output_path: Path,
    gravity=(0.0, 0.0, -9.81),
    timestep=0.002,
    base_position=(0.0, 0.0, 0.0),
    base_orientation_rpy=(0.0, 0.0, 0.0),
    show_frames=False,
    free_base=False,
    enable_contacts=False,
    joint_damping=None,
    joint_armature=None,
    joint_friction_loss=None,
):
    urdf_path = Path(urdf_path).resolve()
    output_path = Path(output_path).resolve()
    gravity = np.asarray(gravity, dtype=float)
    base_position = np.asarray(base_position, dtype=float)
    base_orientation_rpy = np.asarray(base_orientation_rpy, dtype=float)
    for name, values in (
            ('gravity', gravity),
            ('base_position', base_position),
            ('base_orientation_rpy', base_orientation_rpy)):
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError(f'{name} must contain three finite values')
    robot = ET.parse(urdf_path).getroot()
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    if not links or not joints:
        raise ValueError(f'{urdf_path} has no links or joints')

    children = {}
    child_names = set()
    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        children.setdefault(parent, []).append(joint)
        child_names.add(child)
    roots = set(links) - child_names
    if len(roots) != 1:
        raise ValueError(f'expected one root link, found {sorted(roots)}')
    root_link = roots.pop()

    mujoco = ET.Element('mujoco', {'model': 'exo7'})
    ET.SubElement(mujoco, 'compiler', {
        'angle': 'radian',
        'autolimits': 'true',
        'balanceinertia': 'true',
        'inertiafromgeom': 'false',
    })
    ET.SubElement(mujoco, 'option', {
        'timestep': f'{float(timestep):.12g}',
        'gravity': _vector(gravity),
        'integrator': 'implicitfast',
    })
    visual = ET.SubElement(mujoco, 'visual')
    ET.SubElement(visual, 'global', {'offwidth': '960', 'offheight': '720'})
    joint_count = len(joints)

    def joint_values(value, name):
        if value is None:
            return None
        if np.isscalar(value):
            values = np.full(joint_count, float(value), dtype=float)
        else:
            values = np.asarray(value, dtype=float).reshape(-1)
            if values.shape != (joint_count,):
                raise ValueError(
                    f'{name} must be a scalar or contain {joint_count} values')
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            raise ValueError(f'{name} must contain finite non-negative values')
        return values

    damping_values = joint_values(joint_damping, 'joint_damping')
    armature_values = joint_values(joint_armature, 'joint_armature')
    friction_values = joint_values(
        joint_friction_loss, 'joint_friction_loss')
    custom_joint_dynamics = any(
        values is not None
        for values in (damping_values, armature_values, friction_values)
    )
    if custom_joint_dynamics:
        damping_values = (
            np.zeros(joint_count) if damping_values is None else damping_values)
        armature_values = (
            np.zeros(joint_count)
            if armature_values is None else armature_values)
        friction_values = (
            np.zeros(joint_count)
            if friction_values is None else friction_values)

    default = ET.SubElement(mujoco, 'default')
    ET.SubElement(default, 'joint', {
        'damping': '0' if custom_joint_dynamics else '0.02',
        'armature': '0' if custom_joint_dynamics else '0.002',
    })
    ET.SubElement(default, 'geom', {
        'contype': '1' if enable_contacts else '0',
        'conaffinity': '1' if enable_contacts else '0',
        'group': '1',
    })

    asset = ET.SubElement(mujoco, 'asset')
    mesh_names = {}
    for link_index, (link_name, link) in enumerate(links.items()):
        visual = link.find('visual')
        mesh = visual.find('geometry/mesh') if visual is not None else None
        if mesh is None:
            continue
        path = _mesh_path(mesh.get('filename'), urdf_path)
        if not path.exists():
            raise FileNotFoundError(f'mesh for {link_name} does not exist: {path}')
        mesh_names[link_name] = []
        for part, mesh_path in enumerate(
                _split_large_binary_stl(path, output_path), start=1):
            mesh_name = f'{link_name}_mesh_{part}'
            mesh_names[link_name].append(mesh_name)
            ET.SubElement(
                asset, 'mesh', {
                    'name': mesh_name,
                    'file': os.path.relpath(mesh_path, output_path.parent),
                })
        ET.SubElement(asset, 'material', {
            'name': f'{link_name}_material',
            'rgba': MATERIAL_COLORS[link_index % len(MATERIAL_COLORS)],
            'specular': '0.25',
            'shininess': '0.35',
        })

    world = ET.SubElement(mujoco, 'worldbody')
    ET.SubElement(world, 'light', {
        'name': 'key', 'pos': '0 -2 3', 'dir': '0 0.5 -1',
        'diffuse': '0.8 0.8 0.8',
    })
    ET.SubElement(world, 'light', {
        'name': 'fill', 'pos': '1 2 2', 'dir': '-0.4 -0.5 -1',
        'diffuse': '0.45 0.45 0.45',
    })
    ET.SubElement(world, 'geom', {
        'name': 'floor', 'type': 'plane', 'size': '3 3 0.1',
        'pos': '0 0 -0.55', 'rgba': '0.18 0.20 0.22 1',
        'contype': '1' if enable_contacts else '0',
        'conaffinity': '1' if enable_contacts else '0',
        'group': '2',
    })
    if show_frames:
        for name, endpoint, color in (
                ('world_x', '0.25 0 0', '0.9 0.1 0.1 1'),
                ('world_y', '0 0.25 0', '0.1 0.8 0.2 1'),
                ('world_z', '0 0 0.25', '0.1 0.3 1.0 1')):
            ET.SubElement(world, 'geom', {
                'name': f'{name}_axis', 'type': 'capsule',
                'fromto': f'0 0 0 {endpoint}', 'size': '0.006',
                'rgba': color, 'contype': '0', 'conaffinity': '0',
                'group': '0',
            })
        if np.linalg.norm(gravity) > 0.0:
            start = base_position + np.asarray([0.32, 0.0, 0.32])
            stop = start + 0.25 * gravity / np.linalg.norm(gravity)
            ET.SubElement(world, 'geom', {
                'name': 'gravity_direction', 'type': 'capsule',
                'fromto': _vector(np.r_[start, stop]), 'size': '0.008',
                'rgba': '1 0.8 0.05 1', 'contype': '0',
                'conaffinity': '0', 'group': '0',
            })
            ET.SubElement(world, 'geom', {
                'name': 'gravity_tip', 'type': 'sphere',
                'pos': _vector(stop), 'size': '0.016',
                'rgba': '1 0.8 0.05 1', 'contype': '0',
                'conaffinity': '0', 'group': '0',
            })

    actuator = ET.SubElement(mujoco, 'actuator')

    base_quaternion = np.roll(
        Rotation.from_euler('xyz', base_orientation_rpy).as_quat(), 1)

    joint_parameters = {
        joint.get('name'): index
        for index, joint in enumerate(joints)
    }

    def add_link(link_name, parent_element, incoming_joint=None, depth=0):
        attributes = {'name': link_name}
        if incoming_joint is None:
            attributes['pos'] = _vector(base_position)
            attributes['quat'] = _vector(base_quaternion)
        else:
            position, quaternion = _origin(incoming_joint.find('origin'))
            attributes['pos'] = _vector(position)
            attributes['quat'] = _vector(quaternion)
        body = ET.SubElement(parent_element, 'body', attributes)
        if incoming_joint is None and free_base:
            ET.SubElement(body, 'freejoint', {'name': 'base_freejoint'})
        if incoming_joint is None and show_frames:
            for name, endpoint, color in (
                    ('base_x', '0.20 0 0', '0.95 0.2 0.2 1'),
                    ('base_y', '0 0.20 0', '0.2 0.9 0.3 1'),
                    ('base_z', '0 0 0.20', '0.2 0.4 1.0 1')):
                ET.SubElement(body, 'geom', {
                    'name': f'{name}_axis', 'type': 'capsule',
                    'fromto': f'0 0 0 {endpoint}', 'size': '0.008',
                    'rgba': color, 'contype': '0', 'conaffinity': '0',
                    'group': '0',
                })
        if incoming_joint is not None:
            joint_type = incoming_joint.get('type')
            if joint_type not in ('continuous', 'revolute'):
                raise ValueError(
                    f'unsupported joint type {joint_type!r} for '
                    f'{incoming_joint.get("name")}')
            axis = _numbers(incoming_joint.find('axis').get('xyz'), 3)
            joint_attributes = {
                'name': incoming_joint.get('name'),
                'type': 'hinge',
                'axis': _vector(axis),
                'limited': 'false',
            }
            if custom_joint_dynamics:
                joint_index = joint_parameters[incoming_joint.get('name')]
                joint_attributes.update({
                    'damping': f'{damping_values[joint_index]:.12g}',
                    'armature': f'{armature_values[joint_index]:.12g}',
                    'frictionloss': f'{friction_values[joint_index]:.12g}',
                })
            ET.SubElement(body, 'joint', joint_attributes)
            ET.SubElement(actuator, 'motor', {
                'name': f'{incoming_joint.get("name")}_motor',
                'joint': incoming_joint.get('name'),
                'gear': '1',
            })

        link = links[link_name]
        inertial = link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin')
            position, _ = _origin(origin)
            rpy = _numbers(origin.get('rpy', '0 0 0'), 3)
            inertia = inertial.find('inertia').attrib
            tensor = np.asarray([
                [inertia['ixx'], inertia['ixy'], inertia['ixz']],
                [inertia['ixy'], inertia['iyy'], inertia['iyz']],
                [inertia['ixz'], inertia['iyz'], inertia['izz']],
            ], dtype=float)
            rotation = Rotation.from_euler('xyz', rpy).as_matrix()
            tensor = rotation @ tensor @ rotation.T
            full_inertia = [
                tensor[0, 0], tensor[1, 1], tensor[2, 2],
                tensor[0, 1], tensor[0, 2], tensor[1, 2],
            ]
            ET.SubElement(body, 'inertial', {
                'pos': _vector(position),
                'mass': inertial.find('mass').get('value'),
                'fullinertia': _vector(full_inertia),
            })
        visual = link.find('visual')
        if visual is not None and link_name in mesh_names:
            position, quaternion = _origin(visual.find('origin'))
            for part, mesh_name in enumerate(mesh_names[link_name], start=1):
                ET.SubElement(body, 'geom', {
                    'name': f'{link_name}_visual_{part}',
                    'type': 'mesh',
                    'mesh': mesh_name,
                    'material': f'{link_name}_material',
                    'pos': _vector(position),
                    'quat': _vector(quaternion),
                })
        ET.SubElement(body, 'site', {
            'name': f'{link_name}_frame', 'type': 'sphere',
            'size': '0.008', 'rgba': '0.95 0.82 0.18 1', 'group': '3',
        })
        for child_joint in children.get(link_name, []):
            add_link(child_joint.find('child').get('link'), body, child_joint, depth + 1)

    add_link(root_link, world)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(mujoco)
    ET.indent(tree, space='  ')
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    return output_path


def preview_model(mjcf_path, joint_positions, preview_path=None, viewer=False):
    """Show one MuJoCo pose without collecting identification data."""
    if preview_path is not None and viewer:
        raise ValueError('choose either --preview or --viewer, not both')
    if preview_path is not None:
        os.environ.setdefault('MUJOCO_GL', 'egl')
    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError(
            'MuJoCo Python is required; install requirements-mujoco.txt') from error

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    positions = np.asarray(joint_positions, dtype=float).reshape(-1)
    joint_ids = []
    for index in range(1, 8):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{index}')
        if joint_id < 0:
            raise ValueError(f'MuJoCo model is missing joint{index}')
        joint_ids.append(joint_id)
    if positions.shape != (len(joint_ids),) or not np.all(np.isfinite(positions)):
        raise ValueError(
            f'joint positions must contain {len(joint_ids)} finite values')
    for joint_id, value in zip(joint_ids, positions):
        data.qpos[model.jnt_qposadr[joint_id]] = value
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    if preview_path is not None:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        body_positions = np.asarray(data.xpos[1:], dtype=float)
        camera.lookat[:] = 0.5 * (
            np.min(body_positions, axis=0) + np.max(body_positions, axis=0))
        camera.distance = max(
            0.75, 2.2 * float(np.max(np.ptp(body_positions, axis=0))))
        camera.azimuth = 135.0
        camera.elevation = -18.0
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera)
            pixels = renderer.render()
        from PIL import Image
        preview_path = Path(preview_path).expanduser().resolve()
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(pixels).save(preview_path)
        print(f'Preview: {preview_path}')

    if viewer:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as active_viewer:
            body_positions = np.asarray(data.xpos[1:], dtype=float)
            active_viewer.cam.lookat[:] = 0.5 * (
                np.min(body_positions, axis=0) + np.max(body_positions, axis=0))
            active_viewer.cam.distance = max(
                0.75, 2.2 * float(np.max(np.ptp(body_positions, axis=0))))
            active_viewer.cam.azimuth = 135.0
            active_viewer.cam.elevation = -18.0
            while active_viewer.is_running():
                step_start = time.perf_counter()
                mujoco.mj_step(model, data)
                active_viewer.sync()
                time.sleep(max(0.0, model.opt.timestep - (
                    time.perf_counter() - step_start)))


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Convert and preview the seven-DOF URDF in MuJoCo')
    parser.add_argument('--urdf', type=Path, default=None)
    parser.add_argument(
        '--config', type=Path, default=default_config_path(),
        help='identification.json; simulation pose defaults are read from it')
    parser.add_argument(
        '--output', type=Path,
        default=RESULTS_ROOT / 'seven_dof/model_visualization/'
        'exo7_preview_sim.xml',
        help='MJCF output path')
    parser.add_argument(
        '--base-pos', type=float, nargs=3, default=None,
        metavar=('X', 'Y', 'Z'), help='world position of the URDF base')
    parser.add_argument(
        '--base-rpy', type=float, nargs=3, default=None,
        metavar=('ROLL', 'PITCH', 'YAW'),
        help='URDF-base orientation in world XYZ RPY radians')
    parser.add_argument(
        '--gravity', type=float, nargs=3, metavar=('GX', 'GY', 'GZ'),
        default=None,
        help='MuJoCo world-frame gravity vector (default: identification.json)')
    parser.add_argument(
        '--joint-positions', type=float, nargs=7, default=(0.0,) * 7,
        metavar=('Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'Q7'),
        help='seven joint angles used only for the preview')
    parser.add_argument(
        '--show-frames', action='store_true',
        help='draw XYZ axes and the gravity direction')
    base_mode = parser.add_mutually_exclusive_group()
    base_mode.add_argument(
        '--fixed-base', action='store_true',
        help='fix the URDF base to the MuJoCo world (default)')
    base_mode.add_argument(
        '--free-base', action='store_true',
        help='make the URDF base a free body for gravity and mouse dragging')
    parser.add_argument(
        '--contacts', action='store_true',
        help='enable mesh-floor contacts')
    parser.add_argument(
        '--drop', action='store_true',
        help='shortcut for --free-base --contacts in the interactive viewer')
    viewer_mode = parser.add_mutually_exclusive_group()
    viewer_mode.add_argument(
        '--viewer', dest='viewer', action='store_true',
        help='open the interactive viewer (default)')
    viewer_mode.add_argument(
        '--no-viewer', dest='viewer', action='store_false',
        help='do not open a viewer; useful for headless conversion')
    parser.set_defaults(viewer=True)
    parser.add_argument(
        '--preview', nargs='?', type=Path, const=RESULTS_ROOT /
        'seven_dof/model_visualization/exo_orientation.png', default=None,
        help='write a PNG preview; without a path use model_visualization/')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    simulation = config.get('simulation', {})
    if args.gravity is None:
        args.gravity = tuple(config.get('gravity', (0.0, 0.0, -9.81)))
    args.timestep = float(simulation.get('timestep', 0.002))
    args.joint_damping = simulation.get('joint_damping')
    args.joint_armature = simulation.get('joint_armature')
    args.joint_friction_loss = simulation.get('joint_friction_loss')
    if args.base_pos is None:
        args.base_pos = tuple(simulation.get('base_position', (0.0, 0.0, 0.0)))
    if args.base_rpy is None:
        args.base_rpy = tuple(
            simulation.get('base_orientation_rpy', (0.0, 0.0, 0.0)))
    if len(args.base_pos) != 3 or len(args.base_rpy) != 3:
        parser.error('--base-pos and --base-rpy must each contain three values')
    if args.fixed_base and args.drop:
        parser.error('--fixed-base and --drop cannot be used together')
    return args


def main(argv=None):
    args = parse_arguments(argv)
    path = convert_urdf_to_mjcf(
        args.urdf or default_urdf_path(),
        args.output,
        args.gravity,
        timestep=args.timestep,
        base_position=args.base_pos,
        base_orientation_rpy=args.base_rpy,
        show_frames=args.show_frames,
        free_base=args.free_base or args.drop,
        enable_contacts=args.contacts or args.drop,
        joint_damping=args.joint_damping,
        joint_armature=args.joint_armature,
        joint_friction_loss=args.joint_friction_loss,
    )
    print(path)
    if args.preview is not None:
        preview_model(
            path, args.joint_positions,
            preview_path=args.preview, viewer=False)
    elif args.viewer:
        preview_model(path, args.joint_positions, viewer=True)


if __name__ == '__main__':
    main()
