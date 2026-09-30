#!/usr/bin/env python3

"""Build a MuJoCo MJCF model from the SolidWorks URDF export."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import struct
import threading
import time
import xml.etree.ElementTree as ET

from collision import collision_settings
from common import default_config_path, default_urdf_path, load_config, RESULTS_ROOT
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


MATERIAL_COLORS = (
    '0.76 0.78 0.80 1',
    '0.18 0.42 0.62 1',
    '0.82 0.25 0.22 1',
    '0.32 0.56 0.39 1',
)


def _joint_array(value, joint_count, name, minimum=0.0):
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size == 1:
        values = np.full(joint_count, float(values[0]), dtype=float)
    if values.shape != (joint_count,) or not np.all(np.isfinite(values)):
        raise ValueError(f'{name} must contain one finite value per joint')
    if np.any(values < minimum):
        raise ValueError(f'{name} must be >= {minimum:g}')
    return values


class JointControlState:
    """Thread-safe state shared by the Tk panel and simulation loop."""

    def __init__(self, targets, enabled=False):
        self.targets = np.asarray(targets, dtype=float).copy()
        self.enabled = bool(enabled)
        self.reset_requested = False
        self.hold_requested = False
        self.lock = threading.Lock()

    def snapshot(self):
        with self.lock:
            return self.enabled, self.targets.copy()

    def set_enabled(self, value):
        with self.lock:
            self.enabled = bool(value)

    def toggle_enabled(self):
        with self.lock:
            self.enabled = not self.enabled

    def set_target(self, index, value):
        with self.lock:
            self.targets[index] = float(value)

    def request_reset(self):
        with self.lock:
            self.reset_requested = True

    def request_hold(self):
        with self.lock:
            self.hold_requested = True

    def consume_requests(self):
        with self.lock:
            reset = self.reset_requested
            hold = self.hold_requested
            self.reset_requested = False
            self.hold_requested = False
            return reset, hold

    def hold(self, positions):
        with self.lock:
            self.targets[:] = positions
            self.enabled = True

    def reset_targets(self, positions):
        with self.lock:
            self.targets[:] = positions


class JointControlPanel:
    """Tk angle controls for the seven MuJoCo joints."""

    def __init__(self, state, joint_names, angle_range_deg):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.state = state
        self.closed = False
        self.root = tk.Tk()
        self.root.title('MuJoCo joint angle controls')
        self.root.geometry('700x570')
        self.root.minsize(560, 430)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        enabled, targets = state.snapshot()
        self.enabled_var = tk.BooleanVar(value=enabled)
        ttk.Checkbutton(
            self.root, text='Angle control (actuators)',
            variable=self.enabled_var, command=self._set_enabled,
        ).pack(anchor='w', padx=12, pady=(10, 4))

        buttons = ttk.Frame(self.root)
        buttons.pack(fill='x', padx=10, pady=(0, 6))
        ttk.Button(
            buttons, text='Hold current pose', command=state.request_hold,
        ).pack(side='left', padx=2)
        ttk.Button(
            buttons, text='Reset', command=state.request_reset,
        ).pack(side='left', padx=2)

        header = ttk.Frame(self.root)
        header.pack(fill='x', padx=12)
        ttk.Label(header, text='Target angle [deg]').pack(side='left')
        ttk.Label(header, text='Actual / torque', width=23).pack(side='right')

        low, high = angle_range_deg
        self.target_vars = []
        self.actual_vars = []
        for index, (name, target) in enumerate(zip(joint_names, targets)):
            row = ttk.Frame(self.root)
            row.pack(fill='x', padx=10, pady=2)
            ttk.Label(row, text=name, width=8).pack(side='left')
            target_var = tk.DoubleVar(value=np.rad2deg(target))
            scale = tk.Scale(
                row, variable=target_var, from_=low, to=high,
                resolution=0.1, orient=tk.HORIZONTAL, showvalue=True,
                command=lambda value, joint=index: self._set_target(
                    joint, value),
            )
            scale.pack(side='left', fill='x', expand=True)
            actual_var = tk.StringVar(value='+0.0 deg / +0.00 Nm')
            ttk.Label(
                row, textvariable=actual_var, width=23, anchor='e',
            ).pack(side='right')
            self.target_vars.append(target_var)
            self.actual_vars.append(actual_var)

        ttk.Separator(self.root).pack(fill='x', padx=10, pady=6)
        self.status_var = tk.StringVar(value='Starting simulation...')
        ttk.Label(
            self.root, textvariable=self.status_var, justify='left',
        ).pack(anchor='w', padx=12, pady=2)

    def _set_enabled(self):
        self.state.set_enabled(self.enabled_var.get())

    def _set_target(self, index, degrees):
        self.state.set_target(index, np.deg2rad(float(degrees)))

    def update(self, positions, torques, contact_count, minimum_distance):
        if self.closed:
            return False
        try:
            enabled, targets = self.state.snapshot()
            self.enabled_var.set(enabled)
            for index, (position, torque) in enumerate(
                    zip(positions, torques)):
                target_degrees = np.rad2deg(targets[index])
                if abs(self.target_vars[index].get() - target_degrees) > 0.05:
                    self.target_vars[index].set(target_degrees)
                self.actual_vars[index].set(
                    f'{np.rad2deg(position):+7.2f} deg / {torque:+6.2f} Nm')
            distance_text = (
                'n/a' if minimum_distance is None
                else f'{1000.0 * minimum_distance:.2f} mm')
            self.status_var.set(
                f'Control: {"ON" if enabled else "OFF"}    '
                f'contacts: {contact_count}    '
                f'minimum contact distance: {distance_text}\n'
                'Keys in viewer: C toggle, H hold current pose, R reset')
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.closed = True
        return not self.closed

    def close(self):
        self.closed = True
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


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


def _read_binary_stl(path: Path):
    with path.open('rb') as stream:
        header = stream.read(80)
        count_bytes = stream.read(4)
        if len(count_bytes) != 4:
            raise ValueError(f'invalid binary STL: {path}')
        face_count = struct.unpack('<I', count_bytes)[0]
        faces = stream.read()
    if len(faces) != 50 * face_count:
        raise ValueError(f'only binary STL meshes are supported: {path}')
    return header, face_count, faces


def _asset_directory(output_path: Path) -> Path:
    asset_dir = output_path.parent / f'{output_path.stem}_assets'
    asset_dir.mkdir(parents=True, exist_ok=True)
    return asset_dir


def _split_large_binary_stl(path: Path, output_path: Path, max_faces=190000):
    asset_dir = _asset_directory(output_path)
    header, face_count, faces = _read_binary_stl(path)
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


def _write_collision_hull(path: Path, output_path: Path, name: str) -> Path:
    _, face_count, faces = _read_binary_stl(path)
    face_dtype = np.dtype([
        ('normal', '<f4', (3,)),
        ('vertices', '<f4', (3, 3)),
        ('attribute', '<u2'),
    ])
    records = np.frombuffer(faces, dtype=face_dtype, count=face_count)
    vertices = np.unique(records['vertices'].reshape(-1, 3), axis=0)
    if vertices.shape[0] < 4:
        raise ValueError(f'collision mesh has fewer than four vertices: {path}')
    hull = ConvexHull(vertices)
    triangles = np.asarray(vertices[hull.simplices], dtype=np.float32)
    edges1 = triangles[:, 1] - triangles[:, 0]
    edges2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges1, edges2)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals[valid] /= lengths[valid, None]
    normals[~valid] = 0.0

    destination = _asset_directory(output_path) / f'{name}_collision_hull.stl'
    output_records = np.zeros(triangles.shape[0], dtype=face_dtype)
    output_records['normal'] = normals
    output_records['vertices'] = triangles
    header = b'MuJoCo convex collision hull'.ljust(80, b' ')
    with destination.open('wb') as stream:
        stream.write(header)
        stream.write(struct.pack('<I', triangles.shape[0]))
        stream.write(output_records.tobytes())
    return destination


def convert_urdf_to_mjcf(
    urdf_path: Path,
    output_path: Path,
    gravity=(0.0, 0.0, -9.81),
    timestep=0.002,
    base_position=(0.0, 0.0, 0.0),
    base_orientation_rpy=(0.0, 0.0, 0.0),
    show_frames=False,
    free_base=False,
    enable_contacts=True,
    collision_clearance_m=0.005,
    joint_damping=None,
    joint_armature=None,
    joint_friction_loss=None,
):
    urdf_path = Path(urdf_path).resolve()
    output_path = Path(output_path).resolve()
    gravity = np.asarray(gravity, dtype=float)
    base_position = np.asarray(base_position, dtype=float)
    base_orientation_rpy = np.asarray(base_orientation_rpy, dtype=float)
    collision_clearance_m = float(collision_clearance_m)
    if not np.isfinite(collision_clearance_m) or collision_clearance_m < 0.0:
        raise ValueError('collision_clearance_m must be finite and non-negative')
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
    if enable_contacts:
        ET.SubElement(mujoco.find('option'), 'flag', {'multiccd': 'enable'})
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
        'contype': '0',
        'conaffinity': '0',
        'group': '1',
    })

    asset = ET.SubElement(mujoco, 'asset')
    visual_mesh_names = {}
    collision_meshes = {}
    for link_index, (link_name, link) in enumerate(links.items()):
        visual = link.find('visual')
        mesh = visual.find('geometry/mesh') if visual is not None else None
        if mesh is not None:
            path = _mesh_path(mesh.get('filename'), urdf_path)
            if not path.exists():
                raise FileNotFoundError(
                    f'visual mesh for {link_name} does not exist: {path}')
            visual_mesh_names[link_name] = []
            for part, mesh_path in enumerate(
                    _split_large_binary_stl(path, output_path), start=1):
                mesh_name = f'{link_name}_mesh_{part}'
                visual_mesh_names[link_name].append(mesh_name)
                attributes = {
                    'name': mesh_name,
                    'file': os.path.relpath(mesh_path, output_path.parent),
                }
                if mesh.get('scale'):
                    attributes['scale'] = mesh.get('scale')
                ET.SubElement(asset, 'mesh', attributes)

        collision_meshes[link_name] = []
        for collision_index, collision in enumerate(
                link.findall('collision'), start=1):
            collision_mesh = collision.find('geometry/mesh')
            if collision_mesh is None:
                raise ValueError(
                    f'{link_name} collision geometry must be a mesh')
            collision_path = _mesh_path(
                collision_mesh.get('filename'), urdf_path)
            if not collision_path.exists():
                raise FileNotFoundError(
                    f'collision mesh for {link_name} does not exist: '
                    f'{collision_path}')
            hull_path = _write_collision_hull(
                collision_path, output_path,
                f'{link_name}_{collision_index}')
            collision_name = f'{link_name}_collision_mesh_{collision_index}'
            attributes = {
                'name': collision_name,
                'file': os.path.relpath(hull_path, output_path.parent),
            }
            if collision_mesh.get('scale'):
                attributes['scale'] = collision_mesh.get('scale')
            ET.SubElement(asset, 'mesh', attributes)
            collision_meshes[link_name].append((collision_name, collision))
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
        'margin': f'{collision_clearance_m:.12g}',
        'gap': f'{collision_clearance_m:.12g}',
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

    if enable_contacts:
        contact = ET.SubElement(mujoco, 'contact')
        for joint in joints:
            ET.SubElement(contact, 'exclude', {
                'body1': joint.find('parent').get('link'),
                'body2': joint.find('child').get('link'),
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
        if visual is not None and link_name in visual_mesh_names:
            position, quaternion = _origin(visual.find('origin'))
            for part, mesh_name in enumerate(
                    visual_mesh_names[link_name], start=1):
                ET.SubElement(body, 'geom', {
                    'name': f'{link_name}_visual_{part}',
                    'type': 'mesh',
                    'mesh': mesh_name,
                    'material': f'{link_name}_material',
                    'pos': _vector(position),
                    'quat': _vector(quaternion),
                    'contype': '0',
                    'conaffinity': '0',
                    'group': '1',
                })
        for collision_index, (mesh_name, collision) in enumerate(
                collision_meshes.get(link_name, []), start=1):
            position, quaternion = _origin(collision.find('origin'))
            ET.SubElement(body, 'geom', {
                'name': f'{link_name}_collision_{collision_index}',
                'type': 'mesh',
                'mesh': mesh_name,
                'pos': _vector(position),
                'quat': _vector(quaternion),
                'contype': '1' if enable_contacts else '0',
                'conaffinity': '1' if enable_contacts else '0',
                'margin': f'{collision_clearance_m:.12g}',
                'gap': f'{collision_clearance_m:.12g}',
                'group': '4',
                'rgba': '0.95 0.18 0.10 0',
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


def _joint_servo_torque(
        data, qpos_address, dof_address, targets,
        position_kp, velocity_kd, torque_limits):
    positions = np.asarray(data.qpos[qpos_address], dtype=float)
    velocities = np.asarray(data.qvel[dof_address], dtype=float)
    gravity_and_coriolis = np.asarray(
        data.qfrc_bias[dof_address], dtype=float)
    torque = (
        gravity_and_coriolis
        + position_kp * (targets - positions)
        - velocity_kd * velocities
    )
    return np.clip(torque, -torque_limits, torque_limits)


def preview_model(
        mjcf_path, joint_positions, preview_path=None, viewer=False,
        control_gui=True, control_enabled=False, position_kp=20.0,
        velocity_kd=1.0, torque_limits=8.0,
        angle_range_deg=(-180.0, 180.0)):
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
    joint_names = [f'joint{index}' for index in range(1, 8)]
    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f'MuJoCo model is missing {joint_name}')
        joint_ids.append(joint_id)
    if positions.shape != (len(joint_ids),) or not np.all(np.isfinite(positions)):
        raise ValueError(
            f'joint positions must contain {len(joint_ids)} finite values')
    qpos_address = np.asarray(
        [model.jnt_qposadr[joint_id] for joint_id in joint_ids], dtype=int)
    dof_address = np.asarray(
        [model.jnt_dofadr[joint_id] for joint_id in joint_ids], dtype=int)
    actuator_ids = np.asarray([
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{name}_motor')
        for name in joint_names
    ], dtype=int)
    if np.any(actuator_ids < 0):
        missing = [
            f'{name}_motor' for name, actuator_id
            in zip(joint_names, actuator_ids) if actuator_id < 0
        ]
        raise ValueError(f'MuJoCo model is missing actuators: {missing}')
    data.qpos[qpos_address] = positions
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
        joint_count = len(joint_ids)
        position_kp = _joint_array(
            position_kp, joint_count, 'position_kp')
        velocity_kd = _joint_array(
            velocity_kd, joint_count, 'velocity_kd')
        torque_limits = _joint_array(
            torque_limits, joint_count, 'torque_limits', minimum=1e-9)
        angle_range_deg = np.asarray(angle_range_deg, dtype=float)
        if (angle_range_deg.shape != (2,)
                or not np.all(np.isfinite(angle_range_deg))
                or angle_range_deg[0] >= angle_range_deg[1]):
            raise ValueError('angle_range_deg must be finite MIN MAX values')

        state = JointControlState(positions, enabled=control_enabled)

        def key_callback(key):
            if key in (ord('C'), ord('c')):
                state.toggle_enabled()
            elif key in (ord('H'), ord('h')):
                state.request_hold()
            elif key in (ord('R'), ord('r')):
                state.request_reset()

        panel = None
        active_viewer = mujoco.viewer.launch_passive(
            model, data, key_callback=key_callback,
            show_left_ui=True, show_right_ui=True)
        try:
            if control_gui:
                try:
                    panel = JointControlPanel(
                        state, joint_names, angle_range_deg)
                except Exception as error:
                    raise RuntimeError(
                        'Could not create the joint control GUI. Use --no-gui '
                        'on a headless machine.') from error
            body_positions = np.asarray(data.xpos[1:], dtype=float)
            active_viewer.cam.lookat[:] = 0.5 * (
                np.min(body_positions, axis=0) + np.max(body_positions, axis=0))
            active_viewer.cam.distance = max(
                0.75, 2.2 * float(np.max(np.ptp(body_positions, axis=0))))
            active_viewer.cam.azimuth = 135.0
            active_viewer.cam.elevation = -18.0
            next_step = time.perf_counter()
            next_gui_update = 0.0
            while active_viewer.is_running():
                now = time.perf_counter()
                if now < next_step:
                    time.sleep(min(0.001, next_step - now))
                    continue
                next_step += float(model.opt.timestep)

                reset_requested, hold_requested = state.consume_requests()
                with active_viewer.lock():
                    if reset_requested:
                        mujoco.mj_resetData(model, data)
                        data.qpos[qpos_address] = positions
                        data.qvel[:] = 0.0
                        state.reset_targets(positions)
                    if hold_requested:
                        state.hold(
                            np.asarray(data.qpos[qpos_address], dtype=float))

                    mujoco.mj_forward(model, data)
                    enabled, targets = state.snapshot()
                    if enabled:
                        data.ctrl[actuator_ids] = _joint_servo_torque(
                            data, qpos_address, dof_address, targets,
                            position_kp, velocity_kd, torque_limits)
                    else:
                        data.ctrl[actuator_ids] = 0.0
                    mujoco.mj_step(model, data)
                    actual_positions = np.asarray(
                        data.qpos[qpos_address], dtype=float).copy()
                    actual_torques = np.asarray(
                        data.qfrc_actuator[dof_address], dtype=float).copy()
                    contact_distances = [
                        float(data.contact[index].dist)
                        for index in range(data.ncon)
                    ]
                active_viewer.sync()
                if panel is not None and now >= next_gui_update:
                    if not panel.update(
                            actual_positions, actual_torques,
                            len(contact_distances),
                            min(contact_distances) if contact_distances else None):
                        break
                    next_gui_update = now + 1.0 / 30.0
        finally:
            if panel is not None:
                panel.close()
            active_viewer.close()


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
    contact_mode = parser.add_mutually_exclusive_group()
    contact_mode.add_argument(
        '--contacts', dest='contacts', action='store_true',
        help='enable generated collision hulls and floor contact (default)')
    contact_mode.add_argument(
        '--no-contacts', dest='contacts', action='store_false',
        help='disable contact for visualization diagnostics only')
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
    parser.set_defaults(viewer=True, contacts=True)
    parser.add_argument(
        '--no-gui', action='store_true',
        help='disable the joint-angle control panel')
    parser.add_argument(
        '--control-on', action='store_true',
        help='start with actuator angle control enabled (default: off)')
    parser.add_argument(
        '--angle-range-deg', type=float, nargs=2,
        default=(-180.0, 180.0), metavar=('MIN', 'MAX'),
        help='joint slider range in degrees (default: -180 180)')
    parser.add_argument(
        '--preview', nargs='?', type=Path, const=RESULTS_ROOT /
        'seven_dof/model_visualization/exo_orientation.png', default=None,
        help='write a PNG preview; without a path use model_visualization/')
    args = parser.parse_args(argv)
    config = load_config(args.config)
    simulation = config.get('simulation', {})
    collision = collision_settings(config)
    args.collision_clearance = collision['minimum_clearance_m']
    args.collision_enabled = collision['enabled']
    if args.gravity is None:
        args.gravity = tuple(config.get('gravity', (0.0, 0.0, -9.81)))
    args.timestep = float(simulation.get('timestep', 0.002))
    args.joint_damping = simulation.get('joint_damping')
    args.joint_armature = simulation.get('joint_armature')
    args.joint_friction_loss = simulation.get('joint_friction_loss')
    controller = simulation.get('controller', {})
    args.position_kp = controller.get('position_kp', 20.0)
    args.velocity_kd = controller.get('velocity_kd', 1.0)
    args.torque_limits = controller.get('torque_limits', 8.0)
    if args.base_pos is None:
        args.base_pos = tuple(simulation.get('base_position', (0.0, 0.0, 0.0)))
    if args.base_rpy is None:
        args.base_rpy = tuple(
            simulation.get('base_orientation_rpy', (0.0, 0.0, 0.0)))
    if len(args.base_pos) != 3 or len(args.base_rpy) != 3:
        parser.error('--base-pos and --base-rpy must each contain three values')
    if args.fixed_base and args.drop:
        parser.error('--fixed-base and --drop cannot be used together')
    if (not np.all(np.isfinite(args.angle_range_deg))
            or args.angle_range_deg[0] >= args.angle_range_deg[1]):
        parser.error('--angle-range-deg requires finite MIN < MAX')
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
        enable_contacts=(args.contacts and args.collision_enabled) or args.drop,
        collision_clearance_m=args.collision_clearance,
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
        preview_model(
            path, args.joint_positions, viewer=True,
            control_gui=not args.no_gui,
            control_enabled=args.control_on,
            position_kp=args.position_kp,
            velocity_kd=args.velocity_kd,
            torque_limits=args.torque_limits,
            angle_range_deg=args.angle_range_deg)


if __name__ == '__main__':
    main()
