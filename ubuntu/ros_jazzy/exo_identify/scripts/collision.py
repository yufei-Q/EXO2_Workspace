from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


COLLISION_GEOM_TOKEN = '_collision'


def collision_model_fingerprint(mjcf_path) -> str:
    """Hash the MJCF and collision mesh files used by its collision geoms."""
    path = Path(mjcf_path).expanduser().resolve()
    root = ET.parse(path).getroot()
    mesh_files = {
        mesh.get('name'): mesh.get('file')
        for mesh in root.findall('./asset/mesh')
    }
    collision_mesh_names = {
        geom.get('mesh')
        for geom in root.iter('geom')
        if COLLISION_GEOM_TOKEN in (geom.get('name') or '')
        and geom.get('mesh')
    }
    dependencies = [path]
    for mesh_name in sorted(collision_mesh_names):
        filename = mesh_files.get(mesh_name)
        if not filename:
            raise ValueError(
                f'collision geom references missing mesh asset {mesh_name!r}')
        mesh_path = Path(filename)
        if not mesh_path.is_absolute():
            mesh_path = path.parent / mesh_path
        dependencies.append(mesh_path.resolve())

    digest = hashlib.sha256()
    for dependency in dependencies:
        if not dependency.is_file():
            raise FileNotFoundError(
                f'collision-model dependency does not exist: {dependency}')
        dependency_name = (
            path.name
            if dependency == path
            else dependency.relative_to(path.parent).as_posix()
        )
        digest.update(dependency_name.encode('utf-8'))
        digest.update(b'\0')
        with dependency.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    return digest.hexdigest()


def collision_settings(config: dict) -> dict:
    values = config.get('collision', {})
    clearance = float(values.get('minimum_clearance_m', 0.005))
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError('collision.minimum_clearance_m must be finite and non-negative')
    return {
        'enabled': bool(values.get('enabled', True)),
        'minimum_clearance_m': clearance,
        'abort_collection_on_violation': bool(
            values.get('abort_collection_on_violation', True)),
    }


@dataclass(frozen=True)
class CollisionResult:
    collision_free: bool
    checked_samples: int
    minimum_distance_m: float
    minimum_distance_is_lower_bound: bool
    violation_sample: int | None = None
    violation_time_s: float | None = None
    geom1: str | None = None
    geom2: str | None = None
    body1: str | None = None
    body2: str | None = None

    def to_dict(self):
        return asdict(self)

    def describe(self) -> str:
        if self.collision_free:
            operator = '>=' if self.minimum_distance_is_lower_bound else '='
            return (
                f'collision-free for {self.checked_samples} samples; '
                f'minimum distance {operator} {self.minimum_distance_m:.6g} m')
        return (
            f'clearance violation at sample {self.violation_sample}'
            f'{f", t={self.violation_time_s:.6g} s" if self.violation_time_s is not None else ""}'
            ': '
            f'{self.body1}/{self.geom1} vs {self.body2}/{self.geom2}, '
            f'distance={self.minimum_distance_m:.6g} m')


def _name(mujoco, model, object_type, object_id, fallback):
    value = mujoco.mj_id2name(model, object_type, int(object_id))
    return value if value is not None else f'{fallback}#{int(object_id)}'


def collision_geom_ids(mujoco, model) -> np.ndarray:
    result = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name and COLLISION_GEOM_TOKEN in name:
            result.append(geom_id)
    return np.asarray(result, dtype=int)


def validate_collision_model(mujoco, model, minimum_clearance_m: float) -> np.ndarray:
    geom_ids = collision_geom_ids(mujoco, model)
    if geom_ids.size == 0:
        raise ValueError(
            'MJCF contains no named collision geoms; regenerate it with '
            'mujoco_model.py')
    if np.any(model.geom_contype[geom_ids] == 0) or np.any(
            model.geom_conaffinity[geom_ids] == 0):
        raise ValueError('MJCF collision geoms are present but contact is disabled')
    margins = np.asarray(model.geom_margin[geom_ids], dtype=float)
    if np.any(margins + 1e-12 < minimum_clearance_m):
        raise ValueError(
            'MJCF collision margins are smaller than the configured minimum '
            'clearance; regenerate the model with the current configuration')
    gaps = np.asarray(model.geom_gap[geom_ids], dtype=float)
    if np.any(gaps + 1e-12 < minimum_clearance_m):
        raise ValueError(
            'MJCF collision gaps are smaller than the configured minimum '
            'clearance; proximity contacts could apply force before actual '
            'penetration')
    visual_ids = []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name and '_visual_' in name:
            visual_ids.append(geom_id)
    if visual_ids:
        visual_ids = np.asarray(visual_ids, dtype=int)
        if np.any(model.geom_contype[visual_ids] != 0) or np.any(
                model.geom_conaffinity[visual_ids] != 0):
            raise ValueError(
                'MJCF visual geoms must have contact disabled; use dedicated '
                'collision geoms for contact')
    return geom_ids


def clearance_contacts(
        mujoco, model, data, minimum_clearance_m: float) -> list[dict]:
    contacts = []
    for index in range(data.ncon):
        contact = data.contact[index]
        if float(contact.dist) + 1e-12 >= minimum_clearance_m:
            continue
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = int(model.geom_bodyid[geom1])
        body2 = int(model.geom_bodyid[geom2])
        contacts.append({
            'distance_m': float(contact.dist),
            'geom1': _name(
                mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, geom1, 'geom'),
            'geom2': _name(
                mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, geom2, 'geom'),
            'body1': _name(
                mujoco, model, mujoco.mjtObj.mjOBJ_BODY, body1, 'body'),
            'body2': _name(
                mujoco, model, mujoco.mjtObj.mjOBJ_BODY, body2, 'body'),
        })
    return contacts


def geom_distance(mujoco, model, data, geom1, geom2, distmax=1.0):
    """Return MuJoCo's signed distance and nearest points for two geoms."""
    ids = []
    for value in (geom1, geom2):
        requested = value
        if isinstance(value, str):
            value = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, value)
            if value < 0:
                raise ValueError(f'MJCF does not contain geom {requested!r}')
        ids.append(int(value))
    nearest_points = np.zeros(6, dtype=float)
    distance = mujoco.mj_geomDistance(
        model, data, ids[0], ids[1], float(distmax), nearest_points)
    return float(distance), nearest_points.reshape(2, 3)


class CollisionChecker:
    """Kinematically check trajectories with the same MJCF used for simulation."""

    def __init__(self, mjcf_path, joint_count, minimum_clearance_m):
        try:
            import mujoco
        except ImportError as error:
            raise RuntimeError(
                'MuJoCo is required for collision-aware trajectory design') from error
        self.mujoco = mujoco
        self.path = Path(mjcf_path).expanduser().resolve()
        if not self.path.is_file():
            raise FileNotFoundError(f'MJCF does not exist: {self.path}')
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)
        self.joint_count = int(joint_count)
        self.minimum_clearance_m = float(minimum_clearance_m)
        self.collision_geom_ids = validate_collision_model(
            mujoco, self.model, self.minimum_clearance_m)
        self.qpos_addresses = []
        for index in range(1, self.joint_count + 1):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{index}')
            if joint_id < 0:
                raise ValueError(f'MJCF is missing joint{index}')
            self.qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
        self.qpos_addresses = np.asarray(self.qpos_addresses, dtype=int)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def check_positions(self, q, time_values=None) -> CollisionResult:
        positions = np.asarray(q, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != self.joint_count:
            raise ValueError(
                f'q must have shape (samples, {self.joint_count})')
        if not np.all(np.isfinite(positions)):
            raise ValueError('collision-check positions contain non-finite values')
        if time_values is not None:
            times = np.asarray(time_values, dtype=float).reshape(-1)
            if times.shape != (positions.shape[0],):
                raise ValueError('collision-check timestamps do not match q')
        else:
            times = None

        minimum_seen = np.inf
        for sample, position in enumerate(positions):
            self.data.qpos[self.qpos_addresses] = position
            self.data.qvel[:] = 0.0
            self.data.qacc[:] = 0.0
            self.data.ctrl[:] = 0.0
            self.data.qfrc_applied[:] = 0.0
            self.mujoco.mj_forward(self.model, self.data)
            violations = clearance_contacts(
                self.mujoco, self.model, self.data,
                self.minimum_clearance_m)
            if not violations:
                continue
            closest = min(violations, key=lambda item: item['distance_m'])
            minimum_seen = min(minimum_seen, closest['distance_m'])
            return CollisionResult(
                collision_free=False,
                checked_samples=sample + 1,
                minimum_distance_m=minimum_seen,
                minimum_distance_is_lower_bound=False,
                violation_sample=sample,
                violation_time_s=(float(times[sample]) if times is not None else None),
                geom1=closest['geom1'],
                geom2=closest['geom2'],
                body1=closest['body1'],
                body2=closest['body2'],
            )
        return CollisionResult(
            collision_free=True,
            checked_samples=positions.shape[0],
            minimum_distance_m=self.minimum_clearance_m,
            minimum_distance_is_lower_bound=True,
        )
