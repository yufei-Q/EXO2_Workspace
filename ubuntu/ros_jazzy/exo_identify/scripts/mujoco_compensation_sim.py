#!/usr/bin/env python3

"""Interactive MuJoCo gravity/friction compensation demonstration.

The MuJoCo model is the source of the true physical state.  The controller
receives a separately noise-corrupted copy of q and dq, evaluates the
identified gravity/friction model, and sends the resulting torque to the
MuJoCo motor actuators.  The viewer's normal mouse perturbation is applied to
the same MjData, so a selected link can be pushed while compensation is on.

The optional Tk control panel is deliberately separate from the MuJoCo
viewer.  MuJoCo's viewer has no public API for adding application-specific
sliders, while the passive viewer still provides its standard mouse tools.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time

from collect_mujoco_dynamics import joint_addresses
from collision import (
    clearance_contacts,
    collision_settings,
    validate_collision_model,
)
from common import default_urdf_path, load_config, RESULTS_ROOT
import numpy as np


DEFAULT_MJCF = RESULTS_ROOT / 'seven_dof/simulation_model/exo7_sim.xml'
DEFAULT_PARAMETERS = (
    RESULTS_ROOT / 'seven_dof/dynamics_identification/'
    'identified_parameters_sim.npz'
)
DEFAULT_OUTPUT = RESULTS_ROOT / 'seven_dof/compensation_simulation'


def _joint_array(value, count, name, minimum=0.0):
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size == 1:
        values = np.full(count, float(values[0]))
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise ValueError(f'{name} must contain one finite value per joint')
    if np.any(values < minimum):
        raise ValueError(f'{name} must be >= {minimum:g}')
    return values


def simulation_noise(config, joint_count):
    noise = config.get('simulation', {}).get('noise', {})
    return {
        'position': _joint_array(
            noise.get('position_std', 0.0), joint_count,
            'position noise standard deviation'),
        'velocity': _joint_array(
            noise.get('velocity_std', 0.0), joint_count,
            'velocity noise standard deviation'),
        'acceleration': _joint_array(
            noise.get('acceleration_std', 0.0), joint_count,
            'acceleration noise standard deviation'),
        'command_torque': _joint_array(
            noise.get('command_torque_std', 0.0), joint_count,
            'command torque noise standard deviation'),
        'external_torque': _joint_array(
            noise.get('external_torque_std', 0.0), joint_count,
            'external torque noise standard deviation'),
    }


class CompensationModel:
    """Load either the identified NPZ model or exported JSON formula."""

    def __init__(self, path, config_path=None, urdf_path=None):
        try:
            from gravity_compensation import GravityFormula, GravityRegressorModel
        except ImportError as error:
            raise RuntimeError(
                'gravity_compensation.py and its Pinocchio dependencies are '
                'required for the interactive compensation simulator') from error
        model_path = Path(path).expanduser().resolve()
        if model_path.suffix.lower() == '.json':
            self._model = GravityFormula(model_path)
        elif model_path.suffix.lower() == '.npz':
            self._model = GravityRegressorModel(
                model_path, config_path=config_path, urdf_path=urdf_path)
        else:
            raise ValueError('parameters must be an identified .npz or formula .json')
        self.joint_count = self._model.joint_count

    def gravity(self, q):
        return np.asarray(self._model.evaluate(q), dtype=float).reshape(-1)

    def friction(self, dq):
        return np.asarray(self._model.evaluate_friction(dq), dtype=float).reshape(-1)


@dataclass
class RuntimeSettings:
    gravity_enabled: bool = True
    friction_enabled: bool = False
    braking_enabled: bool = False
    noise_enabled: bool = True
    gravity_scale: float = 1.0
    friction_scale: float = 1.0
    braking_damping: float = 0.3
    position_noise: float = 0.0005
    velocity_noise: float = 0.002
    acceleration_noise: float = 0.02
    command_torque_noise: float = 0.01
    external_torque_noise: float = 0.0
    paused: bool = False
    reset_requested: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self):
        with self.lock:
            return {
                'gravity_enabled': self.gravity_enabled,
                'friction_enabled': self.friction_enabled,
                'braking_enabled': self.braking_enabled,
                'noise_enabled': self.noise_enabled,
                'gravity_scale': float(self.gravity_scale),
                'friction_scale': float(self.friction_scale),
                'braking_damping': max(0.0, float(self.braking_damping)),
                'position_noise': max(0.0, float(self.position_noise)),
                'velocity_noise': max(0.0, float(self.velocity_noise)),
                'acceleration_noise': max(0.0, float(self.acceleration_noise)),
                'command_torque_noise': max(0.0, float(self.command_torque_noise)),
                'external_torque_noise': max(0.0, float(self.external_torque_noise)),
                'paused': self.paused,
            }

    def request_reset(self):
        with self.lock:
            self.reset_requested = True

    def consume_reset(self):
        with self.lock:
            value = self.reset_requested
            self.reset_requested = False
            return value


class ControlPanel:
    """Small Tk panel for runtime switches and global noise controls."""

    NOISE_SLIDER_LIMITS = {
        'position_noise': (0.0, 0.5, 0.001),
        'velocity_noise': (0.0, 5.0, 0.01),
        'acceleration_noise': (0.0, 20.0, 0.1),
        'command_torque_noise': (0.0, 5.0, 0.01),
        'external_torque_noise': (0.0, 5.0, 0.01),
    }

    def __init__(self, settings: RuntimeSettings, joint_count: int):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.settings = settings
        self.root = tk.Tk()
        self.root.title('MuJoCo compensation controls')
        self.root.geometry('380x600')
        self.root.minsize(300, 260)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.closed = False

        container = ttk.Frame(self.root)
        container.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(
            container, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(
            container, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar.grid(row=0, column=1, sticky='ns')
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        self.content = ttk.Frame(self.canvas)
        self.content_window = self.canvas.create_window(
            (0, 0), window=self.content, anchor='nw')
        self.content.bind('<Configure>', self._update_scroll_region)
        self.canvas.bind('<Configure>', self._resize_scroll_content)
        self.root.bind('<MouseWheel>', self._scroll_with_wheel)
        self.root.bind('<Button-4>', self._scroll_with_wheel)
        self.root.bind('<Button-5>', self._scroll_with_wheel)

        self.status_var = tk.StringVar(value='Starting simulation...')
        self.vars = {
            'gravity_enabled': tk.BooleanVar(value=settings.gravity_enabled),
            'friction_enabled': tk.BooleanVar(value=settings.friction_enabled),
            'braking_enabled': tk.BooleanVar(value=settings.braking_enabled),
            'noise_enabled': tk.BooleanVar(value=settings.noise_enabled),
            'paused': tk.BooleanVar(value=settings.paused),
        }
        for name, text in (
                ('gravity_enabled', 'Gravity compensation'),
                ('friction_enabled', 'Friction compensation'),
                ('braking_enabled', 'Velocity braking when gravity is ON'),
                ('noise_enabled', 'Enable sensor/command noise'),
                ('paused', 'Pause physics')):
            ttk.Checkbutton(
                self.content, text=text, variable=self.vars[name],
                command=lambda key=name: self._set_bool(key),
            ).pack(anchor='w', padx=12, pady=4)

        ttk.Separator(self.content).pack(fill='x', padx=8, pady=6)
        self._add_scale('gravity_scale', 'Gravity scale', 0.0, 1.5,
                        settings.gravity_scale, 0.01)
        self._add_scale('friction_scale', 'Friction scale', 0.0, 1.5,
                        settings.friction_scale, 0.01)
        self._add_scale('braking_damping', 'Braking damping [N m/(rad/s)]',
                        0.0, 2.0, settings.braking_damping, 0.01)
        self._add_noise_scale(
            'position_noise', 'q noise [rad]', settings.position_noise)
        self._add_noise_scale(
            'velocity_noise', 'dq noise [rad/s]', settings.velocity_noise)
        self._add_noise_scale(
            'acceleration_noise', 'ddq noise [rad/s^2]',
            settings.acceleration_noise)
        self._add_noise_scale(
            'command_torque_noise', 'command torque noise [N m]',
            settings.command_torque_noise)
        self._add_noise_scale(
            'external_torque_noise', 'random external torque [N m]',
            settings.external_torque_noise)

        ttk.Button(self.content, text='Reset to initial pose',
                   command=settings.request_reset).pack(fill='x', padx=12, pady=8)
        status_label = ttk.Label(
            self.content, textvariable=self.status_var,
            justify='left', wraplength=335)
        status_label.pack(anchor='w', padx=12, pady=4)
        help_label = ttk.Label(
            self.content,
            text='Use the MuJoCo viewer mouse: click-drag a link to apply a '
            'perturbation force. Keys: G/F toggle compensation, N noise, '
            'Space pause, R reset.',
            justify='left', wraplength=335)
        help_label.pack(anchor='w', padx=12, pady=6)
        self.wrapped_labels = (status_label, help_label)
        self._joint_count = joint_count

    def _update_scroll_region(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))

    def _resize_scroll_content(self, event):
        self.canvas.itemconfigure(self.content_window, width=event.width)
        wraplength = max(1, event.width - 24)
        for label in self.wrapped_labels:
            label.configure(wraplength=wraplength)

    def _scroll_with_wheel(self, event):
        if getattr(event, 'num', None) == 4:
            units = -1
        elif getattr(event, 'num', None) == 5:
            units = 1
        else:
            delta = getattr(event, 'delta', 0)
            if delta == 0:
                return
            units = (
                -int(delta / 120)
                if abs(delta) >= 120 else (-1 if delta > 0 else 1)
            )
        self.canvas.yview_scroll(units, 'units')
        return 'break'

    def _add_noise_scale(self, name, label, initial):
        low, high, resolution = self.NOISE_SLIDER_LIMITS[name]
        self._add_scale(name, label, low, high, initial, resolution)

    def _add_scale(self, name, label, low, high, initial, resolution):
        import tkinter as tk
        from tkinter import ttk
        frame = ttk.Frame(self.content)
        frame.pack(fill='x', padx=8, pady=1)
        ttk.Label(frame, text=label).pack(anchor='w')
        variable = tk.DoubleVar(value=initial)
        scale = tk.Scale(
            frame, variable=variable, from_=low, to=high,
            resolution=resolution, orient=tk.HORIZONTAL, showvalue=True,
            command=lambda value, key=name: self._set_float(key, value),
            length=320,
        )
        scale.pack(fill='x')
        self.vars[name] = variable

    def _set_bool(self, name):
        with self.settings.lock:
            setattr(self.settings, name, bool(self.vars[name].get()))

    def _set_float(self, name, value):
        with self.settings.lock:
            setattr(self.settings, name, float(value))

    def update_status(self, text):
        if not self.closed:
            self.status_var.set(text)

    def update(self):
        if self.closed:
            return False
        try:
            current = self.settings.snapshot()
            for name in ('gravity_enabled', 'friction_enabled',
                         'braking_enabled',
                         'noise_enabled', 'paused'):
                self.vars[name].set(current[name])
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


def _make_initial_state(mujoco, model, data, qpos_address, dof_address, q):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0
    data.qpos[qpos_address] = q
    data.qvel[dof_address] = 0.0
    mujoco.mj_forward(model, data)


def _build_logger(output_path, joint_count):
    columns = ['t']
    for prefix in ('q_true', 'dq_true', 'ddq_true', 'q_measured',
                   'dq_measured', 'ddq_measured', 'gravity', 'friction',
                   'braking', 'command_torque', 'actuator_torque'):
        columns.extend(f'{prefix}{index + 1}' for index in range(joint_count))
    columns.extend((
        'mouse_perturbation_active',
        'clearance_contact_count',
        'penetrating_contact_count',
    ))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stream = output_path.open('w', newline='', encoding='utf-8')
    writer = csv.writer(stream)
    writer.writerow(columns)
    return stream, writer


def run_simulation(args):
    try:
        import mujoco
        import mujoco.viewer
    except ImportError as error:
        raise RuntimeError(
            'MuJoCo Python is required; install requirements-mujoco.txt') from error

    config = load_config(args.config)
    mjcf_path = args.mjcf.expanduser().resolve()
    if not mjcf_path.is_file():
        raise FileNotFoundError(
            f'MJCF does not exist: {mjcf_path}. Generate it with mujoco_model.py first.')
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    collision = collision_settings(config)
    if not collision['enabled']:
        raise ValueError(
            'Collision checking must be enabled for the interactive MuJoCo '
            'compensation simulation')
    validate_collision_model(
        mujoco, model, collision['minimum_clearance_m'])
    joint_count = model.nv
    if model.nq != joint_count or model.nu != joint_count:
        raise ValueError(
            f'expected a one-DOF-per-actuator model, got nq={model.nq}, '
            f'nv={model.nv}, nu={model.nu}')
    qpos_address, dof_address = joint_addresses(mujoco, model, joint_count)

    q_initial = np.zeros(joint_count)
    if args.initial_q is not None:
        q_initial = np.asarray(args.initial_q, dtype=float)
        if q_initial.shape != (joint_count,) or not np.all(np.isfinite(q_initial)):
            raise ValueError(f'--initial-q must contain {joint_count} finite values')
    noise = simulation_noise(config, joint_count)
    settings = RuntimeSettings(
        position_noise=float(noise['position'][0]),
        velocity_noise=float(noise['velocity'][0]),
        acceleration_noise=float(noise['acceleration'][0]),
        command_torque_noise=float(noise['command_torque'][0]),
        external_torque_noise=0.0,
        gravity_enabled=not args.no_gravity,
        friction_enabled=args.friction,
    )
    rng = np.random.default_rng(args.seed)
    data = mujoco.MjData(model)
    _make_initial_state(mujoco, model, data, qpos_address, dof_address, q_initial)
    initial_violations = clearance_contacts(
        mujoco, model, data, collision['minimum_clearance_m'])
    if initial_violations:
        closest = min(
            initial_violations, key=lambda item: item['distance_m'])
        raise ValueError(
            'Initial compensation-simulation pose violates collision '
            f'clearance: {closest["body1"]}/{closest["geom1"]} vs '
            f'{closest["body2"]}/{closest["geom2"]}, '
            f'distance={closest["distance_m"]:.6g} m')
    parameters = CompensationModel(
        args.parameters, config_path=args.config, urdf_path=args.urdf)
    if parameters.joint_count != joint_count:
        raise ValueError(
            f'identified model has {parameters.joint_count} joints but MJCF has '
            f'{joint_count}')

    viewer = None
    if not args.no_viewer:
        def key_callback(key):
            # GLFW key codes are ASCII-compatible for these letters.
            if key in (ord('G'), ord('g')):
                with settings.lock:
                    settings.gravity_enabled = not settings.gravity_enabled
            elif key in (ord('F'), ord('f')):
                with settings.lock:
                    settings.friction_enabled = not settings.friction_enabled
            elif key in (ord('N'), ord('n')):
                with settings.lock:
                    settings.noise_enabled = not settings.noise_enabled
            elif key == 32:
                with settings.lock:
                    settings.paused = not settings.paused
            elif key in (ord('R'), ord('r')):
                settings.request_reset()

        viewer = mujoco.viewer.launch_passive(
            model, data, key_callback=key_callback,
            show_left_ui=True, show_right_ui=True)

    panel = None
    if not args.no_gui:
        try:
            panel = ControlPanel(settings, joint_count)
        except Exception as error:
            if viewer is not None:
                viewer.close()
            raise RuntimeError(
                'Could not create the Tk control panel. Use --no-gui on a '
                'headless machine.') from error

    output_path = args.output.expanduser().resolve()
    stream, writer = _build_logger(output_path, joint_count)
    next_tick = time.perf_counter()
    simulation_elapsed = 0.0
    next_log = 0.0
    next_status = 0.0
    log_period = 1.0 / args.log_rate
    simulation_duration = args.duration if args.duration > 0.0 else None
    step_period = float(model.opt.timestep) / args.speed
    try:
        while True:
            if viewer is not None and not viewer.is_running():
                break
            if panel is not None and not panel.update():
                break
            if (simulation_duration is not None
                    and simulation_elapsed >= simulation_duration):
                break
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            next_tick += step_period

            runtime = settings.snapshot()
            if settings.consume_reset():
                with (viewer.lock() if viewer is not None else _null_context()):
                    _make_initial_state(
                        mujoco, model, data, qpos_address, dof_address, q_initial)
                next_tick = time.perf_counter() + step_period
                continue
            if runtime['paused']:
                next_tick = time.perf_counter() + step_period
                if viewer is not None:
                    viewer.sync()
                continue

            mouse_active = False
            with (viewer.lock() if viewer is not None else _null_context()):
                # MuJoCo's viewer updates this perturbation while the user
                # click-drags a selected body.  Convert it to xfrc_applied
                # before mj_step so it affects this physics step.
                data.qfrc_applied[:] = 0.0
                data.xfrc_applied[:] = 0.0
                if viewer is not None and viewer.perturb.active:
                    mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)
                external_std = runtime['external_torque_noise']
                if runtime['noise_enabled'] and external_std > 0.0:
                    data.qfrc_applied[:] = rng.normal(0.0, external_std, joint_count)

                mujoco.mj_forward(model, data)
                q_true = np.asarray(data.qpos[qpos_address], dtype=float).copy()
                dq_true = np.asarray(data.qvel[dof_address], dtype=float).copy()
                ddq_true = np.asarray(data.qacc[dof_address], dtype=float).copy()
                q_measured = q_true.copy()
                dq_measured = dq_true.copy()
                ddq_measured = ddq_true.copy()
                if runtime['noise_enabled']:
                    q_measured += rng.normal(0.0, runtime['position_noise'], joint_count)
                    dq_measured += rng.normal(0.0, runtime['velocity_noise'], joint_count)
                    ddq_measured += rng.normal(
                        0.0, runtime['acceleration_noise'], joint_count)

                gravity = parameters.gravity(q_measured)
                friction = parameters.friction(dq_measured)
                gravity_command = (
                    runtime['gravity_scale'] * gravity
                    if runtime['gravity_enabled'] else np.zeros(joint_count))
                friction_command = (
                    runtime['friction_scale'] * friction
                    if runtime['friction_enabled'] else np.zeros(joint_count))
                braking_command = (
                    -runtime['braking_damping'] * dq_measured
                    if runtime['braking_enabled'] and runtime['gravity_enabled']
                    else np.zeros(joint_count))
                command_torque = (
                    gravity_command + friction_command + braking_command)
                if runtime['noise_enabled']:
                    command_torque += rng.normal(
                        0.0, runtime['command_torque_noise'], joint_count)
                data.ctrl[:] = command_torque
                mujoco.mj_step(model, data)
                simulation_elapsed += float(model.opt.timestep)
                clearance_contact_count = sum(
                    float(data.contact[index].dist) <
                    collision['minimum_clearance_m']
                    for index in range(data.ncon)
                )
                penetrating_contact_count = sum(
                    float(data.contact[index].dist) < 0.0
                    for index in range(data.ncon)
                )
                actuator_torque = np.asarray(
                    data.qfrc_actuator[dof_address], dtype=float).copy()
                mouse_active = bool(
                    viewer is not None and viewer.perturb.active)
            if viewer is not None:
                viewer.sync()

            if simulation_elapsed >= next_log:
                writer.writerow([
                    f'{simulation_elapsed:.9f}',
                    *q_true, *dq_true, *ddq_true,
                    *q_measured, *dq_measured, *ddq_measured,
                    *gravity_command, *friction_command, *braking_command,
                    *command_torque, *actuator_torque,
                    int(mouse_active),
                    clearance_contact_count,
                    penetrating_contact_count,
                ])
                next_log += log_period
            if panel is not None and simulation_elapsed >= next_status:
                status = (
                    f't={simulation_elapsed:7.2f} s\n'
                    f'gravity={"ON" if runtime["gravity_enabled"] else "OFF"} '
                    f'x{runtime["gravity_scale"]:.2f}; '
                    f'friction={"ON" if runtime["friction_enabled"] else "OFF"} '
                    f'x{runtime["friction_scale"]:.2f}; '
                    f'brake={"ON" if runtime["braking_enabled"] else "OFF"} '
                    f'k={runtime["braking_damping"]:.2f}\n'
                    f'noise={"ON" if runtime["noise_enabled"] else "OFF"}; '
                    f'q1={q_true[0]:+.3f} rad, dq1={dq_true[0]:+.3f} rad/s\n'
                    f'contacts: clearance={clearance_contact_count}, '
                    f'penetrating={penetrating_contact_count}; '
                    f'mouse perturbation={"ACTIVE" if mouse_active else "none"}')
                panel.update_status(status)
                next_status += 0.1
    finally:
        stream.close()
        if panel is not None:
            panel.close()
        if viewer is not None:
            viewer.close()
    print(f'Interactive compensation log: {output_path}')


class _null_context:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Run interactive MuJoCo gravity/friction compensation')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--urdf', type=Path, default=default_urdf_path())
    parser.add_argument('--mjcf', type=Path, default=DEFAULT_MJCF)
    parser.add_argument('--parameters', type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument('--output', type=Path,
                        default=DEFAULT_OUTPUT / 'live_compensation.csv')
    parser.add_argument('--initial-q', type=float, nargs='+')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='simulation seconds; 0 keeps running until the viewer closes')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='simulation speed relative to wall time')
    parser.add_argument('--log-rate', type=float, default=50.0)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--no-gravity', action='store_true')
    friction = parser.add_mutually_exclusive_group()
    friction.add_argument(
        '--friction', dest='friction', action='store_true',
        help='start with active friction compensation enabled')
    friction.add_argument(
        '--no-friction', dest='friction', action='store_false',
        help='start with active friction compensation disabled (default)')
    parser.set_defaults(friction=False)
    parser.add_argument('--no-viewer', action='store_true',
                        help='run headless; useful for a short smoke test')
    parser.add_argument('--no-gui', action='store_true',
                        help='do not create the Tk runtime control panel')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    if args.speed <= 0.0 or not np.isfinite(args.speed):
        raise ValueError('--speed must be finite and positive')
    if args.duration < 0.0 or not np.isfinite(args.duration):
        raise ValueError('--duration must be finite and non-negative')
    if args.log_rate <= 0.0 or not np.isfinite(args.log_rate):
        raise ValueError('--log-rate must be finite and positive')
    if args.no_viewer and args.duration == 0.0:
        raise ValueError(
            '--duration must be positive when --no-viewer is used; '
            'otherwise the headless loop has no close event')
    run_simulation(args)


if __name__ == '__main__':
    main()
