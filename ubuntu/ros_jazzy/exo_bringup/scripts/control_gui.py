#!/usr/bin/env python3

import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, UInt8, UInt8MultiArray

from scripts.protocol import (
    JOINT_NAMES,
    MODE_MIT,
    MODE_VELOCITY,
    MOTOR_COUNT,
)


COMMAND_TOPIC = '/dm_motor_usb/command'
ENABLE_TOPIC = '/dm_motor_usb/enable'
MODE_TOPIC = '/dm_motor_usb/control_mode'
FEEDBACK_TOPIC = '/dm_motor_usb/feedback'
STATUS_TOPIC = '/dm_motor_usb/status'
TEMPERATURE_TOPIC = '/dm_motor_usb/temperature'


class MotorControlGuiNode(Node):
    def __init__(self, event_queue):
        super().__init__('dm_motor_control_gui')
        self.event_queue = event_queue

        self.declare_parameter('d4340_velocity_limit', 2.0)
        self.declare_parameter('d4340_torque_limit', 2.0)
        self.declare_parameter('d4310_velocity_limit', 2.0)
        self.declare_parameter('d4310_torque_limit', 2.0)
        self.control_limits = {
            'position': 12.5,
            'd4340': {
                'velocity': self._read_limit(
                    'd4340_velocity_limit', physical_limit=20.0),
                'effort': self._read_limit(
                    'd4340_torque_limit', physical_limit=28.0),
            },
            'd4310': {
                'velocity': self._read_limit(
                    'd4310_velocity_limit', physical_limit=30.0),
                'effort': self._read_limit(
                    'd4310_torque_limit', physical_limit=12.5),
            },
        }

        self.command_pub = self.create_publisher(
            JointState, COMMAND_TOPIC, 10)
        self.enable_pub = self.create_publisher(Bool, ENABLE_TOPIC, 10)
        self.mode_pub = self.create_publisher(UInt8, MODE_TOPIC, 10)

        self.create_subscription(
            JointState, FEEDBACK_TOPIC, self.feedback_callback, 10)
        self.create_subscription(
            UInt8MultiArray, STATUS_TOPIC, self.status_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            TEMPERATURE_TOPIC,
            self.temperature_callback,
            10,
        )

    def _read_limit(self, name, physical_limit):
        value = float(self.get_parameter(name).value)
        if value <= 0.0 or value > physical_limit:
            raise ValueError(
                f'{name} must be greater than 0 and no greater than '
                f'{physical_limit}')
        return value

    def publish_command(self, position, velocity, effort):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(position)
        message.velocity = list(velocity)
        message.effort = list(effort)
        self.command_pub.publish(message)

    def publish_enable(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.enable_pub.publish(message)

    def publish_mode(self, mode):
        message = UInt8()
        message.data = int(mode)
        self.mode_pub.publish(message)

    def feedback_callback(self, message):
        if (len(message.position) < MOTOR_COUNT or
                len(message.velocity) < MOTOR_COUNT or
                len(message.effort) < MOTOR_COUNT):
            return
        self.event_queue.put((
            'feedback',
            list(message.position[:MOTOR_COUNT]),
            list(message.velocity[:MOTOR_COUNT]),
            list(message.effort[:MOTOR_COUNT]),
            time.monotonic(),
        ))

    def status_callback(self, message):
        if len(message.data) >= MOTOR_COUNT:
            self.event_queue.put(
                ('status', list(message.data[:MOTOR_COUNT])))

    def temperature_callback(self, message):
        if len(message.data) >= MOTOR_COUNT * 2:
            self.event_queue.put(
                ('temperature', list(message.data[:MOTOR_COUNT * 2])))


class MotorControlGui:
    COLORS = {
        'background': '#f3f6fa',
        'surface': '#ffffff',
        'primary': '#2563eb',
        'primary_dark': '#1d4ed8',
        'success': '#059669',
        'danger': '#dc2626',
        'warning_bg': '#fff7ed',
        'warning_border': '#f97316',
        'text': '#172033',
        'muted': '#64748b',
        'border': '#dbe3ee',
        'table_alt': '#f8fafc',
    }

    CONTROL_LABELS = {
        'position': '位置 (rad)',
        'velocity': '速度 (rad/s)',
        'effort': '前馈力矩 (N·m)',
    }

    def __init__(self, root, node, event_queue, control_limits):
        self.root = root
        self.node = node
        self.event_queue = event_queue
        self.control_limits = control_limits
        self.targets = {
            name: [0.0] * MOTOR_COUNT for name in self.CONTROL_LABELS
        }
        self.feedback = {
            name: [None] * MOTOR_COUNT for name in self.CONTROL_LABELS
        }
        self.motor_status = [None] * MOTOR_COUNT
        self.temperature = [None] * (MOTOR_COUNT * 2)
        self.last_feedback_time = None
        self.continuous_after_id = None
        self.closing = False
        self.publish_count = 0

        self.root.title('EXO2 七电机调试控制台')
        self.root.geometry('1240x780')
        self.root.minsize(1040, 700)
        self.root.configure(bg=self.COLORS['background'])
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.selected_motor = tk.IntVar(value=0)
        self.mode_var = tk.StringVar(value='MIT')
        self.continuous_var = tk.BooleanVar(value=False)
        self.rate_var = tk.StringVar(value='20.0')
        self.control_vars = {}
        self.entry_vars = {}
        self.control_scales = {}
        self.range_min_vars = {}
        self.range_max_vars = {}

        self._configure_style()
        self._build_layout()
        self._refresh_table()
        self._load_selected_motor()
        self.root.after(50, self._process_events)
        self.root.after(500, self._update_feedback_age)

    def _configure_style(self):
        default_font = tkfont.nametofont('TkDefaultFont')
        default_font.configure(size=10)
        tkfont.nametofont('TkTextFont').configure(size=10)

        style = ttk.Style(self.root)
        if 'clam' in style.theme_names():
            style.theme_use('clam')

        colors = self.COLORS
        style.configure(
            '.',
            background=colors['background'],
            foreground=colors['text'],
            font=default_font,
        )
        style.configure('App.TFrame', background=colors['background'])
        style.configure('Card.TFrame', background=colors['surface'])
        style.configure(
            'Card.TLabel',
            background=colors['surface'],
            foreground=colors['text'],
        )
        style.configure(
            'Card.TCheckbutton',
            background=colors['surface'],
            foreground=colors['text'],
        )
        style.configure(
            'Card.TLabelframe',
            background=colors['surface'],
            bordercolor=colors['border'],
            borderwidth=1,
            relief='solid',
        )
        style.configure(
            'Card.TLabelframe.Label',
            background=colors['surface'],
            foreground=colors['text'],
            font=(default_font.actual('family'), 11, 'bold'),
        )
        style.configure(
            'Muted.TLabel',
            background=colors['surface'],
            foreground=colors['muted'],
        )
        style.configure(
            'Primary.TButton',
            background=colors['primary'],
            foreground='white',
            borderwidth=0,
            padding=(16, 9),
        )
        style.map(
            'Primary.TButton',
            background=[('active', colors['primary_dark'])],
        )
        style.configure(
            'Safe.TButton',
            background=colors['success'],
            foreground='white',
            borderwidth=0,
            padding=(16, 9),
        )
        style.map(
            'Safe.TButton',
            background=[('active', '#047857')],
        )
        style.configure(
            'Danger.TButton',
            background=colors['danger'],
            foreground='white',
            borderwidth=0,
            padding=(16, 9),
        )
        style.map(
            'Danger.TButton',
            background=[('active', '#b91c1c')],
        )
        style.configure('Secondary.TButton', padding=(14, 8))
        style.configure(
            'TEntry',
            fieldbackground=colors['surface'],
            bordercolor=colors['border'],
            padding=6,
        )
        style.configure(
            'TCombobox',
            fieldbackground=colors['surface'],
            bordercolor=colors['border'],
            padding=5,
        )
        style.configure(
            'Horizontal.TScale',
            background=colors['surface'],
            troughcolor='#dce7f7',
        )
        style.configure(
            'Treeview',
            background=colors['surface'],
            fieldbackground=colors['surface'],
            foreground=colors['text'],
            rowheight=31,
            borderwidth=0,
        )
        style.configure(
            'Treeview.Heading',
            background='#e8eef7',
            foreground=colors['text'],
            font=(default_font.actual('family'), 10, 'bold'),
            padding=(5, 8),
            relief='flat',
        )
        style.map(
            'Treeview',
            background=[('selected', '#dbeafe')],
            foreground=[('selected', colors['text'])],
        )
        style.configure(
            'Status.TLabel',
            background='#e8eef7',
            foreground=colors['muted'],
            padding=(12, 7),
        )

    def _build_layout(self):
        colors = self.COLORS
        header = tk.Frame(self.root, bg=colors['primary'], padx=18, pady=14)
        header.pack(fill='x')
        title_group = tk.Frame(header, bg=colors['primary'])
        title_group.pack(side='left')
        tk.Label(
            title_group,
            text='EXO2 七电机调试控制台',
            bg=colors['primary'],
            fg='white',
            font=(tkfont.nametofont('TkDefaultFont').actual('family'),
                  18, 'bold'),
        ).pack(anchor='w')
        tk.Label(
            title_group,
            text='USB Bridge · MIT / 速度模式 · 7轴目标与反馈',
            bg=colors['primary'],
            fg='#dbeafe',
            font=(tkfont.nametofont('TkDefaultFont').actual('family'), 10),
        ).pack(anchor='w', pady=(3, 0))
        self.feedback_badge = tk.Label(
            header,
            text='● 等待反馈',
            bg='#f59e0b',
            fg='white',
            padx=12,
            pady=7,
            font=(tkfont.nametofont('TkDefaultFont').actual('family'),
                  10, 'bold'),
        )
        self.feedback_badge.pack(side='right', padx=(12, 0))

        main = ttk.Frame(
            self.root, style='App.TFrame', padding=(16, 12, 16, 12))
        main.pack(fill='both', expand=True)

        warning_border = tk.Frame(main, bg=colors['warning_border'])
        warning_border.pack(fill='x', pady=(0, 12))
        warning = tk.Label(
            warning_border,
            text=(
                '⚠  安全提示：调试前请脱离人体与外骨骼负载，'
                '准备硬件急停；'
                '连续发送只更新目标，不会自动使能电机。'),
            bg=colors['warning_bg'],
            fg='#9a3412',
            anchor='w',
            padx=12,
            pady=9,
        )
        warning.pack(fill='x', padx=(4, 0))

        top = ttk.LabelFrame(
            main, text='运行设置', style='Card.TLabelframe', padding=12)
        top.pack(fill='x', pady=(0, 12))

        ttk.Label(top, text='电机：', style='Card.TLabel').grid(
            row=0, column=0, sticky='w')
        motor_values = [
            f'{index + 1} - {name}'
            for index, name in enumerate(JOINT_NAMES)
        ]
        self.motor_combo = ttk.Combobox(
            top, values=motor_values, state='readonly', width=20)
        self.motor_combo.current(0)
        self.motor_combo.grid(row=0, column=1, padx=(0, 18), sticky='w')
        self.motor_combo.bind('<<ComboboxSelected>>', self._motor_changed)

        ttk.Label(top, text='控制模式：', style='Card.TLabel').grid(
            row=0, column=2, sticky='w')
        self.mode_combo = ttk.Combobox(
            top,
            textvariable=self.mode_var,
            values=('MIT', '速度'),
            state='readonly',
            width=9,
        )
        self.mode_combo.grid(row=0, column=3, padx=(0, 6), sticky='w')
        ttk.Button(
            top,
            text='应用模式',
            style='Secondary.TButton',
            command=self._apply_mode,
        ).grid(row=0, column=4, padx=(0, 18), sticky='w')

        ttk.Label(
            top, text='持续频率 (Hz)：', style='Card.TLabel'
        ).grid(row=0, column=5, sticky='w')
        ttk.Entry(top, textvariable=self.rate_var, width=8).grid(
            row=0, column=6, padx=(0, 8), sticky='w')
        ttk.Checkbutton(
            top,
            text='持续发送',
            style='Card.TCheckbutton',
            variable=self.continuous_var,
            command=self._toggle_continuous,
        ).grid(row=0, column=7, sticky='w')

        control_frame = ttk.LabelFrame(
            main,
            text='所选电机控制量',
            style='Card.TLabelframe',
            padding=14,
        )
        control_frame.pack(fill='x', pady=(0, 12))
        control_frame.columnconfigure(2, weight=1)

        for row, (name, label) in enumerate(self.CONTROL_LABELS.items()):
            limit = self._limit_for(name, motor_index=0)
            value_var = tk.DoubleVar(value=0.0)
            entry_var = tk.StringVar(value='0.000')
            self.control_vars[name] = value_var
            self.entry_vars[name] = entry_var

            min_var = tk.StringVar(value=f'{-limit:.2f}')
            max_var = tk.StringVar(value=f'{limit:.2f}')
            self.range_min_vars[name] = min_var
            self.range_max_vars[name] = max_var

            ttk.Label(
                control_frame,
                text=label,
                width=18,
                style='Card.TLabel',
            ).grid(row=row, column=0, sticky='w', pady=7)
            ttk.Label(
                control_frame,
                textvariable=min_var,
                width=8,
                anchor='e',
                style='Muted.TLabel',
            ).grid(row=row, column=1, sticky='e', padx=(6, 5))
            scale = ttk.Scale(
                control_frame,
                from_=-limit,
                to=limit,
                variable=value_var,
                command=lambda value, key=name: self._slider_changed(
                    key, value),
            )
            scale.grid(row=row, column=2, sticky='ew', padx=6, pady=7)
            self.control_scales[name] = scale
            ttk.Label(
                control_frame,
                textvariable=max_var,
                width=8,
                anchor='w',
                style='Muted.TLabel',
            ).grid(row=row, column=3, sticky='w', padx=(5, 8))
            entry = ttk.Entry(
                control_frame, textvariable=entry_var, width=12)
            entry.grid(row=row, column=4, sticky='e', pady=7)
            entry.bind(
                '<Return>', lambda _event, key=name: self._entry_changed(key))
            entry.bind(
                '<FocusOut>',
                lambda _event, key=name: self._entry_changed(key),
            )

        button_frame = ttk.Frame(main, style='App.TFrame')
        button_frame.pack(fill='x', pady=(0, 12))
        ttk.Button(
            button_frame,
            text='发送一次',
            style='Primary.TButton',
            command=self._send_once,
        ).pack(side='left', padx=(0, 8))
        ttk.Button(
            button_frame,
            text='全部目标归零',
            style='Secondary.TButton',
            command=self._clear_targets,
        ).pack(side='left', padx=(0, 8))
        ttk.Button(
            button_frame,
            text='失能并清零',
            style='Safe.TButton',
            command=self._disable_and_zero,
        ).pack(side='left', padx=(0, 8))
        ttk.Button(
            button_frame,
            text='使能全部电机',
            style='Danger.TButton',
            command=self._enable,
        ).pack(side='left')

        table_frame = ttk.LabelFrame(
            main,
            text='目标与反馈总览',
            style='Card.TLabelframe',
            padding=8,
        )
        table_frame.pack(fill='both', expand=True)

        columns = (
            'motor', 'target_p', 'target_v', 'target_t',
            'feedback_p', 'feedback_v', 'feedback_t',
            'status', 'mos', 'rotor',
        )
        self.table = ttk.Treeview(
            table_frame, columns=columns, show='headings', height=8)
        scrollbar = ttk.Scrollbar(
            table_frame, orient='vertical', command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.bind('<<TreeviewSelect>>', self._table_motor_selected)
        headings = (
            '电机', '目标位置', '目标速度', '目标力矩',
            '反馈位置', '反馈速度', '反馈力矩',
            '状态', 'MOS℃', '转子℃',
        )
        widths = (130, 90, 90, 90, 90, 90, 90, 70, 70, 70)
        for column, heading, width in zip(columns, headings, widths):
            self.table.heading(column, text=heading)
            self.table.column(column, width=width, anchor='center')
        scrollbar.pack(side='right', fill='y')
        self.table.pack(side='left', fill='both', expand=True)
        self.table.tag_configure('even', background=self.COLORS['surface'])
        self.table.tag_configure('odd', background=self.COLORS['table_alt'])

        self.status_var = tk.StringVar(value='就绪；当前未使能电机')
        ttk.Label(
            self.root,
            textvariable=self.status_var,
            style='Status.TLabel',
            anchor='w',
        ).pack(fill='x', side='bottom')

    def _motor_changed(self, _event=None):
        self.selected_motor.set(self.motor_combo.current())
        self._load_selected_motor()

    def _table_motor_selected(self, _event=None):
        selection = self.table.selection()
        if not selection:
            return
        index = int(selection[0])
        if index == self.selected_motor.get():
            return
        self.motor_combo.current(index)
        self.selected_motor.set(index)
        self._load_selected_motor()

    def _load_selected_motor(self):
        index = self.selected_motor.get()
        for name in self.CONTROL_LABELS:
            lower, upper = self._limits(name)
            self.control_scales[name].configure(from_=lower, to=upper)
            self.range_min_vars[name].set(f'{lower:.2f}')
            self.range_max_vars[name].set(f'{upper:.2f}')
        for name in self.CONTROL_LABELS:
            value = self.targets[name][index]
            self.control_vars[name].set(value)
            self.entry_vars[name].set(f'{value:.3f}')
        if self.table.exists(str(index)):
            self.table.selection_set(str(index))
            self.table.see(str(index))

    def _limits(self, name):
        motor_index = self.selected_motor.get()
        limit = self._limit_for(name, motor_index)
        return -limit, limit

    def _limit_for(self, name, motor_index):
        if name == 'position':
            return self.control_limits['position']
        motor_model = 'd4340' if motor_index < 4 else 'd4310'
        return self.control_limits[motor_model][name]

    def _slider_changed(self, name, raw_value):
        value = float(raw_value)
        self.targets[name][self.selected_motor.get()] = value
        self.entry_vars[name].set(f'{value:.3f}')
        self._refresh_table()

    def _entry_changed(self, name):
        try:
            value = float(self.entry_vars[name].get())
        except ValueError:
            value = self.targets[name][self.selected_motor.get()]
            self.status_var.set(f'{name} 输入无效，已恢复原值')
        lower, upper = self._limits(name)
        value = min(max(value, lower), upper)
        self.targets[name][self.selected_motor.get()] = value
        self.control_vars[name].set(value)
        self.entry_vars[name].set(f'{value:.3f}')
        self._refresh_table()

    def _publish_command(self, update_status=True):
        for name in self.CONTROL_LABELS:
            self._entry_changed(name)
        self.node.publish_command(
            self.targets['position'],
            self.targets['velocity'],
            self.targets['effort'],
        )
        self.publish_count += 1
        if update_status:
            self.status_var.set(
                f'已发送完整7电机目标，第 {self.publish_count} 次')

    def _send_once(self):
        self._publish_command(update_status=True)

    def _continuous_rate(self):
        try:
            rate = float(self.rate_var.get())
        except ValueError:
            return None
        if rate < 0.1 or rate > 100.0:
            return None
        return rate

    def _toggle_continuous(self):
        if self.continuous_var.get():
            if self._continuous_rate() is None:
                self.continuous_var.set(False)
                messagebox.showerror(
                    '频率无效', '持续频率必须在 0.1～100 Hz 之间。')
                return
            self.status_var.set('持续发送已启动（不会自动使能）')
            self._continuous_tick()
        else:
            if self.continuous_after_id is not None:
                self.root.after_cancel(self.continuous_after_id)
                self.continuous_after_id = None
            self.status_var.set('持续发送已停止')

    def _continuous_tick(self):
        if not self.continuous_var.get() or self.closing:
            self.continuous_after_id = None
            return
        rate = self._continuous_rate()
        if rate is None:
            self.continuous_var.set(False)
            self.status_var.set('持续发送已停止：频率无效')
            self.continuous_after_id = None
            return
        self._publish_command(update_status=False)
        interval_ms = max(10, round(1000.0 / rate))
        self.continuous_after_id = self.root.after(
            interval_ms, self._continuous_tick)

    def _clear_targets(self):
        for values in self.targets.values():
            for index in range(MOTOR_COUNT):
                values[index] = 0.0
        self._load_selected_motor()
        self._refresh_table()
        self.status_var.set('全部目标已在界面中归零，尚未发送')

    def _disable_and_zero(self):
        self.continuous_var.set(False)
        self._toggle_continuous()
        self.node.publish_enable(False)
        self._clear_targets()
        self._publish_command(update_status=False)
        self.status_var.set('已发送失能和全零目标')

    def _enable(self):
        confirmed = messagebox.askyesno(
            '确认使能',
            '将发送当前界面中的完整目标，并使能全部7台电机。\n\n'
            '是否已确认机械安全、反馈正常且硬件急停可用？',
            icon='warning',
        )
        if not confirmed:
            return
        self._publish_command(update_status=False)
        self.status_var.set('当前目标已发送；100 ms 后请求使能全部电机')
        self.root.after(100, self._finish_enable)

    def _finish_enable(self):
        if self.closing:
            return
        self.node.publish_enable(True)
        self.status_var.set('已请求使能全部电机')

    def _apply_mode(self):
        self.continuous_var.set(False)
        self._toggle_continuous()
        self.node.publish_enable(False)
        mode = MODE_VELOCITY if self.mode_var.get() == '速度' else MODE_MIT
        self.node.publish_mode(mode)
        self._clear_targets()
        self._publish_command(update_status=False)
        mode_name = '速度' if mode == MODE_VELOCITY else 'MIT'
        self.status_var.set(
            f'已切换到{mode_name}模式；电机保持失能且目标已清零')

    @staticmethod
    def _format_feedback(value):
        return '--' if value is None else f'{value:.3f}'

    def _refresh_table(self):
        selected_items = self.table.get_children()
        if not selected_items:
            for index in range(MOTOR_COUNT):
                row_tag = 'even' if index % 2 == 0 else 'odd'
                self.table.insert(
                    '', 'end', iid=str(index), tags=(row_tag,))
        for index in range(MOTOR_COUNT):
            mos = self.temperature[index * 2]
            rotor = self.temperature[index * 2 + 1]
            status = self.motor_status[index]
            values = (
                f'{index + 1} {JOINT_NAMES[index]}',
                f'{self.targets["position"][index]:.3f}',
                f'{self.targets["velocity"][index]:.3f}',
                f'{self.targets["effort"][index]:.3f}',
                self._format_feedback(self.feedback['position'][index]),
                self._format_feedback(self.feedback['velocity'][index]),
                self._format_feedback(self.feedback['effort'][index]),
                '--' if status is None else str(status),
                '--' if mos is None else f'{mos:.1f}',
                '--' if rotor is None else f'{rotor:.1f}',
            )
            self.table.item(str(index), values=values)

    def _process_events(self):
        changed = False
        while True:
            try:
                event = self.event_queue.get_nowait()
            except queue.Empty:
                break
            if event[0] == 'feedback':
                self.feedback['position'] = event[1]
                self.feedback['velocity'] = event[2]
                self.feedback['effort'] = event[3]
                self.last_feedback_time = event[4]
                changed = True
            elif event[0] == 'status':
                self.motor_status = event[1]
                changed = True
            elif event[0] == 'temperature':
                self.temperature = event[1]
                changed = True
        if changed:
            self._refresh_table()
        if not self.closing:
            self.root.after(50, self._process_events)

    def _update_feedback_age(self):
        if self.last_feedback_time is None:
            self.feedback_badge.configure(
                text='● 等待反馈', bg='#f59e0b')
        else:
            age = time.monotonic() - self.last_feedback_time
            if age < 0.5:
                self.feedback_badge.configure(
                    text=f'● 反馈正常  {age * 1000:.0f} ms',
                    bg=self.COLORS['success'],
                )
            else:
                self.feedback_badge.configure(
                    text=f'● 反馈超时  {age:.1f} s',
                    bg=self.COLORS['danger'],
                )
        if not self.closing:
            self.root.after(500, self._update_feedback_age)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.continuous_var.set(False)
        if self.continuous_after_id is not None:
            self.root.after_cancel(self.continuous_after_id)
            self.continuous_after_id = None
        self.node.publish_enable(False)
        for values in self.targets.values():
            for index in range(MOTOR_COUNT):
                values[index] = 0.0
        self.node.publish_command(
            self.targets['position'],
            self.targets['velocity'],
            self.targets['effort'],
        )
        self.status_var.set('正在发送失能与全零目标并关闭…')
        self.root.after(100, self.root.destroy)


def main(args=None):
    rclpy.init(args=args)
    node = None
    spin_thread = None
    try:
        event_queue = queue.SimpleQueue()
        node = MotorControlGuiNode(event_queue)
        root = tk.Tk()
        app = MotorControlGui(
            root, node, event_queue, node.control_limits)
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        root.mainloop()
        del app
    except tk.TclError as error:
        print(f'dm_motor_control_gui startup failed: {error}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=0.5)
        if node is not None:
            node.destroy_node()


if __name__ == '__main__':
    main()
