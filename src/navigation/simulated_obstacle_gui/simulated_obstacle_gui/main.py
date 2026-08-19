#!/usr/bin/env python3
"""Qt control panel for a topic-controlled simulated costmap obstacle."""

import signal
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtWidgets import QDoubleSpinBox
from python_qt_binding.QtWidgets import QFormLayout
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool


class SimulatedObstaclePublisher(Node):
    """Publish a map-frame center and enabled flag using latched QoS."""

    def __init__(self) -> None:
        super().__init__('simulated_obstacle_gui')
        self.declare_parameter('center_topic', '/simulated_obstacle/center')
        self.declare_parameter('enabled_topic', '/simulated_obstacle/enabled')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_enabled', False)
        self.declare_parameter('clear_on_exit', True)

        self.center_topic = str(self.get_parameter('center_topic').value)
        self.enabled_topic = str(self.get_parameter('enabled_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_enabled = bool(
            self.get_parameter('initial_enabled').value)
        self.clear_on_exit = bool(self.get_parameter('clear_on_exit').value)

        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.center_publisher = self.create_publisher(
            PointStamped, self.center_topic, qos)
        self.enabled_publisher = self.create_publisher(
            Bool, self.enabled_topic, qos)

    def publish_obstacle(self, x: float, y: float, enabled: bool) -> None:
        """Publish the center first, then atomically meaningful enabled state."""
        center = PointStamped()
        center.header.stamp = self.get_clock().now().to_msg()
        center.header.frame_id = self.frame_id
        center.point.x = x
        center.point.y = y
        center.point.z = 0.0
        self.center_publisher.publish(center)

        enabled_message = Bool()
        enabled_message.data = enabled
        self.enabled_publisher.publish(enabled_message)


class SimulatedObstacleWindow(QMainWindow):
    """Small control panel for repeatedly publishing obstacle coordinates."""

    def __init__(self, node: SimulatedObstaclePublisher) -> None:
        super().__init__()
        self.node = node
        self._closing = False
        self.setWindowTitle('Nav2 圆形模拟障碍物')
        self.setMinimumWidth(440)
        self._build_ui()
        self._start_timers()
        QTimer.singleShot(0, self.publish_current)

    def _build_ui(self) -> None:
        central = QWidget(self)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(16, 16, 16, 16)
        root_layout.setSpacing(12)

        title = QLabel('二维代价地图模拟障碍物')
        title.setStyleSheet('font-size: 20px; font-weight: bold;')
        root_layout.addWidget(title)

        frame_label = QLabel(
            f'输入坐标系：{self.node.frame_id}  '
            f'（圆半径由 nav2_params.yaml 决定）')
        frame_label.setWordWrap(True)
        root_layout.addWidget(frame_label)

        coordinate_group = QGroupBox('圆心坐标')
        coordinate_form = QFormLayout(coordinate_group)
        self.x_spin = self._coordinate_spin_box(self.node.initial_x)
        self.y_spin = self._coordinate_spin_box(self.node.initial_y)
        coordinate_form.addRow('X / m', self.x_spin)
        coordinate_form.addRow('Y / m', self.y_spin)
        root_layout.addWidget(coordinate_group)

        self.enabled_checkbox = QCheckBox('在代价地图中启用障碍物')
        self.enabled_checkbox.setChecked(self.node.initial_enabled)
        root_layout.addWidget(self.enabled_checkbox)

        auto_row = QHBoxLayout()
        self.auto_publish_checkbox = QCheckBox('持续发布')
        self.auto_publish_checkbox.setChecked(True)
        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(0.2, 50.0)
        self.rate_spin.setDecimals(1)
        self.rate_spin.setSingleStep(1.0)
        self.rate_spin.setSuffix(' Hz')
        self.rate_spin.setValue(self.node.publish_rate)
        auto_row.addWidget(self.auto_publish_checkbox)
        auto_row.addStretch(1)
        auto_row.addWidget(QLabel('发布频率'))
        auto_row.addWidget(self.rate_spin)
        root_layout.addLayout(auto_row)

        button_row = QHBoxLayout()
        publish_button = QPushButton('应用并发布')
        publish_button.clicked.connect(self.publish_current)
        clear_button = QPushButton('移除障碍物')
        clear_button.clicked.connect(self.clear_obstacle)
        button_row.addWidget(publish_button)
        button_row.addWidget(clear_button)
        root_layout.addLayout(button_row)

        self.status_label = QLabel('尚未发布')
        self.status_label.setStyleSheet('color: #1565C0; font-weight: bold;')
        self.status_label.setWordWrap(True)
        root_layout.addWidget(self.status_label)

        topics_label = QLabel(
            f'圆心：{self.node.center_topic}\n'
            f'启停：{self.node.enabled_topic}')
        topics_label.setStyleSheet('color: #666666;')
        topics_label.setWordWrap(True)
        root_layout.addWidget(topics_label)

        self.setCentralWidget(central)

    @staticmethod
    def _coordinate_spin_box(initial_value: float) -> QDoubleSpinBox:
        spin_box = QDoubleSpinBox()
        spin_box.setRange(-1000.0, 1000.0)
        spin_box.setDecimals(3)
        spin_box.setSingleStep(0.05)
        spin_box.setValue(initial_value)
        return spin_box

    def _start_timers(self) -> None:
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_once)
        self.ros_timer.start(20)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._auto_publish)
        self.rate_spin.valueChanged.connect(self._update_publish_interval)
        self._update_publish_interval()

    def _update_publish_interval(self, _unused: float = 0.0) -> None:
        interval_ms = max(20, round(1000.0 / self.rate_spin.value()))
        self.publish_timer.start(interval_ms)

    def _spin_once(self) -> None:
        if self._closing:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exception:  # noqa: BLE001
            self.status_label.setText(f'ROS 处理异常：{exception}')

    def _auto_publish(self) -> None:
        if self.auto_publish_checkbox.isChecked():
            self.publish_current(show_status=False)

    def publish_current(
        self, _unused: bool = False, show_status: bool = True
    ) -> None:
        x = self.x_spin.value()
        y = self.y_spin.value()
        enabled = self.enabled_checkbox.isChecked()
        self.node.publish_obstacle(x, y, enabled)
        if show_status:
            state = '启用' if enabled else '停用'
            self.status_label.setText(
                f'已发布：{state}，{self.node.frame_id} '
                f'({x:.3f}, {y:.3f})')

    def clear_obstacle(self, _unused: bool = False) -> None:
        self.enabled_checkbox.setChecked(False)
        self.publish_current()

    def closeEvent(self, event) -> None:  # noqa: N802
        self._closing = True
        self.ros_timer.stop()
        self.publish_timer.stop()
        if self.node.clear_on_exit:
            self.enabled_checkbox.setChecked(False)
            self.node.publish_obstacle(
                self.x_spin.value(), self.y_spin.value(), False)
        event.accept()


def main(args=None) -> None:
    """Run the ROS publisher and Qt event loop in one process."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = None
    try:
        node = SimulatedObstaclePublisher()
        window = SimulatedObstacleWindow(node)
        signal.signal(signal.SIGINT, lambda *_args: window.close())
        window.show()
        app.exec_()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
