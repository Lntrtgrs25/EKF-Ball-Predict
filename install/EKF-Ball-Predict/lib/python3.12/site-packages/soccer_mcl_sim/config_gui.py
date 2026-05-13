# config_gui.py
import rclpy
from rclpy.node import Node
from basho_interfaces.srv import GetConfig, SetConfig, SaveConfig
import json
import sys

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QSpinBox,
    QCheckBox, QScrollArea
)


# ROS2 Client Node
class ConfigClient(Node):
    def __init__(self):
        super().__init__('config_client')

        self.get_cli = self.create_client(GetConfig, '/basho/config/get_config')
        self.set_cli = self.create_client(SetConfig, '/basho/config/set_config')
        self.save_cli = self.create_client(SaveConfig, '/basho/config/save_config')

        for cli in [self.get_cli, self.set_cli, self.save_cli]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {cli.srv_name} not available, waiting...')

    def fetch_config(self):
        req = GetConfig.Request()
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        data = json.loads(resp.json)
        return data["localization"]

    def send_set(self, config_dict):
        req = SetConfig.Request()
        req.json = json.dumps(config_dict)
        future = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_save(self, config_dict):
        req = SaveConfig.Request()
        req.json = json.dumps(config_dict)
        future = self.save_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class ConfigGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MCL Localization Tuner")
        self.setMinimumSize(500, 600)

        self.widgets = {}
        self.json_data = {}

        rclpy.init(args=None)
        self.ros_client = ConfigClient()

        self.layout = QVBoxLayout()
        self.scroll = QScrollArea()
        self.scroll_widget = QWidget()
        self.form_layout = QVBoxLayout()

        self.reload()

        self.scroll_widget.setLayout(self.form_layout)
        self.scroll.setWidget(self.scroll_widget)
        self.scroll.setWidgetResizable(True)
        self.layout.addWidget(self.scroll)

        btn_row = QHBoxLayout()
        self.btn_apply = QPushButton("Apply")
        self.btn_save = QPushButton("Save")
        self.btn_reload = QPushButton("Reload")

        self.btn_apply.clicked.connect(self.apply_config)
        self.btn_save.clicked.connect(self.save_config)
        self.btn_reload.clicked.connect(self.reload)

        btn_row.addWidget(self.btn_apply)
        btn_row.addWidget(self.btn_save)
        btn_row.addWidget(self.btn_reload)
        self.layout.addLayout(btn_row)
        self.setLayout(self.layout)

    def collect_json(self):
        out = {"localization": {}}
        for k, w in self.widgets.items():
            if isinstance(w, QCheckBox):
                out["localization"][k] = w.isChecked()
            else:
                out["localization"][k] = w.value()
        return out


    def build_form(self):
        self.widgets.clear()

        for key, val in self.json_data.items():
            row = QHBoxLayout()
            label = QLabel(key)
            label.setMinimumWidth(200)

            if isinstance(val, bool):
                w = QCheckBox()
                w.setChecked(val)
            elif isinstance(val, int):
                w = QSpinBox()
                w.setRange(-100000, 100000)
                w.setValue(val)
            else:
                w = QDoubleSpinBox()
                w.setDecimals(6)

                mag = abs(val)
                if mag > 100:
                    step = 10
                elif mag > 10:
                    step = 1
                elif mag > 1:
                    step = 0.1
                else:
                    step = 0.01

                w.setSingleStep(step)
                w.setRange(-1e6, 1e6)
                w.setValue(val)

            self.widgets[key] = w
            row.addWidget(label)
            row.addWidget(w)
            self.form_layout.addLayout(row)


    def apply_config(self):
        config = self.collect_json()
        self.ros_client.send_set(config)
        print("[CONFIG] Applied")

    def save_config(self):
        config = self.collect_json()
        self.ros_client.send_save(config)
        print("[CONFIG] Saved")

    def reload(self):
        while self.form_layout.count():
            item = self.form_layout.takeAt(0)
            if item.layout():
                while item.layout().count():
                    item.layout().takeAt(0)
        self.json_data = self.ros_client.fetch_config()
        self.build_form()
        self.apply_config()
        print("[CONFIG] Reloaded")


def main():
    app = QApplication(sys.argv)
    gui = ConfigGUI()
    gui.show()
    sys.exit(app.exec_())
