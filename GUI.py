import sys
import time
import math
import serial
import serial.tools.list_ports
import threading
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QTabWidget
import pyqtgraph as pg

class SerialPlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Dual-Frequency BioZ Plotter")

        main_layout = QtWidgets.QVBoxLayout(self)
        control_layout = QtWidgets.QHBoxLayout()
        slider_layout = QtWidgets.QHBoxLayout()

        # COM Port selector
        self.port_box = QtWidgets.QComboBox()
        self.refresh_ports()
        control_layout.addWidget(self.port_box)

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)
        control_layout.addWidget(self.connect_button)
        control_layout.addWidget(self.disconnect_button)

        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        control_layout.addWidget(self.start_button)
        control_layout.addWidget(self.stop_button)

        self.clear_button = QtWidgets.QPushButton("Clear Plots")
        control_layout.addWidget(self.clear_button)

        self.log_checkbox = QtWidgets.QCheckBox("Enable Logging")
        control_layout.addWidget(self.log_checkbox)

        main_layout.addLayout(control_layout)

        # Slider
        self.slider_label = QtWidgets.QLabel("Window Size: 500")
        self.window_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.window_slider.setMinimum(100)
        self.window_slider.setMaximum(2000)
        self.window_slider.setValue(500)
        self.window_slider.setTickInterval(100)
        self.window_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        slider_layout.addWidget(self.slider_label)
        slider_layout.addWidget(self.window_slider)
        main_layout.addLayout(slider_layout)

        # Frequencies and data containers
        self.freqs = [4104, 131328]
        self.window_size = 500
        self.x_data = {f: [] for f in self.freqs}
        self.q_data = {f: [] for f in self.freqs}
        self.i_data = {f: [] for f in self.freqs}
        self.pending_data = {f: [] for f in self.freqs}

        # Tabs and Graphs
        self.tabs = QTabWidget()
        self.graphs, self.curves, self.scatters = [], [], []
        colors = ['y', 'g', 'c', 'r']
        titles = ['Q', 'I']

        for idx, freq in enumerate(self.freqs):
            tab = QtWidgets.QWidget()
            vbox = QtWidgets.QVBoxLayout(tab)
            for j, title in enumerate(titles):
                graph = pg.PlotWidget(title=f"{title} @ F{idx+1} ({freq} Hz)")
                graph.setMinimumHeight(200)
                graph.setMouseEnabled(x=False, y=False)
                vb = graph.getViewBox()
                vb.setMouseMode(pg.ViewBox.PanMode)
                vb.setMenuEnabled(False)
                vb.enableAutoRange(y=True)
                vb.enableAutoRange(x=False)

                curve = graph.plot(pen=colors[idx * 2 + j])
                scatter = pg.ScatterPlotItem(brush=colors[idx * 2 + j], size=5)
                graph.addItem(scatter)

                self.graphs.append(graph)
                self.curves.append(curve)
                self.scatters.append(scatter)
                vbox.addWidget(graph)

            self.tabs.addTab(tab, f"F{idx+1} ({freq} Hz)")

        main_layout.addWidget(self.tabs)

        # Serial and threading
        self.serial = None
        self.read_thread = None
        self.running = False
        self.log_file = None

        # Connect signals
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        self.start_button.clicked.connect(self.send_start)
        self.stop_button.clicked.connect(self.send_stop)
        self.window_slider.valueChanged.connect(self.update_window_size)
        self.clear_button.clicked.connect(self.clear_plots)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_box.clear()
        self.port_box.addItems([port.device for port in ports])

    def connect_serial(self):
        port_name = self.port_box.currentText()
        try:
            self.serial = serial.Serial(port_name, 115200, timeout=1)
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print("Connected to:", port_name)
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.start_button.setEnabled(True)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open port: {e}")

    def disconnect_serial(self):
        try:
            if self.running:
                self.running = False
                self.read_thread.join()
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception as e:
            print("Disconnect error:", e)
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def send_start(self):
        if self.serial and self.serial.is_open:
            if self.log_checkbox.isChecked():
                try:
                    filename = time.strftime("bioz_log_%Y%m%d_%H%M%S.csv")
                    self.log_file = open(filename, "w")
                    self.log_file.write("timestamp,Q,I,F,phase_deg\n")
                except Exception as e:
                    QtWidgets.QMessageBox.critical(self, "Logging Error", str(e))
                    return

            from datetime import datetime
            now = datetime.now()
            timestamp_str = now.strftime("start %Y-%m-%d %H:%M:%S\r\n")
            self.serial.write(timestamp_str.encode())

            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial)
            self.read_thread.start()
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)

    def send_stop(self):
        if self.serial and self.serial.is_open:
            self.serial.write(b'stop\r\n')
            self.running = False
            self.read_thread.join()
            self.stop_button.setEnabled(False)
            self.start_button.setEnabled(True)
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def update_window_size(self, value):
        self.window_size = value
        self.slider_label.setText(f"Window Size: {value}")

    def clear_plots(self):
        for freq in self.freqs:
            self.x_data[freq].clear()
            self.q_data[freq].clear()
            self.i_data[freq].clear()
            self.pending_data[freq].clear()
        for curve in self.curves:
            curve.setData([], [])
        for scatter in self.scatters:
            scatter.setData([], [])

    def read_serial(self):
        while self.running:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 4:
                    continue
                timestamp, q_raw, i_raw, freq = map(float, parts[:4])
                freq = int(round(freq))
                if freq not in self.freqs:
                    continue

                # Calculate phase in degrees
                phase_rad = math.atan2(q_raw, i_raw)
                phase_deg = phase_rad * (180.0 / math.pi)

                self.pending_data[freq].append((timestamp, q_raw, i_raw, phase_deg))

                if self.log_file:
                    self.log_file.write(f"{timestamp},{q_raw},{i_raw},{freq},{phase_deg:.2f}\n")

            except Exception as e:
                print("Serial read error:", e)

    def update_plots(self):
        for freq in self.freqs:
            f_idx = self.freqs.index(freq)
            updates = self.pending_data[freq]
            if not updates:
                continue

            for timestamp, q, i, _ in updates:
                self.x_data[freq].append(timestamp)
                self.q_data[freq].append(q)
                self.i_data[freq].append(i)

            self.pending_data[freq] = []

            for dset in [self.x_data, self.q_data, self.i_data]:
                if len(dset[freq]) > self.window_size:
                    dset[freq] = dset[freq][-self.window_size:]

            self.curves[f_idx * 2].setData(self.x_data[freq], self.q_data[freq])
            self.scatters[f_idx * 2].setData(self.x_data[freq], self.q_data[freq])
            self.curves[f_idx * 2 + 1].setData(self.x_data[freq], self.i_data[freq])
            self.scatters[f_idx * 2 + 1].setData(self.x_data[freq], self.i_data[freq])

            if len(self.x_data[freq]) >= 10:
                xmin = self.x_data[freq][0]
                xmax = self.x_data[freq][-1]
                self.graphs[f_idx * 2].setXRange(xmin, xmax, padding=0.01)
                self.graphs[f_idx * 2 + 1].setXRange(xmin, xmax, padding=0.01)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    plotter = SerialPlotter()
    plotter.resize(1000, 900)
    plotter.show()
    sys.exit(app.exec_())
