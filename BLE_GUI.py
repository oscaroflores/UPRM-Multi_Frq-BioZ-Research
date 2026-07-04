import sys
import math
import asyncio
import time
from datetime import datetime
from qasync import QEventLoop, asyncSlot
import asyncio

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from bleak import BleakScanner, BleakClient

class BLEBioZPlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLE BioZ Dual-Frequency Plotter")

        self.devices = []
        self.client = None
        self.notify_char = None
        self.write_char = None

        self.freqs = [4104, 131328]
        self.window_size = 500
        self.x_data = {f: [] for f in self.freqs}
        self.q_data = {f: [] for f in self.freqs}
        self.i_data = {f: [] for f in self.freqs}
        self.pending_data = {f: [] for f in self.freqs}
        self.accel_time = []
        self.accel_data = {"x": [], "y": [], "z": []}
        self.velocity_data = {"x": [], "y": [], "z": []}
        self.pending_accel = []
        self.accel_index = 0
        self.log_file = None
        self.recording = False
        self.bioz_coeff = 1.0
        self.rx_buffer = ""
        self.combined_span_ms = 10000  # fallback span for combined plot

        layout = QtWidgets.QVBoxLayout(self)

        control = QtWidgets.QHBoxLayout()
        self.device_box = QtWidgets.QComboBox()
        self.scan_button = QtWidgets.QPushButton("Scan")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.clear_button = QtWidgets.QPushButton("Clear Plots")
        self.log_checkbox = QtWidgets.QCheckBox("Enable Logging")

        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        
        for w in [self.device_box, self.scan_button, self.connect_button, self.disconnect_button,
                  self.start_button, self.stop_button, self.clear_button, self.log_checkbox]:
            control.addWidget(w)
        layout.addLayout(control)

        # Slider
        slider_layout = QtWidgets.QHBoxLayout()
        self.slider_label = QtWidgets.QLabel("Window Size: 500")
        self.window_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.window_slider.setMinimum(100)
        self.window_slider.setMaximum(2000)
        self.window_slider.setValue(500)
        self.window_slider.valueChanged.connect(self.update_window_size)
        slider_layout.addWidget(self.slider_label)
        slider_layout.addWidget(self.window_slider)
        layout.addLayout(slider_layout)

        # Live log viewer.
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setPlaceholderText("Incoming BLE data will appear here...")
        layout.addWidget(self.log_view)

        # Graphs
        self.tabs = QtWidgets.QTabWidget()
        self.graphs, self.curves, self.scatters = [], [], []
        self.accel_graphs, self.accel_curves, self.accel_scatters = [], [], []
        self.combined_plot = None
        self.combined_curves = {}
        self.combined_scatters = {}
        self.combined_imu_plot = None
        self.combined_imu_curves = {}
        self.combined_imu_scatters = {}
        self.combined_vel_plot = None
        self.combined_vel_curves = {}
        self.combined_vel_scatters = {}
        colors = ['y', 'g', 'c', 'r']
        for idx, freq in enumerate(self.freqs):
            tab = QtWidgets.QWidget()
            vbox = QtWidgets.QVBoxLayout(tab)
            for j, label in enumerate(['Q', 'I']):
                g = pg.PlotWidget(title=f"{label} @ {freq} Hz")
                c = g.plot(pen=colors[idx * 2 + j])
                s = pg.ScatterPlotItem(brush=colors[idx * 2 + j], size=5)
                g.addItem(s)
                self.graphs.append(g)
                self.curves.append(c)
                self.scatters.append(s)
                vbox.addWidget(g)
            self.tabs.addTab(tab, f"{freq} Hz")

        accel_tab = QtWidgets.QWidget()
        accel_layout = QtWidgets.QVBoxLayout(accel_tab)
        accel_colors = ['#e67e22', '#2980b9', '#2ecc71']
        for label, color in zip(["Accel X", "Accel Y", "Accel Z"], accel_colors):
            g = pg.PlotWidget(title=label)
            c = g.plot(pen=color)
            s = pg.ScatterPlotItem(brush=color, size=5)
            g.addItem(s)
            accel_layout.addWidget(g)
            self.accel_graphs.append(g)
            self.accel_curves.append(c)
            self.accel_scatters.append(s)
            self.graphs.append(g)
            self.curves.append(c)
            self.scatters.append(s)
        self.tabs.addTab(accel_tab, "Accelerometer")

        # Combined BioZ view (both freqs, Q/I on one plot)
        combined_tab = QtWidgets.QWidget()
        combined_layout = QtWidgets.QVBoxLayout(combined_tab)
        self.combined_plot = pg.PlotWidget(title="BioZ Q/I - Dual Frequency")
        self.combined_plot.addLegend()

        combined_colors = {
            (self.freqs[0], "Q"): ('#3498db', QtCore.Qt.SolidLine),
            (self.freqs[0], "I"): ('#e67e22', QtCore.Qt.SolidLine),
            (self.freqs[1], "Q"): ('#2ecc71', QtCore.Qt.DashLine),
            (self.freqs[1], "I"): ('#c0392b', QtCore.Qt.DashLine),
        }
        for (freq, comp), (color, style) in combined_colors.items():
            pen = pg.mkPen(color=color, width=2, style=style)
            curve = self.combined_plot.plot(pen=pen, name=f"{comp} @ {freq} Hz")
            self.combined_curves[(freq, comp)] = curve
            scatter = pg.ScatterPlotItem(brush=color, size=5, pen=pg.mkPen(color=color))
            self.combined_plot.addItem(scatter)
            self.combined_scatters[(freq, comp)] = scatter

        combined_layout.addWidget(self.combined_plot, 3)

        # Combined IMU view (all axes together)
        self.combined_imu_plot = pg.PlotWidget(title="IMU Accel (All Axes)")
        self.combined_imu_plot.addLegend()
        imu_colors = {
            "x": '#e67e22',
            "y": '#2980b9',
            "z": '#2ecc71',
        }
        for axis, color in imu_colors.items():
            pen = pg.mkPen(color=color, width=2)
            curve = self.combined_imu_plot.plot(pen=pen, name=f"Accel {axis.upper()}")
            self.combined_imu_curves[axis] = curve
            scatter = pg.ScatterPlotItem(brush=color, size=5, pen=pg.mkPen(color=color))
            self.combined_imu_plot.addItem(scatter)
            self.combined_imu_scatters[axis] = scatter
        combined_layout.addWidget(self.combined_imu_plot, 1)

        self.tabs.addTab(combined_tab, "Combined BioZ")
        layout.addWidget(self.tabs)

        self.scan_button.clicked.connect(self.scan_devices)
        self.connect_button.clicked.connect(self.connect_device)
        self.disconnect_button.clicked.connect(self.disconnect_device)
        self.start_button.clicked.connect(self.send_start)
        self.stop_button.clicked.connect(self.send_stop)
        self.clear_button.clicked.connect(self.clear_plots)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)
        asyncio.ensure_future(self.scan_devices())


    def update_window_size(self, val):
        self.window_size = val
        self.slider_label.setText(f"Window Size: {val}")

    def clear_plots(self):
        for f in self.freqs:
            self.x_data[f].clear()
            self.q_data[f].clear()
            self.i_data[f].clear()
            self.pending_data[f].clear()
        self.accel_time.clear()
        for axis in self.accel_data:
            self.accel_data[axis].clear()
        self.pending_accel.clear()
        self.accel_index = 0
        for c in self.curves: c.setData([], [])
        for s in self.scatters: s.setData([], [])
        for curve in self.combined_curves.values():
            curve.setData([], [])
        for scatter in self.combined_scatters.values():
            scatter.setData([], [])
        for curve in self.combined_imu_curves.values():
            curve.setData([], [])
        for scatter in self.combined_imu_scatters.values():
            scatter.setData([], [])

    @asyncSlot()
    async def scan_devices(self):
        self.devices = await BleakScanner.discover()
        self.device_box.clear()
        filtered = [d for d in self.devices if d.name and d.name.strip()]
        self.devices = filtered  # overwrite with filtered list
        for d in self.devices:
            self.device_box.addItem(f"{d.name} ({d.address})")



    @asyncSlot()
    async def connect_device(self):
        idx = self.device_box.currentIndex()
        if idx < 0: return

        device = self.devices[idx]
        try:
            self.client = BleakClient(device)
            await self.client.connect()
            self.disconnect_button.setEnabled(True)
            await self.auto_select_characteristic()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Connect Error", str(e))
    @asyncSlot()
    async def auto_select_characteristic(self):
        try:
            services = self.client.services  # Already populated after connect() in Bleak 1.0+
            for service in services:
                for char in service.characteristics:
                    props = char.properties
                    if "notify" in props and ("write" in props or "write-without-response" in props):
                        try:
                            await self.client.start_notify(char.uuid, self.handle_notification)
                            self.notify_char = char.uuid
                            self.write_char = char.uuid
                            self.start_button.setEnabled(True)
                            print(f"[Characteristic] Using UUID {char.uuid}")
                            return
                        except Exception as e:
                            print(f"[Notify Error] {e}")
            QtWidgets.QMessageBox.warning(self, "BLE Error", "No valid notify+write characteristic found.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Char Discovery Error", str(e))


    @asyncSlot()
    async def disconnect_device(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self.client = None
        self.notify_char = None
        self.write_char = None
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)


    @asyncSlot()
    async def send_start(self):
        if not self.client or not self.write_char:
            return
        if self.log_checkbox.isChecked():
            filename = time.strftime("bioz_log_%Y%m%d_%H%M%S.csv")
            self.log_file = open(filename, "w")
            self.log_file.write("timestamp,Q,I,F,phase_deg\n")
        now = datetime.now()
        msg = now.strftime("start@%Y%m%d@%H%M%S")
        await self.client.write_gatt_char(self.write_char, msg.encode())
        self.recording = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)


    @asyncSlot()
    async def send_stop(self):
        if self.client and self.write_char:
            await self.client.write_gatt_char(self.write_char, b"stop")
        self.recording = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def update_plots(self):
        combined_has_data = False
        first_points = []
        last_points = []

        for freq in self.freqs:
            idx = self.freqs.index(freq)
            updates = self.pending_data[freq]
            if not updates:
                continue
            for t, q, i, _ in updates:
                self.x_data[freq].append(t)
                self.q_data[freq].append(q)
                self.i_data[freq].append(i)
            self.pending_data[freq] = []

            for arr in [self.x_data, self.q_data, self.i_data]:
                if len(arr[freq]) > self.window_size:
                    arr[freq] = arr[freq][-self.window_size:]

            self.curves[idx * 2].setData(self.x_data[freq], self.q_data[freq])
            self.scatters[idx * 2].setData(self.x_data[freq], self.q_data[freq])
            self.curves[idx * 2 + 1].setData(self.x_data[freq], self.i_data[freq])
            self.scatters[idx * 2 + 1].setData(self.x_data[freq], self.i_data[freq])

            if len(self.x_data[freq]) >= 10:
                xmin = self.x_data[freq][0]
                xmax = self.x_data[freq][-1]
                self.graphs[idx * 2].setXRange(xmin, xmax, padding=0.01)
                self.graphs[idx * 2 + 1].setXRange(xmin, xmax, padding=0.01)

            # Combined plot updates
            if self.x_data[freq]:
                combined_has_data = True
                x_first = self.x_data[freq][0]
                x_last = self.x_data[freq][-1]
                first_points.append(x_first)
                last_points.append(x_last)
                self.combined_curves[(freq, "Q")].setData(self.x_data[freq], self.q_data[freq])
                self.combined_curves[(freq, "I")].setData(self.x_data[freq], self.i_data[freq])
                self.combined_scatters[(freq, "Q")].setData(self.x_data[freq], self.q_data[freq])
                self.combined_scatters[(freq, "I")].setData(self.x_data[freq], self.i_data[freq])

        if self.pending_accel:
            for entry in self.pending_accel:
                if len(entry) == 4:
                    t_ms, ax, ay, az = entry
                    last_t = self.accel_time[-1] if self.accel_time else None
                    if last_t is not None and t_ms <= last_t:
                        continue  # drop out-of-order or duplicate timestamps
                    self.accel_time.append(t_ms)
                    self.accel_data["x"].append(ax)
                    self.accel_data["y"].append(ay)
                    self.accel_data["z"].append(az)
                else:
                    ax, ay, az = entry
                    self.accel_time.append(self.accel_index)
                    self.accel_index += 1
                    self.accel_data["x"].append(ax)
                    self.accel_data["y"].append(ay)
                    self.accel_data["z"].append(az)
            self.pending_accel = []

            if len(self.accel_time) > self.window_size:
                self.accel_time = self.accel_time[-self.window_size:]
                for axis in self.accel_data:
                    self.accel_data[axis] = self.accel_data[axis][-self.window_size:]

            for idx, axis in enumerate(["x", "y", "z"]):
                self.accel_curves[idx].setData(self.accel_time, self.accel_data[axis])
                self.accel_scatters[idx].setData(self.accel_time, self.accel_data[axis])
                # Combined IMU plot
                self.combined_imu_curves[axis].setData(self.accel_time, self.accel_data[axis])
                self.combined_imu_scatters[axis].setData(self.accel_time, self.accel_data[axis])

            if len(self.accel_time) >= 10:
                xmin = self.accel_time[0]
                xmax = self.accel_time[-1]
                for g in self.accel_graphs:
                    g.setXRange(xmin, xmax, padding=0.01)
                self.combined_imu_plot.setXRange(xmin, xmax, padding=0.01)

        if combined_has_data and first_points and last_points:
            # Use overlap of timelines to avoid bouncing as window slides
            x_min = max(first_points)
            x_max = min(last_points)
            if x_max <= x_min:
                x_max = max(last_points)
                x_min = x_max - self.combined_span_ms
            self.combined_plot.setXRange(x_min, x_max, padding=0.01)

    async def debug_start_wrapper(self):
        print("[DEBUG] Calling send_start() coroutine")
        await self.send_start()
        print("[DEBUG] send_start() finished")

    async def debug_stop_wrapper(self):
        print("[DEBUG] Calling send_stop() coroutine")
        await self.send_stop()
        print("[DEBUG] send_stop() finished")

    def handle_notification(self, _, data):
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            return

        self.rx_buffer += text
        lines = self.rx_buffer.splitlines()
        if self.rx_buffer.endswith(("\n", "\r")):
            self.rx_buffer = ""
        else:
            self.rx_buffer = lines[-1] if lines else ""
            lines = lines[:-1]

        for raw_line in lines:
            line = raw_line.strip()
            if not line:
                continue
            try:
                self._process_message(line)
            except Exception as e:
                print("[Notify Error]", e)

    def _map_frequency(self, value):
        if not self.freqs:
            return None
        target = float(value)
        freq = min(self.freqs, key=lambda f: abs(f - target))
        if abs(freq - target) < 5000:
            return freq
        return None

    def _process_message(self, line):
        self._append_log_line(line)

        if line.startswith("bioz_coeff:"):
            try:
                self.bioz_coeff = float(line.split(":", 1)[1])
                print(f"[BIOZ] Coefficient set to {self.bioz_coeff}")
            except ValueError:
                pass
            return

        if line == "startPhys":
            print("[BUTTON] Physical Start Triggered")
            asyncio.create_task(self.debug_start_wrapper())
            return
        if line == "stopPhys":
            print("[BUTTON] Physical Stop Triggered")
            asyncio.create_task(self.debug_stop_wrapper())
            return

        parts = line.split(",")
        if len(parts) == 4:
            cleaned = [p.strip().replace("(", "").replace(")", "") for p in parts]
            try:
                vals = [float(p) for p in cleaned]
            except ValueError:
                return

            freq_candidate = self._map_frequency(vals[3])
            if freq_candidate is not None:
                timestamp, q, i, _ = vals
                phase = math.atan2(q, i) * 180.0 / math.pi
                self.pending_data[freq_candidate].append((timestamp, q, i, phase))
                if self.log_file:
                    self.log_file.write(
                        f"{timestamp},{q},{i},{freq_candidate},{phase:.2f}\n"
                    )
            else:
                t_ms, ax, ay, az = vals
                self.pending_accel.append((t_ms, ax, ay, az))
            return

        if len(parts) == 3:
            try:
                cleaned = [p.strip().replace("(", "").replace(")", "") for p in parts]
                vals = [float(p) for p in cleaned]
            except ValueError:
                return
            self.pending_accel.append(tuple(vals))
            return

    def _append_log_line(self, line):
        QtCore.QMetaObject.invokeMethod(
            self.log_view,
            "appendPlainText",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, line),
        )


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    gui = BLEBioZPlotter()
    gui.resize(1000, 900)
    gui.show()

    with loop:
        loop.run_forever()
