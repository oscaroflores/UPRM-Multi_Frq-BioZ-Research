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
        self.log_file = None

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

        # Graphs
        self.tabs = QtWidgets.QTabWidget()
        self.graphs, self.curves, self.scatters = [], [], []
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
        for c in self.curves: c.setData([], [])
        for s in self.scatters: s.setData([], [])

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
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    @asyncSlot()
    async def send_stop(self):
        if self.client and self.write_char:
            await self.client.write_gatt_char(self.write_char, b"stop")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if self.log_file:
            self.log_file.close()
            self.log_file = None


    def handle_notification(self, _, data):
        try:
            line = data.decode("utf-8").strip()
            parts = line.split(",")
            if len(parts) != 4:
                return
            timestamp, q, i, freq = map(float, parts)
            freq = int(round(freq))
            if freq not in self.freqs:
                return
            phase = math.atan2(q, i) * 180.0 / math.pi
            self.pending_data[freq].append((timestamp, q, i, phase))
            if self.log_file:
                self.log_file.write(f"{timestamp},{q},{i},{freq},{phase:.2f}\n")
        except Exception as e:
            print("[Notify Error]", e)

    def update_plots(self):
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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    gui = BLEBioZPlotter()
    gui.resize(1000, 900)
    gui.show()

    with loop:
        loop.run_forever()
