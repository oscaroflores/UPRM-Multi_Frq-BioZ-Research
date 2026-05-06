import sys
import math
import asyncio
import time
import struct
import csv
from datetime import datetime
from pathlib import Path
from qasync import QEventLoop, asyncSlot
import asyncio

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from bleak import BleakScanner, BleakClient

BIOZ_PACKET_MAGIC = 0x5A42
BIOZ_PACKET_VERSION = 1
BIOZ_PACKET_TYPE_SAMPLE = 1
BIOZ_PACKET = struct.Struct("<HBBIfff")
LOG_PACKET_MAGIC = 0x4C47
LOG_PACKET_VERSION = 1
LOG_PACKET_TYPE_CHUNK = 1
LOG_PACKET_HEADER = struct.Struct("<HBBHH")

class BLEBioZPlotter(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLE BioZ Dual-Frequency Plotter")

        self.devices = []
        self.client = None
        self.notify_char = None
        self.write_char = None
        self.write_without_response = False

        self.freqs = [4104, 131328]
        self.window_size = 500
        self.x_data = {f: [] for f in self.freqs}
        self.q_data = {f: [] for f in self.freqs}
        self.i_data = {f: [] for f in self.freqs}
        self.pending_data = {f: [] for f in self.freqs}
        self.log_file = None
        self.recording = False
        self.sd_logs = {}
        self.pending_download_path = None
        self.download_file = None
        self.download_name = None
        self.download_expected = 0
        self.download_received = 0
        self.download_next_seq = 0
        self.download_ack_task = None
        self.sd_samples = []
        self.sd_loaded_path = None

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

        self.main_tabs = QtWidgets.QTabWidget()
        self.live_tab = QtWidgets.QWidget()
        self.sd_tab = QtWidgets.QWidget()
        live_layout = QtWidgets.QVBoxLayout(self.live_tab)
        sd_layout = QtWidgets.QVBoxLayout(self.sd_tab)
        self.main_tabs.addTab(self.live_tab, "Live BioZ")
        self.main_tabs.addTab(self.sd_tab, "SD Explorer")
        layout.addWidget(self.main_tabs)

        log_control = QtWidgets.QHBoxLayout()
        self.sd_log_box = QtWidgets.QComboBox()
        self.refresh_logs_button = QtWidgets.QPushButton("Refresh SD Logs")
        self.download_log_button = QtWidgets.QPushButton("Download Selected")
        self.load_local_log_button = QtWidgets.QPushButton("Open Local .dat")
        self.export_csv_button = QtWidgets.QPushButton("Export CSV")
        self.log_status_label = QtWidgets.QLabel("")
        self.refresh_logs_button.setEnabled(False)
        self.download_log_button.setEnabled(False)
        self.export_csv_button.setEnabled(False)
        for w in [self.sd_log_box, self.refresh_logs_button, self.download_log_button,
                  self.load_local_log_button, self.export_csv_button, self.log_status_label]:
            log_control.addWidget(w)
        sd_layout.addLayout(log_control)

        self.sd_plot_tabs = QtWidgets.QTabWidget()
        self.sd_graphs, self.sd_curves, self.sd_scatters = [], [], []
        for idx, freq in enumerate(self.freqs):
            tab = QtWidgets.QWidget()
            vbox = QtWidgets.QVBoxLayout(tab)
            for j, label in enumerate(['Q', 'I']):
                g = pg.PlotWidget(title=f"SD {label} @ {freq} Hz")
                c = g.plot(pen=['y', 'g', 'c', 'r'][idx * 2 + j])
                s = pg.ScatterPlotItem(brush=['y', 'g', 'c', 'r'][idx * 2 + j], size=4)
                g.addItem(s)
                self.sd_graphs.append(g)
                self.sd_curves.append(c)
                self.sd_scatters.append(s)
                vbox.addWidget(g)
            self.sd_plot_tabs.addTab(tab, f"{freq} Hz")

        self.csv_view = QtWidgets.QPlainTextEdit()
        self.csv_view.setReadOnly(True)
        self.csv_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.csv_view.setPlaceholderText("Downloaded or opened .dat logs appear here as CSV text.")

        sd_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        sd_splitter.addWidget(self.sd_plot_tabs)
        sd_splitter.addWidget(self.csv_view)
        sd_splitter.setStretchFactor(0, 3)
        sd_splitter.setStretchFactor(1, 2)
        sd_layout.addWidget(sd_splitter)

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
        live_layout.addLayout(slider_layout)

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
        live_layout.addWidget(self.tabs)

        self.scan_button.clicked.connect(self.scan_devices)
        self.connect_button.clicked.connect(self.connect_device)
        self.disconnect_button.clicked.connect(self.disconnect_device)
        self.start_button.clicked.connect(self.send_start)
        self.stop_button.clicked.connect(self.send_stop)
        self.clear_button.clicked.connect(self.clear_plots)
        self.refresh_logs_button.clicked.connect(self.refresh_sd_logs)
        self.download_log_button.clicked.connect(self.download_selected_log)
        self.load_local_log_button.clicked.connect(self.open_local_log)
        self.export_csv_button.clicked.connect(self.export_sd_csv)

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
                            self.write_without_response = "write-without-response" in props
                            self.start_button.setEnabled(True)
                            self.refresh_logs_button.setEnabled(True)
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
        self.write_without_response = False
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.refresh_logs_button.setEnabled(False)
        self.download_log_button.setEnabled(False)
        if self.download_file:
            self.download_file.close()
            self.download_file = None
        self.pending_download_path = None


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

    @asyncSlot()
    async def refresh_sd_logs(self):
        if not self.client or not self.write_char:
            return
        self.sd_logs.clear()
        self.sd_log_box.clear()
        self.download_log_button.setEnabled(False)
        self.log_status_label.setText("Listing SD logs...")
        await self.client.write_gatt_char(self.write_char, b"logs:list")

    @asyncSlot()
    async def download_selected_log(self):
        if not self.client or not self.write_char:
            return
        name = self.sd_log_box.currentData()
        if not name:
            return

        default_path = str(name)
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save SD log",
            default_path,
            "BioZ binary logs (*.dat);;All files (*)",
        )
        if not path:
            return

        self.pending_download_path = path
        self.download_received = 0
        self.download_expected = self.sd_logs.get(name, 0)
        self.download_next_seq = 0
        self.log_status_label.setText(f"Starting download: {name}")
        await self.client.write_gatt_char(self.write_char, f"logs:get:{name}".encode())

    def queue_download_ack(self):
        if self.download_ack_task and not self.download_ack_task.done():
            return
        self.download_ack_task = asyncio.create_task(self.send_download_ack())

    async def send_download_ack(self):
        await asyncio.sleep(0)
        if not self.client or not self.client.is_connected or not self.write_char:
            return
        cmd = f"logs:ack:{self.download_next_seq}".encode()
        await self.client.write_gatt_char(
            self.write_char,
            cmd,
            response=not self.write_without_response,
        )

    def open_local_log(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open BioZ .dat log",
            "",
            "BioZ binary logs (*.dat);;All files (*)",
        )
        if path:
            self.load_sd_dat(path)

    def export_sd_csv(self):
        if not self.sd_samples:
            return

        default_path = "bioz_log.csv"
        if self.sd_loaded_path:
            default_path = str(self.sd_loaded_path.with_suffix(".csv"))

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export BioZ CSV",
            default_path,
            "CSV files (*.csv);;All files (*)",
        )
        if not path:
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "Q", "I", "F", "phase_deg"])
            for timestamp, q, i, freq, phase in self.sd_samples:
                writer.writerow([timestamp, q, i, freq, phase])

        self.log_status_label.setText(f"Exported CSV: {Path(path).name}")

    def load_sd_dat(self, path):
        path = Path(path)
        data = path.read_bytes()
        samples = []
        invalid = 0

        usable_len = (len(data) // BIOZ_PACKET.size) * BIOZ_PACKET.size
        for offset in range(0, usable_len, BIOZ_PACKET.size):
            magic, version, packet_type, timestamp, q, i, freq = BIOZ_PACKET.unpack_from(data, offset)
            if (magic != BIOZ_PACKET_MAGIC or
                    version != BIOZ_PACKET_VERSION or
                    packet_type != BIOZ_PACKET_TYPE_SAMPLE):
                invalid += 1
                continue

            freq = int(round(freq))
            phase = math.atan2(q, i) * 180.0 / math.pi
            samples.append((timestamp, q, i, freq, phase))

        self.sd_samples = samples
        self.sd_loaded_path = path
        self.export_csv_button.setEnabled(bool(samples))
        self.populate_sd_csv_preview()
        self.update_sd_plots()

        trailing = len(data) - usable_len
        status = f"Loaded {path.name}: {len(samples)} samples"
        if invalid or trailing:
            status += f" ({invalid} invalid records, {trailing} trailing bytes)"
        self.log_status_label.setText(status)

    def populate_sd_csv_preview(self):
        preview_limit = 5000
        lines = ["timestamp,Q,I,F,phase_deg"]
        for timestamp, q, i, freq, phase in self.sd_samples[:preview_limit]:
            lines.append(f"{timestamp},{q:.6f},{i:.6f},{freq},{phase:.2f}")
        self.csv_view.setPlainText("\n".join(lines))

    def update_sd_plots(self):
        grouped = {f: {"t": [], "q": [], "i": []} for f in self.freqs}
        for timestamp, q, i, freq, _ in self.sd_samples:
            if freq not in grouped:
                continue
            grouped[freq]["t"].append(timestamp)
            grouped[freq]["q"].append(q)
            grouped[freq]["i"].append(i)

        for idx, freq in enumerate(self.freqs):
            t = grouped[freq]["t"]
            q = grouped[freq]["q"]
            i = grouped[freq]["i"]
            self.sd_curves[idx * 2].setData(t, q)
            self.sd_scatters[idx * 2].setData(t, q)
            self.sd_curves[idx * 2 + 1].setData(t, i)
            self.sd_scatters[idx * 2 + 1].setData(t, i)
            if t:
                self.sd_graphs[idx * 2].setXRange(t[0], t[-1], padding=0.01)
                self.sd_graphs[idx * 2 + 1].setXRange(t[0], t[-1], padding=0.01)

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
            if len(data) >= LOG_PACKET_HEADER.size:
                magic, version, packet_type, seq, payload_len = LOG_PACKET_HEADER.unpack(
                    data[:LOG_PACKET_HEADER.size]
                )
                if (magic == LOG_PACKET_MAGIC and
                        version == LOG_PACKET_VERSION and
                        packet_type == LOG_PACKET_TYPE_CHUNK):
                    payload = data[LOG_PACKET_HEADER.size:LOG_PACKET_HEADER.size + payload_len]
                    if self.download_file:
                        self.download_file.write(payload)
                        self.download_received += len(payload)
                        self.download_next_seq = seq + 1
                        self.queue_download_ack()
                        if self.download_expected:
                            pct = 100.0 * self.download_received / self.download_expected
                            self.log_status_label.setText(
                                f"Downloading {self.download_name}: {self.download_received}/{self.download_expected} B ({pct:.1f}%)"
                            )
                    return

            if len(data) == BIOZ_PACKET.size:
                magic, version, packet_type, timestamp, q, i, freq = BIOZ_PACKET.unpack(data)
                if (magic == BIOZ_PACKET_MAGIC and
                        version == BIOZ_PACKET_VERSION and
                        packet_type == BIOZ_PACKET_TYPE_SAMPLE):
                    freq = int(round(freq))
                    if freq not in self.freqs:
                        return
                    phase = math.atan2(q, i) * 180.0 / math.pi
                    self.pending_data[freq].append((timestamp, q, i, phase))
                    if self.log_file:
                        self.log_file.write(f"{timestamp},{q},{i},{freq},{phase:.2f}\n")
                    return

            line = data.decode("utf-8").strip()

            if line == "startPhys":
                print("[BUTTON] Physical Start Triggered")
                asyncio.create_task(self.debug_start_wrapper())
                return
            elif line == "stopPhys":
                print("[BUTTON] Physical Stop Triggered")
                asyncio.create_task(self.debug_stop_wrapper())
                return
            elif line == "logs:begin":
                self.sd_logs.clear()
                self.sd_log_box.clear()
                self.download_log_button.setEnabled(False)
                self.log_status_label.setText("Receiving SD log list...")
                return
            elif line.startswith("logs:file:"):
                try:
                    name, size_text = line[len("logs:file:"):].rsplit(":", 1)
                    size = int(size_text)
                except ValueError:
                    return
                self.sd_logs[name] = size
                self.sd_log_box.addItem(f"{name} ({size} B)", name)
                return
            elif line == "logs:end":
                count = len(self.sd_logs)
                self.download_log_button.setEnabled(count > 0)
                self.log_status_label.setText(f"{count} SD log(s) available")
                return
            elif line.startswith("logs:read_begin:"):
                try:
                    name, size_text = line[len("logs:read_begin:"):].rsplit(":", 1)
                    size = int(size_text)
                except ValueError:
                    return
                if self.download_file:
                    self.download_file.close()
                self.download_name = name
                self.download_expected = size
                self.download_received = 0
                self.download_next_seq = 0
                path = self.pending_download_path or name
                self.download_file = open(path, "wb")
                self.log_status_label.setText(f"Downloading {name}: 0/{size} B")
                return
            elif line.startswith("logs:read_end:"):
                completed_path = self.pending_download_path
                if self.download_file:
                    self.download_file.close()
                    self.download_file = None
                self.pending_download_path = None
                if completed_path:
                    self.load_sd_dat(completed_path)
                self.log_status_label.setText(
                    f"Downloaded {self.download_name}: {self.download_received} B"
                )
                return
            elif line.startswith("logs:error:"):
                if self.download_file:
                    self.download_file.close()
                    self.download_file = None
                self.pending_download_path = None
                self.log_status_label.setText(line)
                return
            elif line == "logs:cancelled":
                self.log_status_label.setText("SD log transfer cancelled")
                return



            # Otherwise treat as normal sample line
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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    gui = BLEBioZPlotter()
    gui.resize(1000, 900)
    gui.show()

    with loop:
        loop.run_forever()
