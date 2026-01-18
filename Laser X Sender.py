import sys
import time
import serial
import serial.tools.list_ports
import re
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtMultimedia import QSound  # at the top of your file

# -------------------- CNC Sender Thread (Planner-aware GRBL) --------------------
class SenderThread(QtCore.QThread):
    log_signal = QtCore.pyqtSignal(str)
    tool_signal = QtCore.pyqtSignal(float, float, float)
    done_signal = QtCore.pyqtSignal()
    progress_signal = QtCore.pyqtSignal(int)  # <-- progress signal

    RX_BUFFER = 128      # GRBL RX buffer size
    SAFETY = 10          # Safety margin to avoid overflow

    def __init__(self, ser, gcode_lines, simulate=False):
        super().__init__()
        self.ser = ser
        self.gcode_lines = gcode_lines
        self.simulate = simulate
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        if self.simulate:
            self._run_simulation()
            return

        in_flight = []
        bytes_in_flight = 0
        idx = 0
        x = y = z = 0.0
        last_update = time.time()

        while not self._stop:
            while idx < len(self.gcode_lines) and bytes_in_flight < (self.RX_BUFFER - self.SAFETY):
                raw = self.gcode_lines[idx].strip()
                idx += 1
                if not raw or raw.startswith("(") or raw.startswith(";"):
                    continue

                line = raw + "\n"
                size = len(line.encode())
                if bytes_in_flight + size > (self.RX_BUFFER - self.SAFETY):
                    idx -= 1
                    break

                # Parse X/Y/Z for GUI
                for axis, val in re.findall(r"([XYZ])([-+]?\d*\.?\d+)", raw.upper()):
                    if axis == "X": x = float(val)
                    elif axis == "Y": y = float(val)
                    elif axis == "Z": z = float(val)

                # Emit tool position (throttled)
                if time.time() - last_update > 0.01:
                    self.tool_signal.emit(x, y, z)
                    last_update = time.time()

                # Send to GRBL
                sent = False
                while not sent and not self._stop:
                    try:
                        self.ser.write(line.encode())
                        sent = True
                        self.log_signal.emit(f">> {raw}")
                    except serial.SerialTimeoutException:
                        self.log_signal.emit("Write timeout, retrying...")
                        time.sleep(0.01)

                in_flight.append(size)
                bytes_in_flight += size

                # Emit progress
                progress = int((idx / len(self.gcode_lines)) * 100)
                self.progress_signal.emit(progress)

            # Read response from GRBL
            try:
                resp = self.ser.readline().decode(errors="ignore").strip()
                if resp:
                    self.log_signal.emit(f"<< {resp}")
                    if resp.startswith("ok") or resp.startswith("error"):
                        if in_flight:
                            bytes_in_flight -= in_flight.pop(0)
            except serial.SerialException:
                self.log_signal.emit("Serial read error")

            if idx >= len(self.gcode_lines) and not in_flight:
                break

            time.sleep(0.001)

        # Final tool update at end
        self.tool_signal.emit(x, y, z)
        self.progress_signal.emit(100)
        self.log_signal.emit("Job complete")
        self.done_signal.emit()

    def _run_simulation(self):
        x = y = z = 0.0
        for i, line in enumerate(self.gcode_lines):
            if self._stop:
                self.log_signal.emit("Simulation stopped")
                break
            self.log_signal.emit(f">> {line} (simulated)")
            for axis, val in re.findall(r"([XYZ])([-+]?\d*\.?\d+)", line.upper()):
                if axis == "X": x = float(val)
                elif axis == "Y": y = float(val)
                elif axis == "Z": z = float(val)
            self.tool_signal.emit(x, y, z)
            # Emit progress for simulation
            progress = int((i / len(self.gcode_lines)) * 100)
            self.progress_signal.emit(progress)
            time.sleep(0.002)
        self.progress_signal.emit(100)
        self.done_signal.emit()


# -------------------- Main Window --------------------
class CNCSender(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laser X sender V1.0")
        self.resize(1100, 700)

        self.ser = None
        self.sender = None
        self.gcode_lines = []
        self.zero_offsets = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        self.last_wco = None
        self.last_mpos = None

        self.use_real_feedback = True

        self._build_ui()
        self.status_timer = QtCore.QTimer()
        self.status_timer.timeout.connect(self.poll_status)

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        left = QtWidgets.QVBoxLayout()
        layout.addLayout(left, 0)

        # Ports & Buttons
        self.port_box = QtWidgets.QComboBox()
        self.refresh_ports()
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.load_btn = QtWidgets.QPushButton("Load G-code")
        self.play_btn = QtWidgets.QPushButton("Play")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        left.addWidget(self.port_box)
        
        # Create horizontal layout for Connect/Disconnect
        conn_layout = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)

        left.addLayout(conn_layout)

        left.addWidget(self.load_btn)
        
        # Create horizontal layout for Play/Stop
        play_layout = QtWidgets.QHBoxLayout()
        play_layout.addWidget(self.play_btn)
        play_layout.addWidget(self.stop_btn)

        # Add the layout to the left panel instead of individual widgets
        left.addLayout(play_layout)

        
        # left.addSpacing(5)
        # left.addWidget(QtWidgets.QLabel("Progress Info"))

        # Run time and ETA labels
        self.run_time_label = QtWidgets.QLabel("Run Time: 00:00:00")
        self.eta_label = QtWidgets.QLabel("ETA: 00:00:00")
        left.addWidget(self.run_time_label)
        left.addWidget(self.eta_label)

        # --- Progress Bar ---
        # left.addSpacing(5)
        # left.addWidget(QtWidgets.QLabel("Progress"))
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        left.addWidget(self.progress_bar)

        # --- Zero Axis ---
        left.addSpacing(10)
        left.addWidget(QtWidgets.QLabel("Zero Axis"))
        zero = QtWidgets.QGridLayout()
        left.addLayout(zero)
        zero.addWidget(self._jog_btn("X0"), 0, 0)
        btn_home = QtWidgets.QPushButton("$Home")
        btn_home.clicked.connect(lambda: self.send_command("$H"))
        zero.addWidget(btn_home, 0, 1)
        zero.addWidget(self._jog_btn("Y0"), 0, 2)

        # --- Jog Buttons ---
        left.addSpacing(10)
        left.addWidget(QtWidgets.QLabel("Jog"))
        jog = QtWidgets.QGridLayout()
        left.addLayout(jog)
        jog.addWidget(self._jog_btn("Y+"), 1, 1)
        jog.addWidget(self._jog_btn("X-"), 2, 0)
        jog.addWidget(self._jog_btn("X+"), 2, 2)
        jog.addWidget(self._jog_btn("Y-"), 3, 1)

        left.addStretch()

        # --- Manual Command ---
        left.addSpacing(10)
        left.addWidget(QtWidgets.QLabel("Manual Command"))
        self.cmd_input = QtWidgets.QLineEdit()
        self.cmd_send_btn = QtWidgets.QPushButton("Send")
        manual_layout = QtWidgets.QHBoxLayout()
        manual_layout.addWidget(self.cmd_input, 1)
        manual_layout.addWidget(self.cmd_send_btn)
        left.addLayout(manual_layout)
        self.cmd_send_btn.clicked.connect(self.send_manual_command)
        self.cmd_input.returnPressed.connect(self.send_manual_command)

        # --- Log ---
        self.log_box = QtWidgets.QTextEdit()
        self.log_box.setReadOnly(True)
        left.addWidget(self.log_box, 1)

        # --- Plot ---
        self.plot = pg.PlotWidget(background="#111")
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        layout.addWidget(self.plot, 1)
        self.path_item = pg.PlotDataItem(pen=pg.mkPen("c", width=1))
        self.plot.addItem(self.path_item)
        self.tool_item = pg.ScatterPlotItem(size=10, brush=pg.mkBrush("r"))
        self.plot.addItem(self.tool_item)

        # Connect buttons
        self.connect_btn.clicked.connect(self.connect)
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.load_btn.clicked.connect(self.load_gcode)
        self.play_btn.clicked.connect(self.play)
        self.stop_btn.clicked.connect(self.stop)

    # ---------------- Jog / Zero / Commands ----------------
    def _jog_btn(self, label):
        btn = QtWidgets.QPushButton(label)
        if label in ("X0", "Y0", "Z0"):
            btn.clicked.connect(lambda _, l=label: self.zero_axis(l[0]))
        else:
            btn.clicked.connect(lambda _, l=label: self.jog(l))
        return btn

    def jog(self, direction):
        if not self.ser or (self.sender and self.sender.isRunning()):
            return
        jog_map = {"X+": "G91 G0 X1", "X-": "G91 G0 X-1", "Y+": "G91 G0 Y1", "Y-": "G91 G0 Y-1"}
        cmd = jog_map[direction] + "\nG90\n"
        self.ser.write(cmd.encode())
        self.log(f"Jog {direction}")

    def zero_axis(self, axis):
        if not self.ser:
            return
        axis = axis.upper()
        try:
            self.ser.write(f"G10 L20 P1 {axis}0\n".encode())
            time.sleep(0.02)
            self.ser.write(b"?")
            time.sleep(0.02)
            resp = self.ser.readline().decode(errors="ignore").strip()
            if resp.startswith("<"):
                self.parse_status(resp)
            self.log(f"Axis {axis} zeroed")
        except Exception as e:
            self.log(f"Error zeroing axis {axis}: {e}")

    def send_manual_command(self):
        if not self.ser:
            self.log("Not connected")
            return
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.log(f">> {cmd}")
            self.cmd_input.clear()
        except serial.SerialException as e:
            self.log(f"Error sending command: {e}")

    def send_command(self, cmd):
        if not self.ser:
            self.log("Not connected")
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.log(f">> {cmd}")
        except serial.SerialException as e:
            self.log(f"Error sending command: {e}")

    # ---------------- CNC Logic ----------------
    def refresh_ports(self):
        self.port_box.clear()
        for p in serial.tools.list_ports.comports():
            self.port_box.addItem(p.device)

    def connect(self):
        try:
            port = self.port_box.currentText()
            self.ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.log("Connected")
            QSound.play("Assets/connected.wav")
            self.status_timer.start(200)
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        except Exception as e:
            self.log(str(e))

    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.log("Disconnected")
        QSound.play("Assets/disconnected.wav")
        self.status_timer.stop()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def load_gcode(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load G-code", "", "G-code (*.nc *.gcode *.tap *.txt)")
        if not path:
            return
        with open(path) as f:
            self.gcode_lines = f.readlines()
        self.log(f"Loaded {len(self.gcode_lines)} lines")
        self.plot_gcode()

    def play(self):
        if not self.gcode_lines:
            return
        self.status_timer.stop()
        self.play_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.use_real_feedback = True
        self.sender = SenderThread(self.ser, self.gcode_lines)
        self.sender.log_signal.connect(self.log)
        self.sender.tool_signal.connect(self.update_tool_position)
        self.sender.progress_signal.connect(self.progress_bar.setValue)  # <-- connect progress
        self.sender.done_signal.connect(self.job_finished)
        self.sender.start()
        
        #========== Progress Timer ================
        self.job_start_time = time.time()
        self.progress_timer = QtCore.QTimer()
        self.progress_timer.timeout.connect(self.update_progress_info)
        self.progress_timer.start(200)  # update every 200ms


    def stop(self):
        if self.ser:
            try:
                self.ser.write(b"M5\n")
            except serial.SerialException as e:
                self.log(f"Stop error: {e}")
        if self.sender:
            self.sender.stop()
        self.log("Stopping...")

    # ---------------- Status ----------------
    def parse_status(self, status):
        m_mpos = re.search(r"MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)
        m_wco  = re.search(r"WCO:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)
        m_wpos = re.search(r"WPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)

        if self.use_real_feedback:
            if m_wpos:
                x = float(m_wpos.group(1))
                y = float(m_wpos.group(2))
                self.tool_item.setData([x], [y])
                return
            if m_wco:
                self.last_wco = (float(m_wco.group(1)), float(m_wco.group(2)))
            if m_mpos and self.last_wco:
                mx, my = float(m_mpos.group(1)), float(m_mpos.group(2))
                ox, oy = self.last_wco
                self.tool_item.setData([mx - ox], [my - oy])
                return
            if m_mpos and self.last_wco is None:
                x = float(m_mpos.group(1))
                y = float(m_mpos.group(2))
                self.tool_item.setData([x], [y])

    def poll_status(self):
        if not self.ser:
            return
        try:
            sent = False
            while not sent:
                try:
                    self.ser.write(b"?")
                    sent = True
                except serial.SerialTimeoutException:
                    time.sleep(0.005)
            resp = self.ser.readline().decode(errors="ignore").strip()
            if resp.startswith("<"):
                self.parse_status(resp)
        except serial.SerialException:
            self.log("Serial error during status poll")

    @QtCore.pyqtSlot(float, float, float)
    def update_tool_position(self, x, y, z):
        self.tool_item.setData([x], [y])

    @QtCore.pyqtSlot()
    def job_finished(self):
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        if self.ser:
            self.status_timer.start(200)
        self.progress_bar.setValue(0)  # reset progress
        self.progress_timer.stop()  # stop updating ETA/run time
        
        #=========== ETA Run time =======================
    def update_progress_info(self):
        if not self.sender or not self.gcode_lines:
            return
        elapsed = int(time.time() - self.job_start_time)
        progress = self.progress_bar.value()

        # Format run time
        h, rem = divmod(elapsed, 3600)
        m, s = divmod(rem, 60)
        self.run_time_label.setText(f"Run Time: {h:02}:{m:02}:{s:02}")

        # Estimate ETA
        if progress > 0:
            total_est = elapsed * 100 / progress
            eta = int(total_est - elapsed)
            h, rem = divmod(eta, 3600)
            m, s = divmod(rem, 60)
            self.eta_label.setText(f"ETA: {h:02}:{m:02}:{s:02}")
        else:
            self.eta_label.setText("ETA: calculating...")


    # ---------------- Plotting ----------------
    def plot_gcode(self):
        x = y = 0.0
        xs, ys = [0], [0]
        for line in self.gcode_lines:
            for axis, val in re.findall(r"([XY])([-+]?\d*\.?\d+)", line.upper()):
                if axis == "X": x = float(val)
                elif axis == "Y": y = float(val)
            xs.append(x)
            ys.append(y)
        self.path_item.setData(xs, ys)

    # ---------------- Utility ----------------
    def log(self, msg):
        self.log_box.append(msg)
        self.log_box.ensureCursorVisible()


# -------------------- Run --------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = CNCSender()
    w.show()
    sys.exit(app.exec_())
