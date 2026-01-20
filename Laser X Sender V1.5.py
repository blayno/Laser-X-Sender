import sys
import time
import serial
import serial.tools.list_ports
import re
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from PyQt5.QtMultimedia import QSound  # at the top of your file
import numpy as np
from PyQt5 import QtGui

# -------------------- CNC Sender Thread (Planner-aware GRBL) --------------------
class SenderThread(QtCore.QThread):
    log_signal = QtCore.pyqtSignal(str)
    tool_signal = QtCore.pyqtSignal(float, float, float)
    done_signal = QtCore.pyqtSignal()
    progress_signal = QtCore.pyqtSignal(int)  # <-- progress signal
    feed_signal = QtCore.pyqtSignal(float)  # NEW
    speed_signal = QtCore.pyqtSignal(float) # NEW
    status_signal = QtCore.pyqtSignal(float, float, float)
    QUIET_STATUS = True


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

                # ---- Parse positions ----
                for axis, val in re.findall(r"([XYZ])([-+]?\d*\.?\d+)", raw.upper()):
                    if axis == "X": x = float(val)
                    elif axis == "Y": y = float(val)
                    elif axis == "Z": z = float(val)

                # ---- Parse feed and speed once per line ----
                m_feed = re.search(r"F([0-9.]+)", raw.upper())
                if m_feed:
                    self.feed_signal.emit(float(m_feed.group(1)))

                m_speed = re.search(r"S([0-9.]+)", raw.upper())
                if m_speed:
                    self.speed_signal.emit(float(m_speed.group(1)))



                # ---- Send to GRBL ----
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

                # ---- Emit progress ----
                progress = int((idx / len(self.gcode_lines)) * 100)
                self.progress_signal.emit(progress)

            # ---- Read response from GRBL ----
            try:
                resp = self.ser.readline().decode(errors="ignore").strip()
                if not resp:
                    pass

                # ---- STATUS REPORT (silent) ----
                elif resp.startswith("<"):
                    m = re.search(
                        r"MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)",
                        resp
                    )
                    if m:
                        x_rt, y_rt, z_rt = map(float, m.groups())
                        self.status_signal.emit(x_rt, y_rt, z_rt)

                # ---- PLANNER RESPONSES (logged) ----
                elif resp.startswith("ok") or resp.startswith("error"):
                    self.log_signal.emit(f"<< {resp}")
                    if in_flight:
                        bytes_in_flight -= in_flight.pop(0)

                # ---- EVERYTHING ELSE (optional log) ----
                else:
                    self.log_signal.emit(f"<< {resp}")


            except serial.SerialException:
                self.log_signal.emit("Serial read error")
                
# ---- =============== Request realtime ?? status print in console(20 Hz)================ ----
            if time.time() - last_update > 0.05:
                try:
                    self.ser.write(b"?")
                except serial.SerialException:
                    pass
                last_update = time.time()



            if idx >= len(self.gcode_lines) and not in_flight:
                break

            time.sleep(0.001)

        # ---- Final update ----
        
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
        self.setWindowTitle("Laser X sender V1.5")
        self.resize(1100, 700)

        self.ser = None
        self.sender = None
        self.gcode_lines = []
        self.zero_offsets = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        self.last_wco = None
        self.last_mpos = None
        
        self.jog_buttons = []
        self.zero_buttons = []  # add this for zero buttons
        
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
        self.play_btn.setEnabled(False)
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
        zero_layout = QtWidgets.QGridLayout()
        left.addLayout(zero_layout)

        # Zero buttons
        zero_x = self._jog_btn("X0")
        zero_y = self._jog_btn("Y0")
        

        # Home button
        btn_home = QtWidgets.QPushButton("$Home")
        btn_home.clicked.connect(lambda: self.send_command("$H"))

        # Add to grid
        zero_layout.addWidget(zero_x, 0, 0)
        zero_layout.addWidget(btn_home, 0, 1)
        zero_layout.addWidget(zero_y, 0, 2)

        # Keep track of zero buttons for UI state management
        self.zero_buttons.extend([zero_x, zero_y, btn_home])

        # --- Jog Distance & Speed Sliders ---
        left.addSpacing(10)
        left.addWidget(QtWidgets.QLabel("Jog"))

        jog_settings_layout = QtWidgets.QGridLayout()
        left.addLayout(jog_settings_layout)

        # Distance slider
        self.jog_distance_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.jog_distance_slider.setMinimum(1)     # 0.1 mm
        self.jog_distance_slider.setMaximum(1000)  # 100.0 mm
        self.jog_distance_slider.setValue(10)     # default 10 mm
        self.jog_distance_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.jog_distance_slider.setTickInterval(50)
        self.jog_distance_label = QtWidgets.QLabel("Distance: 1.0 mm")

        self.jog_distance_slider.valueChanged.connect(
            lambda v: self.jog_distance_label.setText(f"Distance: {v / 10:.1f} mm")
        )

        # Speed slider
        self.jog_speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.jog_speed_slider.setMinimum(1)      # 1 mm/min
        self.jog_speed_slider.setMaximum(5000)   # 5000 mm/min
        self.jog_speed_slider.setValue(1000)      # default 500 mm/min
        self.jog_speed_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.jog_speed_slider.setTickInterval(500)
        self.jog_speed_label = QtWidgets.QLabel("Speed: 1000 mm/min")

        self.jog_speed_slider.valueChanged.connect(
            lambda v: self.jog_speed_label.setText(f"Speed: {v} mm/min")
        )

        # Add to grid layout
        jog_settings_layout.addWidget(self.jog_distance_label, 0, 0)
        jog_settings_layout.addWidget(self.jog_distance_slider, 0, 1)
        jog_settings_layout.addWidget(self.jog_speed_label, 1, 0)
        jog_settings_layout.addWidget(self.jog_speed_slider, 1, 1)

        # --- Jog Buttons ---
        # left.addSpacing(10)
        # left.addWidget(QtWidgets.QLabel("Jog"))
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
        self._build_visualiser_legend()

        self.cut_path = pg.PlotDataItem(
            pen=pg.mkPen("c", width=2)
        )
        self.rapid_path = pg.PlotDataItem(
            pen=pg.mkPen("y", width=1, style=QtCore.Qt.DashLine)
        )

        self.cut_path.setZValue(1)
        self.rapid_path.setZValue(2)   # rapids on top

        self.plot.addItem(self.cut_path)
        self.plot.addItem(self.rapid_path)

        self.tool_item = pg.ScatterPlotItem(
            size=12,
            brush=pg.mkBrush("r"),
            pen=pg.mkPen("w", width=1)  # white outline helps visibility
        )
        self.tool_item.setZValue(10)   # ALWAYS on top
        self.plot.addItem(self.tool_item)


        # Connect buttons
        self.connect_btn.clicked.connect(self.connect)
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.load_btn.clicked.connect(self.load_gcode)
        self.play_btn.clicked.connect(self.play)
        self.stop_btn.clicked.connect(self.stop)
        self.update_jog_zero_state()

    # ---------------- Jog / Zero / Commands ----------------
    def _jog_btn(self, label):
        btn = QtWidgets.QPushButton(label)
        if label in ("X0", "Y0", "Z0"):
            btn.clicked.connect(lambda _, l=label: self.zero_axis(l[0]))
        else:
            btn.clicked.connect(lambda _, l=label: self.jog(l))
            
            self.jog_buttons.append(btn)   # <-- track buttons
        return btn

    def jog(self, direction):
        if not self.ser or (self.sender and self.sender.isRunning()):
            return

        dist = self.jog_distance_slider.value() / 10  # 0.1 mm steps
        speed = self.jog_speed_slider.value()         # mm/min

        # Use G1 for feedrate-aware jog
        jog_map = {
            "X+": f"G91 G1 X{dist} F{speed}",
            "X-": f"G91 G1 X-{dist} F{speed}",
            "Y+": f"G91 G1 Y{dist} F{speed}",
            "Y-": f"G91 G1 Y-{dist} F{speed}"
        }
        cmd = jog_map[direction] + "\nG90\n"
        try:
            self.ser.write(cmd.encode())
            self.log(f"Jog {direction} Dist:{dist} Speed:{speed}")
            self.feed_label.setText(f"F:{speed:.0f}")  # <-- update feed label immediately
        except serial.SerialException as e:
            self.log(f"Jog error: {e}")


    def zero_axis(self, axis):
        if not self.ser:
            return
        axis = axis.upper()
        try:
            self.ser.write(f"G10 L20 P1 {axis}0\n".encode())
            self.log(f"Axis {axis} zeroed")
            # Immediately ask GRBL for status (non-blocking)
            self.ser.write(b"?")
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
            self.status_timer.start(50) #milliseconds
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.play_btn.setEnabled(True)   # <-- enable Play button
            self.update_jog_zero_state()
        except Exception as e:
            self.log(str(e))

    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.log("Disconnected")
        QSound.play("Assets/disconnected.wav")
        self.play_btn.setEnabled(False)   # <-- enable Play button
        self.status_timer.stop()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.update_jog_zero_state()

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
        
        
        self.play_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.use_real_feedback = True
        self.sender = SenderThread(self.ser, self.gcode_lines)
        self.sender.log_signal.connect(self.log)

        self.sender.progress_signal.connect(self.progress_bar.setValue)  # <-- connect progress
        self.sender.done_signal.connect(self.job_finished)
        self.sender.feed_signal.connect(lambda f: self.feed_label.setText(f"F:{f:.0f}"))
        self.sender.speed_signal.connect(lambda s: self.speed_label.setText(f"S:{s:.0f}"))
        self.sender.status_signal.connect(self.update_tool_position)
        self.sender.start()
        self.update_jog_zero_state()   # disable jog/zero
        
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
        self.update_jog_zero_state()

    # ---------------- Status ----------------
    def parse_status(self, status):
        # ---------------- Positions ----------------
        m_mpos = re.search(r"MPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)
        m_wpos = re.search(r"WPos:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)
        m_wco  = re.search(r"WCO:([-0-9.]+),([-0-9.]+),([-0-9.]+)", status)

        if m_wco:
            self.last_wco = [
                float(m_wco.group(1)),
                float(m_wco.group(2)),
                float(m_wco.group(3))
            ]

        x = y = z = 0.0

        if m_wpos:
            x, y, z = map(float, m_wpos.groups())
        elif m_mpos and self.last_wco:
            x = float(m_mpos.group(1)) - self.last_wco[0]
            y = float(m_mpos.group(2)) - self.last_wco[1]
            z = float(m_mpos.group(3)) - self.last_wco[2]
        elif m_mpos:
            x, y, z = map(float, m_mpos.groups())

        self.tool_item.setData([x], [y])
        self.dro_x.setText(f"X:{x:.3f}")
        self.dro_y.setText(f"Y:{y:.3f}")

        # ---------------- Machine state ----------------
        m_state = re.match(r"<([^|,]+)", status)
        state = m_state.group(1) if m_state else "Unknown"

        # ---------------- Feed / Speed ----------------
        # GRBL 1.1: FS:<feed>,<spindle>
        m_fs = re.search(r"FS:([0-9.]+),([0-9.]+)", status)
        if m_fs:
            feed  = float(m_fs.group(1))
            speed = float(m_fs.group(2))

            self.feed_label.setText(f"F:{feed:.0f}")
            self.speed_label.setText(f"S:{speed:.0f}")
            return  # IMPORTANT: prevent F:/S: false matches

        # GRBL 0.9 fallback (only if FS not present)
        m_feed = re.search(r"\bF:([0-9.]+)", status)
        if m_feed:
            self.feed_label.setText(f"F:{float(m_feed.group(1)):.0f}")

        m_speed = re.search(r"\bS:([0-9.]+)", status)
        if m_speed:
            self.speed_label.setText(f"S:{float(m_speed.group(1)):.0f}")


    def poll_status(self):
        """
        Continuously poll GRBL for status reports.
        Non-blocking, works smoothly during fast jogging (GRBL 1.1+).
        """
        if not self.ser or not self.ser.is_open:
            return
            
        # 🚫 Do not poll while streaming
        if self.sender and self.sender.isRunning():
            return

        try:
            # Send status request
            try:
                self.ser.write(b"?")
            except serial.SerialTimeoutException:
                return

            # Read all available lines without blocking
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                # Only parse status reports starting with '<'
                if line.startswith("<"):
                    self.parse_status(line)
        except serial.SerialException:
            self.log("Serial error during status poll")

    @QtCore.pyqtSlot(float, float, float)
    def update_tool_position(self, x, y, z):
        self.tool_item.setData([x], [y])
        self.dro_x.setText(f"X: {x:.3f}")
        self.dro_y.setText(f"Y: {y:.3f}")  

    @QtCore.pyqtSlot()
    def job_finished(self):
        self.play_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        if self.ser:
            self.status_timer.start(50) #milliseconds
        self.progress_bar.setValue(0)  # reset progress
        self.progress_timer.stop()  # stop updating ETA/run time
        
        self.sender = None            # <-- IMPORTANT
        self.update_jog_zero_state()  # <-- re-enable jogs here
        
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
        motion = None
        laser_power = 0.0

        cut_x, cut_y = [], []
        rapid_x, rapid_y = [], []

        for line in self.gcode_lines:
            line_u = line.upper()

            # ---- Motion mode (SAFE) ----
            if re.search(r"\bG0\b|\bG00\b", line_u):
                motion = "G0"
            elif re.search(r"\bG1\b|\bG01\b", line_u):
                motion = "G1"

            # ---- Laser power (modal) ----
            m = re.search(r"\bS([-+]?\d*\.?\d+)", line_u)
            if m:
                laser_power = float(m.group(1))

            # ---- Parse XY ----
            for axis, val in re.findall(r"([XY])([-+]?\d*\.?\d+)", line_u):
                if axis == "X":
                    x = float(val)
                elif axis == "Y":
                    y = float(val)

            # ---- Decide path ----
            laser_on = (motion == "G1") and (laser_power > 0)

            if laser_on:
                cut_x.append(x)
                cut_y.append(y)
                rapid_x.append(np.nan)
                rapid_y.append(np.nan)
            else:
                rapid_x.append(x)
                rapid_y.append(y)
                cut_x.append(np.nan)
                cut_y.append(np.nan)

        self.cut_path.setData(cut_x, cut_y)
        self.rapid_path.setData(rapid_x, rapid_y)
        
    #------- Legend helper -------------------------
    def _update_legend_pos(self):
        vb = self.plot.getViewBox()
        p = vb.mapToView(QtCore.QPointF(6, 6))
        self.legend_proxy.setPos(p.x(), p.y())
        
    #====== Feeds And Speeds Visualiser Legenf===============           
    def _build_visualiser_legend(self):
        self.legend_widget = QtWidgets.QFrame(self.plot)
        self.legend_widget.setStyleSheet("""
            QFrame {
                background-color: rgba(0, 0, 0, 50);
                color: white;
                border-radius: 4px;
            }
            QLabel {
                font-family: Consolas;
                font-size: 16pt;
            }
        """)

        layout = QtWidgets.QVBoxLayout(self.legend_widget)
        layout.setContentsMargins(4, 3, 4, 3)
        layout.setSpacing(1)

        self.dro_x = QtWidgets.QLabel("X:0.000")
        self.dro_y = QtWidgets.QLabel("Y:0.000")
        self.feed_label = QtWidgets.QLabel("F:0")
        self.speed_label = QtWidgets.QLabel("S:0")

        layout.addWidget(self.dro_x)
        layout.addWidget(self.dro_y)
        # layout.addSpacing(2)
        layout.addWidget(self.feed_label)
        layout.addWidget(self.speed_label)

        self.legend_widget.adjustSize()
        self.legend_widget.move(8, 8)
        self.legend_widget.show()
       
#single ui state manager--------------------
    def update_jog_zero_state(self):
        enabled = self.ser is not None and (self.sender is None or not self.sender.isRunning())

        # Jog buttons
        for btn in self.jog_buttons:
            btn.setEnabled(enabled)

        # Zero buttons
        for btn in self.zero_buttons:
            btn.setEnabled(enabled)

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
