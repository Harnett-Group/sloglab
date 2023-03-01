# The MIT License (MIT)
#
# Copyright (c) 2023 Paul Bupe Jr
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
* Author(s): Paul Bupe Jr
"""

import csv
import glob
import sys
import time

import numpy as np
import pyqtgraph as pg
import serial
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets


class SerialCaptureThread(QtCore.QThread):
    sensor_data_signal = QtCore.Signal(np.ndarray)

    def __init__(self, *args, **kwargs):
        super(SerialCaptureThread, self).__init__(*args, **kwargs)
        self.reading = None
        self.serial_device = None
        self.readings = np.empty((0, 3), int)

    def run(self):
        # close port if open
        if self.serial_device:
            self.serial_device.close()
        self.serial_device.open()
        while True:
            try:
                line = self.serial_device.readline().decode("utf-8").rstrip()
                if line:
                    msg = self.parser(line)
                    if msg:
                        self.reading = msg
                        # self.readings = np.vstack((self.readings, self.reading))

                        # send signal to update plots
                        self.sensor_data_signal.emit(self.reading)
            except:
                pass

    def set_serial_device(self, serial_device):
        self.serial_device = serial_device

    def get_readings(self):
        result = self.readings
        return result

    def get_single_reading(self):
        return self.reading

    def parser(self, line):
        packet_list = str(line).split(",")

        if len(packet_list) < 1:
            return None
        try:
            packet_list = [int(x) for x in packet_list]
        except:
            return None

        return packet_list

    def clear_readings(self):
        self.readings = np.empty((0, 3), int)

    def stop(self):
        if self.serial_device:
            self.serial_device.close()
        # self.serial_device.close()
        self.terminate()


class SerialHelper:
    def __init__(self, baudrate=115200, timeout=0.1):
        super(SerialHelper, self).__init__()
        self.port = None
        self.serial = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.baudrates = [
            "Select Baudrate...",
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800",
            "921600",
        ]
        # self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.serial_ports = ["Select Port..."]
        self.get_serial_ports()
        self.serial_port_count = len(self.serial_ports)

    def serial_connect(self):
        """Connect to a serial port."""
        # try connecting to serial port
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except Exception as e:
            self.serial = None
            pass

    def serial_disconnect(self):
        """Disconnect from a serial port."""
        self.serial.close()

    def get_serial_ports(self):
        """Lists serial port names"""

        result = ["Select Port..."]

        if sys.platform.startswith("win"):
            ports = ["COM%s" % (i + 1) for i in range(256)]
        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            ports = glob.glob("/dev/tty[A-Za-z]*")
        elif sys.platform.startswith("darwin"):
            ports = glob.glob("/dev/tty.*")

        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass

        self.serial_ports = result


class SLogLabGui(QtWidgets.QMainWindow, SerialHelper):
    def __init__(self, plot_length=1000, sample=False, *args, **kwargs):
        """
        Initialize the GUI
        :param signal_count:
        :param plot_length:
        :param sample:
        :param args:
        :param kwargs:
        """
        super(SLogLabGui, self).__init__(*args, **kwargs)
        SerialHelper.__init__(self, *args, **kwargs)

        self.plot_layout = None
        self.toolbar = None
        self.main_layout = None
        self.capture_thread = SerialCaptureThread()
        self.capturing = False
        self.port = None
        self.signal_count = 0
        self.plot_length = plot_length
        self.data = None

        self.plot_colors = ["#8dd3c7", "#feffb3", "#bfbbd9", "#fa8174",
                            "#81b1d2", "#fdb462", "#b3de69", "#bc82bd", "#ccebc4", "#ffed6f"]
        self.range = (0, 1)
        self.labels = []

        self.configure_ui()

        self.timer = QtCore.QTimer()

        self.start_time = time.time()
        self.timer.timeout.connect(self.update)
        self.show()
        self.timer.start(30)

    def configure_ui(self):
        """
        Configure the UI
        :return:
        """
        self.main_layout = QtWidgets.QGridLayout()
        self.setCentralWidget(QtWidgets.QWidget(self))
        self.centralWidget().setLayout(self.main_layout)
        self.setWindowTitle("SLogLab: Minimal Serial Logger and Labeler")
        # Change font
        font = QtGui.QFont()
        font.setFamily("Helvetica")
        self.setFont(font)

        self.setStyleSheet(
            """
        QMainWindow {
        background-color: #353535;
        font-family: Helvetica;
        color: rgb(255, 255, 255);}
        QPushButton {
            background-color: #212121; 
            color: #c8c8c8;
            border-radius: 4px;
            border: 2px;
            border-style: solid;
            border-color: #000;
            padding: 5px;}
        QPushButton:pressed {
            background-color: #333333;}
        QPushButton:disabled {
            background-color: #646464;
            border: 0px;}
            }
    
        QToolBar {
            border: 0px;
            padding: 5px;
            spacing: 5px;}
        QToolBar::separator {
            background-color: #353535;
            }
        QComboBox {
            background-color: #2a2a2a;
            color: #c8c8c8;
            selection-background-color: rgb(200, 200, 200);
            border: 1px;
            border-style: solid;
            border-color: #212121;
            border-radius: 2px;
            padding: 5px;}
        QComboBox QAbstractItemView {
            background: #2a2a2a;
            color: #c8c8c8;
            selection-background-color: #333333;
            padding: 5px;}
              """
        )
        self.resize(1300, 600)
        pg.setConfigOptions(antialias=True)

        self.build_toolbar()
        self.build_plots()
        self.build_buttons()

    def build_buttons(self):
        self.buttons_layout = QtWidgets.QVBoxLayout()
        self.buttons_layout.setAlignment(QtCore.Qt.AlignTop)
        # self.buttons_layout.setSpacing(0)
        # self.buttons_layout.setContentsMargins(0, 0, 0, 0)

        self.labels_list = QtWidgets.QListWidget()

        self.labels_list.setSpacing(0)
        # self.labels_list.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.labels_list.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.labels_list.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        # remove focus from list widget
        # self.labels_list.setFocusPolicy(QtCore.Qt.NoFocus)
        # Enable multi-selection
        self.labels_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.labels_list.setMaximumWidth(200)
        self.labels_list.setMaximumHeight(600)

        self.labels_list.setStyleSheet(
            "QListWidget"
            "{"
            "background-color: #2a2a2a; "
            "border: 0px;"
            "text-align: left;"
            "}"
            "QListWidget::Item"
            "{"
            "margin: 2px;"
            "text-align: left;"
            "font-weight: bold;"
            "border-bottom: 1px solid #353535;"
            "padding: 2px;"
            "}"
            "QListWidget::Item:selected"
            "{"
            # "background-color: rgb(197, 198, 199);"
            "background-color: #646464;"
            "}"
        )

        # attach callback to button
        self.labels_list.itemClicked.connect(self.label_clicked)
        # Add to layout

        # Add heading to layout
        self.heading_label = QtWidgets.QLabel("Enter Label:")
        self.heading_label.setStyleSheet(
            "color: rgb(133, 133, 133); text-align: center;"
        )
        self.heading_label.setAlignment(QtCore.Qt.AlignHCenter)
        self.heading_label.setFont(QtGui.QFont("Arial", pointSize=12))

        # Create label number box
        self.label_number = QtWidgets.QSpinBox()
        self.label_number.setRange(0, 999)
        self.label_number.setValue(0)
        self.label_number.setSingleStep(1)
        # self.label_number.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.label_number.setStyleSheet(
            """
            QSpinBox { 
                background-color: #2a2a2a; 
                border: 1px;
                border-style: solid;
                border-radius: 2px; 
                color: rgb(197, 198, 199);
                border-color: #212121;
                padding: 2px;}
            """
        )
        self.label_number.setAlignment(QtCore.Qt.AlignHCenter)
        # self.label_number.setFont(QtGui.QFont("Arial", pointSize=12))

        self.buttons_layout.addWidget(self.heading_label)
        self.buttons_layout.addWidget(self.label_number)

        # add button
        self.create_label_button = QtWidgets.QPushButton("Create Label")
        self.create_label_button.clicked.connect(self.create_label)
        self.buttons_layout.addWidget(self.create_label_button)

        self.buttons_layout.addWidget(self.labels_list)

        self.export_labels_button = QtWidgets.QPushButton("Export Labeled Data to CSV")
        self.export_labels_button.clicked.connect(self.export_labels_to_csv)
        self.buttons_layout.addWidget(self.export_labels_button)



        # clear button
        self.clear_button = QtWidgets.QPushButton("Clear Selected Labels")
        self.clear_button.clicked.connect(self.clear_selected_labels)
        self.buttons_layout.addWidget(self.clear_button)

        self.main_layout.addLayout(self.buttons_layout, 0, 1, QtCore.Qt.AlignTop)

    def build_plots(self):

        self.plot_layout = QtWidgets.QHBoxLayout()
        self.plot_layout.setSpacing(2)
        # background color
        # self.plot_layout.setContentsMargins(10, 10, 10, 10)


        p1 = pg.PlotWidget(name="Data Labeler")

        p1.setLabel("bottom", "Sample")
        p1.setLabel("left", "Amplitude")
        p1.setBackground((21, 21, 21))
        # Change axis width
        # add top and right axis
        p1.showAxis("top")
        p1.showAxis("right")
        p1.getAxis("top").setHeight(10)
        p1.getAxis("right").setWidth(10)

        # remove values from top and right axis
        p1.getAxis("top").setStyle(showValues=False)
        p1.getAxis("right").setStyle(showValues=False)


        # p1.setXRange(0, self.plot_length)
        # p1.setYRange(0, 1024)
        # p1.showGrid(x=True, y=True, alpha=0.5)
        p1.addLegend()
        # lock plot
        p1.setMouseEnabled(False, False)

        self.signal_plots = [
            # p1.plot(np.zeros(1), pen=(255, 0, 0), name="Signal 1"),
            # p1.plot(np.zeros(1), pen=(0, 255, 0), name="Signal 2"),
            # p1.plot(np.zeros(1), pen=(255, 255, 255), name="Signal 3"),
        ]

        self.region = pg.LinearRegionItem()
        self.region.setZValue(10)
        self.region.setRegion([0, 0])
        self.region.sigRegionChangeFinished.connect(self.region_updated)
        p1.addItem(self.region)

        self.plot_layout.addWidget(p1)
        self.main_layout.addLayout(self.plot_layout, 0, 0)

    def build_toolbar(self):
        self.toolbar = self.addToolBar("Toolbar")
        self.toolbar.setMovable(False)
        self.toolbar.setFloatable(False)

        # create dropdown widget
        self.dropdown = QtWidgets.QComboBox()
        self.dropdown.addItems(self.serial_ports)
        self.dropdown.setCurrentIndex(0)
        self.dropdown.currentIndexChanged.connect(self.port_changed)
        self.toolbar.addWidget(self.dropdown)

        # create baudrate dropdown widget
        self.baudrate_dropdown = QtWidgets.QComboBox()
        self.baudrate_dropdown.addItems(self.baudrates)
        self.baudrate_dropdown.setCurrentIndex(5)
        self.baudrate_dropdown.currentIndexChanged.connect(self.baudrate_changed)
        self.toolbar.addWidget(self.baudrate_dropdown)

        # add start and stop capture buttons
        self.start_button = QtWidgets.QPushButton("Start Capture")
        self.start_button.clicked.connect(self.start_capture)
        self.toolbar.addWidget(self.start_button)

        self.stop_button = QtWidgets.QPushButton("Stop Capture")
        self.stop_button.clicked.connect(self.stop_capture)
        self.stop_button.setEnabled(False)
        self.toolbar.addWidget(self.stop_button)

        # Add small divider
        self.toolbar.addSeparator()

        # Export to CSV button
        self.export_button = QtWidgets.QPushButton("Save Full Capture to CSV")
        self.export_button.clicked.connect(self.export_to_csv)
        self.toolbar.addWidget(self.export_button)

        # Add status label
        self.status_label = QtWidgets.QLabel(
            "INFO: Select a port and baudrate to begin capturing data."
        )
        self.status_label.setStyleSheet("color: white; margin-left: 10px;")
        self.toolbar.addWidget(self.status_label)

        # create spacer widget
        spacer = QtWidgets.QWidget()
        spacer.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        self.toolbar.addWidget(spacer)

        # Clear captured data
        self.clear_capture_button = QtWidgets.QPushButton("Clear Captured Data")
        self.clear_capture_button.clicked.connect(self.clear_captured_data)
        self.toolbar.addWidget(self.clear_capture_button)

    def init_signal_plots(self, num_signals):
        for i in range(num_signals):
            self.signal_plots.append(
                self.plot_layout.itemAt(0).widget().plot(
                    np.zeros(1), pen=self.plot_colors[i], name=f"Signal {(i+1)}"
                )
            )

    def baudrate_changed(self):
        self.baudrate = self.baudrates[self.baudrate_dropdown.currentIndex()]

    def port_changed(self, index):
        if index > 0:
            self.port = self.serial_ports[index]
            # print("Port changed to: " + self.serial_ports[index])
        else:
            self.port = None
            # print("Port changed to: None")
        # self.port = self.serial_ports[index]
        # self.serial_port.close()
        # self.serial_port = serial.Serial(self.port, 115200)
        # self.serial_port.flushInput()

    def start_capture(self):

        self.serial_connect()
        if self.port is None:
            self.status_label.setText(
                "ERROR: Connection Failed. Please select a valid port and baudrate"
            )
            return

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("INFO: Capturing data...")

        self.capture_thread.set_serial_device(self.serial)
        self.sensor_data_signal = self.capture_thread.sensor_data_signal
        self.sensor_data_signal.connect(self.update_sensor_data)
        self.capture_thread.start()
        self.capturing = True
        # self.capture_thread.sig_capture_finished.connect(self.capture_finished)
        # self.capture_thread.sig_capture_data.connect(self.capture_data)
        # Verify port and baudrate

    def stop_capture(self):
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("INFO: Capture stopped.")

        self.capture_thread.stop()
        self.serial_disconnect()
        del self.sensor_data_signal

        self.capturing = False

    def clear_captured_data(self):
        for i in range(self.signal_count):
            self.signal_plots[i].clear()

        self.region.setRegion([0, 0])
        self.data = None
        self.clear_selected_labels()
        self.capture_thread.clear_readings()

    def clear_selected_labels(self):

        to_remove = []
        # Clear selected labels
        for i in range(self.labels_list.count()):
            item = self.labels_list.item(i)

            if item.isSelected():
                # Get the index of the item
                index = self.labels_list.row(item)
                to_remove.append(index)

        # Remove the items in reverse order
        for i in reversed(to_remove):
            del self.labels[i]

        self.update_label_list()

        # update status label
        self.status_label.setText("INFO: Labels cleared.")

    def export_to_csv(self):
        if self.data is None:
            self.status_label.setText("ERROR: No data to export")
            return

        # get file name
        file_name = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Data", "", "CSV Files (*.csv)"
        )
        if not file_name[0]:
            # update status label
            self.status_label.setText("ERROR: No file selected")
            return

        # write data to file
        self.save_to_file(file_name[0], self.data)

    def export_labels_to_csv(self):
        # Check is there is data to export
        if self.data is None:
            self.status_label.setText("ERROR: No data to export")
            return

        if len(self.labels) == 0:
            self.status_label.setText("ERROR: No labels to export")
            return

        # get file name
        file_name = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Data", "", "CSV Files (*.csv)"
        )

        if not file_name[0]:
            # update status label
            self.status_label.setText("INFO: No file selected")
            return

        # select range from data
        data = []
        for i in range(len(self.labels)):
            lrange = self.labels[i][1]
            label = self.labels[i][0]
            if lrange:
                tmp_range = self.data[lrange[0]: lrange[1]]
                length = tmp_range.shape[0]
                # create label array
                labels = np.ones((length, 1), int) * label
                combined = np.hstack((tmp_range, labels))
                data.extend(combined)

        # write data to file
        self.save_to_file(file_name[0], data)

    def save_to_file(self, filename, data):
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(data)
        self.status_label.setText("INFO: Data exported to %s" % filename)

    def create_label(self):
        # Get value from label_number widget
        label_number = self.label_number.value()
        lrange = (self.range[0], self.range[1])
        # add label to list
        self.labels.append((label_number, lrange))
        # print(self.labels)
        self.update_label_list()
        self.status_label.setText("INFO: Label created.")

    def update_label_list(self):
        # clear list
        self.labels_list.clear()

        for label in self.labels:
            text = QtWidgets.QListWidgetItem("Label: %d, Range: [%d, %d]" % (label[0], label[1][0], label[1][1]))
            # text.setSizeHint(QtCore.QSize(200, 25))
            # text.setBackground(QtGui.QColor(33, 33, 33))
            text.setForeground(QtGui.QColor(197, 198, 199))
            # text.setTextAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
            text.setFont(QtGui.QFont("Arial", pointSize=10))
            # text.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
            self.labels_list.addItem(text)


    def region_updated(self):
        self.region.setZValue(10)
        x1, x2 = self.region.getRegion()
        self.range = (int(x1), int(x2))
        # print(self.range)

    def label_clicked(self, widgetitem):
        # qlistwidgetitem callback
        index = self.labels_list.row(widgetitem)

        if widgetitem.isSelected():
            lrange = self.labels[index][1]
            self.region.setRegion(lrange)

    def update_sensor_data(self, data):

        signal_count = len(data)
        if self.signal_count != signal_count:
            self.signal_count = signal_count
            self.init_signal_plots(signal_count)

        if self.data is None:
            self.data = np.array(data)

        self.data = np.vstack((self.data, data))

    def update(self):

        if not self.capturing:
            # calculate elapsed time
            elapsed_time = time.time() - self.start_time
            # check if one second has passed then check serial ports
            if elapsed_time > 1:
                self.start_time = time.time()
                self.get_serial_ports()
                # print(self.serial_ports, self.serial_port_count)
                # check if number of serial ports has changed
                if len(self.serial_ports) != self.serial_port_count:
                    self.dropdown.currentIndexChanged.disconnect(self.port_changed)
                    self.dropdown.clear()
                    self.dropdown.addItems(self.serial_ports)
                    self.dropdown.currentIndexChanged.connect(self.port_changed)
                    self.serial_port_count = len(self.serial_ports)

        if self.capturing and self.data is not None:
            for i in range(self.signal_count):
                self.signal_plots[i].setData(
                    np.arange(len(self.data[:, i])),
                    self.data[:, i],
                )


if __name__ == "__main__":
    # mkQApp("GapSensor")
    app = QtWidgets.QApplication(sys.argv)
    vl1 = SLogLabGui()
    # vl1 = GapSensorGUINew()
    sys.exit(app.exec_())
    # pg.exec()
