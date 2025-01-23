import os
import subprocess
import sys
from threading import Thread

import cv2
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QAbstractAnimation
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel,
                             QVBoxLayout, QWidget, QMessageBox, QGraphicsOpacityEffect, QHBoxLayout, QMenuBar, QMenu,
                             QAction, QSizePolicy)


from camera_view import CameraView


class AutoWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Weed Control System')
        self.process = None

        self.showFullScreen()

        # Main layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(20)  # Space between widgets
        self.layout.setAlignment(Qt.AlignCenter)

        # Welcome label
        self.welcome_label = QLabel('Weed Control System')
        self.welcome_label.setFont(QFont('Arial', 50, QFont.Bold))
        self.welcome_label.setAlignment(Qt.AlignCenter)
        self.welcome_label.setStyleSheet("color: white; background-color: #3C3C3C;")

        # QLabel to display the video feed
        self.video_label = QLabel("Camera Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Layout setup
        self.layout.addWidget(self.welcome_label)
        self.layout.addWidget(self.video_label)

        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.layout)


        self.cap = cv2.VideoCapture(0)


        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Upd

        self.start_layout()

        # Add Menu Bar
        self.create_menu_bars()

        # Styling
        self.setStyleSheet("""
                       QWidget {
                     background-color: #5e5e5e;
                 }

                 QLabel {
                     color: white;
                     padding: 30px;
                     font-weight: bold;           /* Bold text */
                     padding: 10px;               /* Padding around the text */
                     border-radius: 10px;         /* Rounded corners for the background */
                 }

                 QPushButton {
                     background-color: #3E3E3E; /* Dark gray background */
                     color: #FFFFFF; /* White text */
                     font-size: 18px;
                     font-weight: bold;
                     border: 2px solid #5A5A5A; /* Subtle border */
                     border-radius: 10px; /* Rounded corners */
                     padding: 10px 20px; /* Spacing inside the button */
                     transition: all 0.3s ease; /* Smooth hover effect */
                 }

                 QPushButton:hover {
                     background-color: #5A5A5A; /* Slightly lighter on hover */
                     border-color: #FFFFFF; /* Highlight border on hover */
                     transform: scale(1.05); /* Slight zoom effect */
                 }

                 QPushButton:pressed {
                     background-color: #2B2B2B; /* Darker on press */
                     border-color: #8A8A8A; /* Dimmed border */
                     transform: scale(0.95); /* Slight shrink effect */
                 }

                 QMenuBar {
                     background-color: #3C3C3C; /* Dark gray for the menubar */
                     color: #FFFFFF; /* White text for visibility */
                     border: 1px solid #2D2D2D; /* Subtle border for contrast */
                     font-size: 70px; /* Increase text size */
                 }

                 QMenuBar::item {
                     background-color: transparent; /* Transparent by default */
                     color: #FFFFFF; /* White text for visibility */
                     padding: 5px 10px; /* Spacing around menu items */
                     margin: 2px; /* Small margins for items */
                     font-size: 30px; /* Increase text size */
                 }

                 QMenuBar::item:selected {
                     background-color: #5A5A5A; /* Highlight color when hovered */
                     color: #E0E0E0; /* Slightly lighter text when hovered */
                 }


                 QMenuBar::item:pressed {
                     background-color: #2D2D2D; /* Darker color for pressed items */
                 }

                 /* Ensure actions in menus have white text */
                 QMenu::item {
                    background-color: #3E3E3E; /* Dark gray background */
                    color: #FFFFFF; /* White text for actions */
                    padding: 5px 10px; /* Spacing around menu items */
                    margin: 2px; /* Small margins for items */
                    font-size: 15px; /* Increase text size */
                 }

                 QMenu::item:selected {
                    background-color: #5A5A5A; /* Highlight color when hovered */
                    color: #E0E0E0; /* Lighter text when hovered */
                 }

                 QMenu::item:pressed {
                    background-color: #2D2D2D; /* Darker color for pressed items */
                 }
             """)

    def create_menu_bars(self):
        # Create the menu bar
        menubar = self.menuBar()

        # Create Menus
        home = menubar.addMenu('≡')
        launch = menubar.addMenu('▶')

        menu_action = QAction('Main Menu', self)
        menu_action.triggered.connect(self.open_Main_Menu)

        launch_action = QAction('Start Autonomous Mode', self)
        launch_action.triggered.connect(self.launch_files)

        homing_action = QAction('Homing', self)
        homing_action.triggered.connect(self.homing)

        idle_action = QAction('Idle', self)
        idle_action.triggered.connect(self.idle)


        camera_action = QAction('Camera View', self)
        camera_action.triggered.connect(self.open_camera_view)

        exit = QAction('Exit', self)
        exit.triggered.connect(self.close_application)

        # Add actions to menus
        home.addAction(menu_action)
        home.addAction(camera_action)
        home.addAction(homing_action)
        home.addAction(idle_action)
        home.addAction(exit)
        launch.addAction(launch_action)

    def start_layout(self):
        self.welcome_label.setText('Weed Control System')

        # Create a layout for the welcome label
        welcome_layout = QVBoxLayout()
        welcome_layout.addWidget(self.welcome_label, alignment=Qt.AlignTop | Qt.AlignHCenter)

        # Add the welcome layout to the main layout
        self.layout.addLayout(welcome_layout)

        # Add a spacer to push the mode buttons down
        self.layout.addStretch()

        self.layout.addWidget(self.video_label)
        self.layout.addStretch()

        button_layout = QVBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        self.layout.addStretch()

    def open_Main_Menu(self):
        from main_window import MainWindow
        self.window = MainWindow()
        self.window.show()
        self.close()

    def homing(self):

        try:
            # Change to the ROS 2 workspace directory
            homimg_dir = ''
            os.chdir(homimg_dir)

            build_process = subprocess.run(['python3', 'homing_python.py'], capture_output=True, text=True)
            QMessageBox.information(self, 'Homing', 'Homing calibration complete')

            if build_process.returncode != 0:
                QMessageBox.critical(self, 'Build Error', f'Build failed:\n{build_process.stderr}')
                return

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

    def idle(self):

        try:
            # Change to the ROS 2 workspace directory
            idle_dir = ''
            os.chdir(idle_dir)

            build_process = subprocess.run(['python3', 'set_all_idle.py'], capture_output=True, text=True)
            QMessageBox.information(self, 'Idle', 'Motor are now idle')

            if build_process.returncode != 0:
                QMessageBox.critical(self, 'Build Error', f'Build failed:\n{build_process.stderr}')
                return

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

    def open_camera_view(self):

        self.cameraView = CameraView()
        self.cameraView.show()
        self.close()

    def launch_files(self):
        try:
            workspace = ''
            # Change to the ROS 2 workspace directory
            os.chdir(workspace)

            def run_launch():
                try:

                    launch_command = ['ros2', 'launch', 'cartesian_positioner_controller', 'all_control.launch.py',
                                      'setup_type:=mobile_weeder', 'manual:=false']

                    self.process = subprocess.Popen(
                        launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
                    )

                    for line in self.process.stdout:
                        print(line, end="")

                except Exception as e:
                    QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

            # Run the launch process in a separate thread
            launch_thread = Thread(target=run_launch, daemon=True)
            launch_thread.start()

            QMessageBox.information(self, 'Launch Started', 'The ROS 2 launch process has started.')

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:

            height, width, _ = frame.shape

            # Set higher resolution (optional)
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


            left_frame = frame[:, :width // 2]  # Left half of the frame


            left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)


            left_frame = cv2.resize(left_frame, (width * 1, height * 2), interpolation=cv2.INTER_CUBIC)


            h, w, ch = left_frame.shape
            bytes_per_line = ch * w
            qimg = QImage(left_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            pixmap = QPixmap.fromImage(qimg)
            self.video_label.setPixmap(pixmap)

    def close_application(self):
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    autoWindow = AutoWindow()
    autoWindow.show()
    sys.exit(app.exec_())
