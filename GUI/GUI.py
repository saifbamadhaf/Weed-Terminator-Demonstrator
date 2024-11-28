import sys
import cv2
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QAbstractAnimation
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel,
                             QVBoxLayout, QWidget, QMessageBox, QGraphicsOpacityEffect, QHBoxLayout, QMenuBar, QMenu,
                             QAction, QToolTip)
import os
import subprocess

from AutoWindow import AutoWindow
from manualWindow import ManualWindow
from test import VideoPlayer
from camera_view import CameraView

# Define the workspace directory
workspace_dir = '/home/saifbamadhaf/ros2_ws'



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Weed Control System')


        # Assuming you are using a QMainWindow or QWidget
        self.showFullScreen()  # This is the correct place to call showMaximized()

        QToolTip.setFont(QFont('Arial', 12))


        # Main layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(20)  # Space between widgets
        self.layout.setAlignment(Qt.AlignCenter)

        # Welcome label
        self.welcome_label = QLabel('Weed Control System')
        self.welcome_label.setFont(QFont('Arial', 50, QFont.Bold))
        self.welcome_label.setAlignment(Qt.AlignCenter)
        self.welcome_label.setStyleSheet("color: white; background-color: #3C3C3C;")

        # Choose mode
        self.select_label = QLabel('Select your preferred mode to start:')
        self.select_label.setFont(QFont('Arial', 20))
        self.select_label.setAlignment(Qt.AlignCenter)
        self.select_label.setStyleSheet("color: white;")
        self.select_label.setVisible(False)

        # Mode buttons
        self.auto_mode_button = QPushButton('Autonomous Mode')

        self.auto_mode_button.clicked.connect(self.start_auto_mode)

        self.auto_mode_button.setFixedSize(500, 100)
        self.auto_mode_button.setToolTip('yooooo')  # Ensure tooltip is set
        self.auto_mode_button.setEnabled(True)  # Ensure button is enabled

        self.manual_mode_button = QPushButton('Manual Mode 🎮')

        self.manual_mode_button.clicked.connect(self.start_manual_mode)
        self.manual_mode_button.setEnabled(False)  # Disabled until system check is done
        self.manual_mode_button.setFixedSize(500, 100)

        # Help buttons
        self.auto_help_button = QPushButton('ⓘ')
        self.auto_help_button.setFixedSize(50, 50)
        self.auto_help_button.setStyleSheet("background-color: black; font-size: 40px;")
        self.auto_help_button.clicked.connect(self.show_auto_help)

        self.manual_help_button = QPushButton('ⓘ')
        self.manual_help_button.setFixedSize(50, 50)
        self.manual_help_button.setStyleSheet("background-color: black; font-size: 40px;")
        self.manual_help_button.clicked.connect(self.show_manual_help)

        # Learn More button
        self.learn_more_button = QPushButton('Play Video ▶')
        self.learn_more_button.setFont(QFont('Arial', 16))
        self.learn_more_button.setStyleSheet("background-color: blue; font-size: 20px;")
        self.learn_more_button.setFixedSize(200, 50)
        self.learn_more_button.clicked.connect(self.show_general_help)





        self.layout.addWidget(self.welcome_label)
        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.layout)

        # Start the GUI flow
        self.show_options()

        # Add Menu Bar
        self.create_menu_bar()

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
               font-size: 30px; /* Increase text size */
            }

            QMenu::item:selected {
               background-color: #5A5A5A; /* Highlight color when hovered */
               color: #E0E0E0; /* Lighter text when hovered */
            }

            QMenu::item:pressed {
               background-color: #2D2D2D; /* Darker color for pressed items */
            }
            QToolTip {
              background-color: white;  /* Light grey background */
              color: black;              /* Dark text color */
              border: 1px solid #aaaaaa;   /* Light border */
              padding: 5px;                /* Padding around the text */
              font-size: 14px;             /* Font size of the tooltip */
              font-family: Arial, sans-serif; /* Font family */
              border-radius: 4px;          /* Rounded corners */
    }
            
        """)

    def create_menu_bar(self):
        # Create the menu bar
        menubar = self.menuBar()

        # Create Menus
        home = menubar.addMenu('≡')

        menu_action = QAction('Main Menu', self)
        #menu_action.triggered.connect(self.show_homing)

        # Create 'Homing' action and connect to function
        homing_action = QAction('Homing', self)
        homing_action.triggered.connect(self.show_homing)

        # Create 'Camera View' action and connect to function
        camera_action = QAction('Camera View', self)
        camera_action.triggered.connect(self.show_camera_view)  # Correct way to connect

        idle_action = QAction('Idle', self)
        idle_action.triggered.connect(self.show_idle)

        exit = QAction('Exit', self)
        exit.triggered.connect(self.close_application)  # Correct way to connect

        # Add actions to menus
        home.addAction(menu_action)
        home.addAction(camera_action)
        home.addAction(homing_action)
        home.addAction(idle_action)
        home.addAction(exit)


    def perform_system_check(self):

        try:
            # Change to the ROS 2 workspace directory
            os.chdir(workspace_dir)

            # Run colcon build
            build_process = subprocess.run(['colcon', 'build'], capture_output=True, text=True)

            if build_process.returncode != 0:
                QMessageBox.critical(self, 'Build Error', f'Build failed:\n{build_process.stderr}')
                return

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

    def show_options(self):


        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(True)

        # Create a layout for the welcome label
        welcome_layout = QVBoxLayout()
        welcome_layout.addWidget(self.welcome_label, alignment=Qt.AlignTop | Qt.AlignHCenter)

        # Add the welcome layout to the main layout
        self.layout.addLayout(welcome_layout)

        # Add a spacer to push the mode buttons down
        self.layout.addStretch()

        button_layout = QVBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        # Show select label
        self.select_label.setVisible(True)  # Make the label visible
        self.layout.addWidget(self.select_label)

        # Add automatic mode button with help
        auto_layout = QHBoxLayout()
        auto_layout.addWidget(self.auto_mode_button)
        auto_layout.addWidget(self.auto_help_button)
        button_layout.addLayout(auto_layout)

        # Add manual mode button with help
        manual_layout = QHBoxLayout()
        manual_layout.addWidget(self.manual_mode_button)
        manual_layout.addWidget(self.manual_help_button)
        button_layout.addLayout(manual_layout)

        # Add the button layout to the main layout
        self.layout.addLayout(button_layout)

        self.layout.addStretch()

        # Create a layout for the exit and help buttons
        exit_layout = QHBoxLayout()
        exit_layout.addStretch()
        exit_layout.addWidget(self.learn_more_button)


        self.layout.addLayout(exit_layout)



    def start_auto_mode(self):
        self.autoWindow = AutoWindow()  # Instantiate VideoPlayer
        self.autoWindow.show()  # Show the player window
        self.close()

    def start_manual_mode(self):
        self.manualWindow = ManualWindow()  # Instantiate VideoPlayer
        self.manualWindow.show()  # Show the player window
        self.close()

    def show_auto_help(self):
        QMessageBox.information(self, 'Automatic Mode Help',
                                'Automatic Mode continuously terminates weeds in the selected area without further user input.')

    def show_manual_help(self):
        QMessageBox.information(self, 'Manual Mode Help',
                                'Manual Control Mode allows you to control the system manually for targeted weed termination.')


    def show_camera_view(self):
        # Placeholder function for camera view
        self.cameraView = CameraView()  # Instantiate VideoPlayer
        self.cameraView.show()  # Show the player window
        self.close()


    def show_calibrate(self):
        # Placeholder function for calibration menu
        homing_action = QAction('Homing', self)
        homing_action.triggered.connect(self.show_homing)
        calibrate_menu = QMenu('Calibrate', self)
        calibrate_menu.addAction(homing_action)
        self.menuBar().addMenu(calibrate_menu)

    def show_homing(self):
        # Placeholder for homing calibration action
        QMessageBox.information(self, 'Homing', 'Performing homing calibration...')
        self.perform_system_check()

    def show_idle(self):
        # Placeholder for homing calibration action
        QMessageBox.information(self, 'Idle', 'Turning system idle...')
        self.perform_system_check()


    def show_general_help(self):
        self.player = VideoPlayer()  # Instantiate VideoPlayer
        self.player.show()  # Show the player window
        self.player.play_video()  # Start the video playback

    def close_application(self):
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
