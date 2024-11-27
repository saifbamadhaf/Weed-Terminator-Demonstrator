import os
import sys
import subprocess  # Import subprocess module
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel,
                             QVBoxLayout, QWidget, QMessageBox, QGraphicsOpacityEffect, QHBoxLayout)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QAbstractAnimation
from PyQt5.QtGui import QFont

# Define the workspace directory
workspace_dir = '/home/saifbamadhaf/ros2_ws'


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Weed Control System')
        self.process = None

        self.showFullScreen()


        # Main layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(20)  # Space between widgets
        self.layout.setAlignment(Qt.AlignCenter)  # Center alignment

        # Welcome label
        self.welcome_label = QLabel('Welcome to the Weed Control System')
        self.welcome_label.setFont(QFont('Arial', 50, QFont.Bold))
        self.welcome_label.setAlignment(Qt.AlignCenter)
        self.welcome_label.setStyleSheet("color: #2E4053;")

        # System check label
        self.check_label = QLabel('Performing system check...')
        self.check_label.setFont(QFont('Arial', 36))
        self.check_label.setAlignment(Qt.AlignCenter)
        self.check_label.setStyleSheet("color: #2E4053;")
        self.check_label.setVisible(False)  # Hidden initially

        # Choose mode
        self.select_label = QLabel('Select your preferred mode to start:')
        self.select_label.setFont(QFont('Arial', 20))
        self.select_label.setAlignment(Qt.AlignCenter)
        self.select_label.setStyleSheet("color: #2E4053;")
        self.select_label.setVisible(False)  # Hidden initially

        # Mode buttons
        self.auto_mode_button = QPushButton('Automatic Mode (Infinite Termination)')
        self.auto_mode_button.setFont(QFont('Arial', 20))
        self.auto_mode_button.clicked.connect(self.start_auto_mode)
        self.auto_mode_button.setEnabled(False)  # Disabled until system check is done
        self.auto_mode_button.setFixedSize(600, 100)  # Adjust size

        self.manual_mode_button = QPushButton('Manual Control Mode')
        self.manual_mode_button.setFont(QFont('Arial', 20))
        self.manual_mode_button.clicked.connect(self.start_manual_mode)
        self.manual_mode_button.setEnabled(False)  # Disabled until system check is done
        self.manual_mode_button.setFixedSize(600, 100)  # Adjust size

        # Help buttons
        self.auto_help_button = QPushButton('Help')
        self.auto_help_button.setFixedSize(100, 50)
        self.auto_help_button.setStyleSheet("background-color: black; color: white; border-radius: 5px;")
        self.auto_help_button.clicked.connect(self.show_auto_help)

        self.manual_help_button = QPushButton('Help')
        self.manual_help_button.setFixedSize(100, 50)
        self.manual_help_button.setStyleSheet("background-color: black; color: white; border-radius: 5px;")
        self.manual_help_button.clicked.connect(self.show_manual_help)

        # General help button
        self.general_help_button = QPushButton('General Help')
        self.general_help_button.setFont(QFont('Arial', 16))
        self.general_help_button.setStyleSheet("background-color: black; color: white; border-radius: 10px;")
        self.general_help_button.setFixedSize(200, 100)
        self.general_help_button.clicked.connect(self.show_general_help)

        # Exit button
        self.exit_button = QPushButton('Exit')
        self.exit_button.setFont(QFont('Arial', 20))
        self.exit_button.setFixedSize(200, 100)
        self.exit_button.setStyleSheet("background-color: #E74C3C; color: white; border-radius: 10px;")
        self.exit_button.clicked.connect(self.close_application)

        # Add widgets to layout
        self.layout.addWidget(self.welcome_label)
        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.layout)

        # Start the GUI flow
        self.start_gui_flow()

        # Styling
        self.setStyleSheet("""
                   QWidget {
                       background-color: #ECF0F1;  
                   }
                   QLabel {
                       color: #2E4053;
                       padding: 30px;
                       font-weight: bold;
                   }
                   QPushButton {
                       background-color: #3498DB;
                       color: white;
                       border-radius: 10px;
                       border: none;  /* Updated border style */
                       font-weight: bold;
                       font-size: 25px;  /* Slightly reduced font size for consistency */
                       box-shadow: 3px 3px 6px rgba(0, 0, 0, 0.2);
                       transition: background-color 0.3s ease, transform 0.2s;
                   }
                   QPushButton:hover {
                       background-color: #2980B9;
                       transform: scale(1.05);
                   }
                   QPushButton:pressed {
                       background-color: #1B4F72;
                       transform: scale(0.95);
                   }
               """)

    def start_gui_flow(self):
        # Fade out welcome label
        self.welcome_label.setGraphicsEffect(self.create_fade_effect(1.0, 0.0, 2000))  # Fade out over 2 seconds
        QTimer.singleShot(0, self.perform_system_check)  # Perform system check after 2 seconds

    def perform_system_check(self):
        self.check_label.setVisible(True)
        self.check_label.setText('System check in progress...')
        self.layout.addWidget(self.check_label)

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

        # Update text after 2 seconds
        QTimer.singleShot(5000, lambda: self.check_label.setText('Finalizing system check... \n Please wait...'))

        # Show options after 4 seconds
        QTimer.singleShot(6000, self.show_options)

    def show_options(self):
        self.welcome_label.setText('Weed Control System')
        self.check_label.setVisible(False)
        self.auto_mode_button.setEnabled(True)
        self.manual_mode_button.setEnabled(True)

        # Create a layout for the welcome label
        welcome_layout = QVBoxLayout()
        welcome_layout.addWidget(self.welcome_label, alignment=Qt.AlignTop | Qt.AlignHCenter)

        # Add the welcome layout to the main layout
        self.layout.addLayout(welcome_layout)

        # Add a spacer to push the mode buttons down
        self.layout.addStretch()

        # Create a layout for the mode buttons
        button_layout = QVBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        # Show select label
        self.select_label.setVisible(True)  # Make the label visible
        self.layout.addWidget(self.select_label)  # Add it to the layout

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

        # Add another spacer to push the exit/help buttons down to the bottom
        self.layout.addStretch()

        # Create a layout for the exit and help buttons
        exit_layout = QHBoxLayout()
        exit_layout.addStretch()  # This will push the buttons to the right
        exit_layout.addWidget(self.general_help_button)
        exit_layout.addWidget(self.exit_button)

        # Add the exit layout to the main layout
        self.layout.addLayout(exit_layout)

    def create_fade_effect(self, start_opacity, end_opacity, duration):
        effect = QGraphicsOpacityEffect()
        effect.setOpacity(start_opacity)
        self.welcome_label.setGraphicsEffect(effect)
        animation = QPropertyAnimation(effect, b"opacity")
        animation.setDuration(duration)
        animation.setStartValue(start_opacity)
        animation.setEndValue(end_opacity)
        animation.start(QAbstractAnimation.DeleteWhenStopped)
        return effect

    def start_auto_mode(self):


        try:
            launch_command = f'source {workspace_dir}/install/setup.bash; ' \
                             f'ros2 launch cartesian_positioner_controller demo_mode_dynamic.launch.py setup_type:=conveyor-weeder sim:=true'

            # Execute the launch command
            self.launch_process = subprocess.Popen(['bash', '-c', launch_command], stdout=subprocess.PIPE,
                                                   stderr=subprocess.PIPE)

            # Inform user about the simulation running
            QMessageBox.information(self, 'Success', 'The simulation is now running in the background.')

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An exception occurred: {str(e)}')

    def start_manual_mode(self):
        QMessageBox.information(self, 'Manual Mode', 'Switching to manual control mode...')

    def show_auto_help(self):
        QMessageBox.information(self, 'Automatic Mode Help',
                                'Automatic Mode continuously terminates weeds in the selected area without further user input.')

    def show_manual_help(self):
        QMessageBox.information(self, 'Manual Mode Help',
                                'Manual Control Mode allows you to control the system manually for targeted weed termination.')

    def show_general_help(self):
        QMessageBox.information(self, 'General Help',
                                'This application helps manage weed termination using an autonomous system.')

    def close_application(self):
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())