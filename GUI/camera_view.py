import sys
import cv2
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QAbstractAnimation
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel,
                             QVBoxLayout, QWidget, QMessageBox, QGraphicsOpacityEffect, QHBoxLayout, QMenuBar, QMenu,
                             QAction, QSizePolicy)
import os
import subprocess

# Define the workspace directory
workspace_dir = '/home/saifbamadhaf/ros2_ws'


class CameraView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Weed Control System')
        self.process = None

        # Assuming you are using a QMainWindow or QWidget
        self.showFullScreen()  # This is the correct place to call showMaximized()

        # Main layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(20)  # Space between widgets
        self.layout.setAlignment(Qt.AlignCenter)

        # Welcome label
        self.welcome_label = QLabel('Weed Control System')
        self.welcome_label.setFont(QFont('Arial', 50, QFont.Bold))
        self.welcome_label.setAlignment(Qt.AlignCenter)
        self.welcome_label.setStyleSheet("color: white; background-color: #3C3C3C;")

        # Exit button
        self.exit_buttons = QPushButton('Back ↩')
        self.exit_buttons.setFixedSize(300, 150)
        self.exit_buttons.setStyleSheet("background-color: black; color: white; border-radius: 10px;")
        self.exit_buttons.clicked.connect(self.back)

        # QLabel to display the video feed
        self.video_label = QLabel("Camera Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Layout setup
        self.layout.addWidget(self.welcome_label)
        self.layout.addWidget(self.video_label)

        self.setCentralWidget(QWidget())
        self.centralWidget().setLayout(self.layout)

        # Initialize camera with higher resolution (e.g., 1920x1080 or 3840x2160)
        self.cap = cv2.VideoCapture(0)  # Adjust index if necessary


        # Timer to update frames
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Update the frame every 30ms


        self.show_option()

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
                       font-size: 30px; /* Increase text size */
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

        menu_action = QAction('Main Menu', self)
        homing_action = QAction('Homing', self)
        homing_action.triggered.connect(self.show_homing)


        menu_action.triggered.connect(self.show_Main_Menu)

        # Add actions to menus
        home.addAction(menu_action)

        home.addAction(homing_action)

    def show_option(self):
        self.welcome_label.setText('Weed Control System')

        # Create a layout for the welcome label
        welcome_layout = QVBoxLayout()
        welcome_layout.addWidget(self.welcome_label, alignment=Qt.AlignTop | Qt.AlignHCenter)

        # Add the welcome layout to the main layout
        self.layout.addLayout(welcome_layout)

        self.layout.addStretch()

        self.layout.addWidget(self.video_label)

        # Add a spacer to push the mode buttons down
        self.layout.addStretch()

        button_layout = QVBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        self.layout.addStretch()

        # Create a layout for the exit button
        exit_layout = QHBoxLayout()
        exit_layout.addStretch()
        exit_layout.addWidget(self.exit_buttons)

        self.layout.addLayout(exit_layout)

    def back(self):
        from GUI import MainWindow
        self.window = MainWindow()  # Instantiate VideoPlayer
        self.window.show()
        self.close()

    def show_Main_Menu(self):
        # Delay the import of MainWindow to avoid circular import
        from GUI import MainWindow
        self.window = MainWindow()  # Instantiate VideoPlayer
        self.window.show()
        self.close()

    def show_homing(self):
        QMessageBox.information(self, "Homing", "Homing action triggered!")

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Get the original frame size
            height, width, _ = frame.shape

            # Set higher resolution (optional)
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

            # Resize using better interpolation
            left_frame = frame[:, :width // 2]  # Left half of the frame

            # Convert to RGB format
            left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)

            # Resize with INTER_CUBIC for better quality
            left_frame = cv2.resize(left_frame, (width * 1, height * 2), interpolation=cv2.INTER_CUBIC)

            # Convert to QImage
            h, w, ch = left_frame.shape
            bytes_per_line = ch * w
            qimg = QImage(left_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

            # Display in QLabel
            pixmap = QPixmap.fromImage(qimg)
            self.video_label.setPixmap(pixmap)

    def closeEvent(self, event):
        if self.cap.isOpened():
            self.cap.release()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CameraView()
    window.show()
    sys.exit(app.exec_())
