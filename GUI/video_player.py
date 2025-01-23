import sys
import cv2
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget

class VideoPlayer(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the main window
        self.setWindowTitle("Video Player")
        self.setGeometry(100, 100, 800, 600)

        # Set up the central widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Video display label
        self.video_label = QLabel(self)
        self.layout.addWidget(self.video_label)


        self.cap = cv2.VideoCapture("video.mp4")
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

        # Start video
        self.play_video()

    def play_video(self):
        if not self.cap.isOpened():
            self.video_label.setText("Error: Could not open video file.")
            return
        self.timer.start(30)  # Update frame every 30 ms

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_image))
        else:
            self.timer.stop()
            self.video_label.setText("End of video.")


    def closeEvent(self, event):

        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    player = VideoPlayer()
    player.show()
    sys.exit(app.exec_())
