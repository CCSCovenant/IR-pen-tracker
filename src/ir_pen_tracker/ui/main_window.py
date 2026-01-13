import cv2
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QStatusBar, QSplitter
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap

from ir_pen_tracker.logic.app_controller import AppController

class ScalableImageLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: black;")
        self._pixmap = None
        
    def setPixmap(self, pixmap):
        self._pixmap = pixmap
        self.update_display()
        
    def resizeEvent(self, event):
        self.update_display()
        super().resizeEvent(event)
        
    def update_display(self):
        if self._pixmap and not self._pixmap.isNull():
            # Scale pixmap to fit label size while maintaining aspect ratio
            scaled = self._pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            super().setPixmap(scaled)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinect Pen Tracker (PyQt Modular)")
        self.setGeometry(100, 100, 1600, 900)
        
        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main Layout (Vertical: Views + Controls)
        main_layout = QVBoxLayout(central_widget)
        
        # Views Layout (Horizontal Splitter: Color | (IR + Ortho))
        views_splitter = QSplitter(Qt.Horizontal)
        
        # Left: Color View
        self.color_label = ScalableImageLabel()
        views_splitter.addWidget(self.color_label)
        
        # Right: Split Vertical (IR + Ortho)
        right_splitter = QSplitter(Qt.Vertical)
        self.ir_label = ScalableImageLabel()
        self.ortho_label = ScalableImageLabel()
        
        right_splitter.addWidget(self.ir_label)
        right_splitter.addWidget(self.ortho_label)
        
        views_splitter.addWidget(right_splitter)
        
        # Set initial stretch factors (e.g., Color gets more space)
        views_splitter.setStretchFactor(0, 2)
        views_splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(views_splitter, stretch=1)
        
        # Controls
        controls_layout = QHBoxLayout()
        
        self.btn_record = QPushButton("Start Recording")
        self.btn_record.clicked.connect(self.on_record_clicked)
        controls_layout.addWidget(self.btn_record)
        
        self.btn_calibrate = QPushButton("Calibrate")
        self.btn_calibrate.clicked.connect(self.on_calibrate_clicked)
        controls_layout.addWidget(self.btn_calibrate)
        
        self.btn_confirm = QPushButton("Confirm Calibration (Space)")
        self.btn_confirm.clicked.connect(self.on_confirm_clicked)
        controls_layout.addWidget(self.btn_confirm)
        
        main_layout.addLayout(controls_layout)
        
        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # Controller
        self.controller = AppController()
        self.controller.frames_ready.connect(self.update_frames)
        self.controller.status_update.connect(self.update_status)
        
        # Start
        self.controller.start()
        
    def closeEvent(self, event):
        self.controller.stop()
        super().closeEvent(event)
        
    @pyqtSlot(dict)
    def update_frames(self, frames):
        # Update Color View
        if "color" in frames:
            self.display_image(frames["color"], self.color_label)
            
        # Update IR View
        if "ir" in frames:
            self.display_image(frames["ir"], self.ir_label)
             
        # Update Ortho View
        if "ortho" in frames:
            self.display_image(frames["ortho"], self.ortho_label)

    def display_image(self, img_np, label_widget):
        if img_np is None:
            return
        
        # Convert to QImage
        # Assuming BGR for Color/IR/Ortho (OpenCV default)
        # Check channels
        if len(img_np.shape) == 2:
            h, w = img_np.shape
            ch = 1
            fmt = QImage.Format_Grayscale8
            bytes_per_line = w
        else:
            h, w, ch = img_np.shape
            fmt = QImage.Format_RGB888
            bytes_per_line = ch * w
            # Convert BGR to RGB in place or copy? 
            # Controller sends BGR, so we need to swap for Qt
            # Note: Doing color conversion here on UI thread might be costly for 3 streams.
            # Ideally Controller sends RGB. But OpenCV draws on BGR.
            # Let's convert here.
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
            
        qt_image = QImage(img_np.data, w, h, bytes_per_line, fmt)
        # Copy to decouple from numpy buffer reuse
        pixmap = QPixmap.fromImage(qt_image.copy())
        label_widget.setPixmap(pixmap)
        
    @pyqtSlot(str)
    def update_status(self, text):
        self.status_bar.showMessage(text)
        
    def on_record_clicked(self):
        self.controller.toggle_recording()
        # Note: We don't have direct access to is_recording state here easily unless we track it 
        # or controller emits state change.
        # But we can rely on status updates or just toggle text.
        # Better: Controller emits signal with recording state.
        # For now, simplistic toggle text update based on next click or assume sync.
        # Let's leave text static or update based on status msg if needed.
        # Ideally: self.controller.rec_manager.is_recording (thread safe?)
        if self.controller.rec_manager.is_recording:
            self.btn_record.setText("Stop Recording")
            self.btn_record.setStyleSheet("background-color: red; color: white;")
        else:
            self.btn_record.setText("Start Recording")
            self.btn_record.setStyleSheet("")
            
    def on_calibrate_clicked(self):
        self.controller.start_calibration()
        
    def on_confirm_clicked(self):
        self.controller.confirm_calibration()
        
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            if self.controller.state == "CALIBRATE":
                self.controller.confirm_calibration()
            elif self.controller.state == "MAIN":
                self.on_record_clicked()
        elif event.key() == Qt.Key_C:
            self.controller.start_calibration()
