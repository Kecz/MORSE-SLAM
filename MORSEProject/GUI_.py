from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import threading
import subprocess
import psutil
from mega_main import *
import sys
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


matplotlib.pyplot.style.use('dark_background')


class MorseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mainWidget = QStackedWidget()
        self.setCentralWidget(self.mainWidget)
        self.setWindowOpacity(0.8)
        self.setStyleSheet(open("Style.qss", "r").read())
        logoWidget = LogoWidget()
        self.mainWidget.addWidget(logoWidget)
        QTimer.singleShot(1, lambda: self.switch())
        self.show()

    def switch(self):
        mainGUI = MainGUI()
        self.mainWidget.addWidget(mainGUI)
        self.mainWidget.setCurrentWidget(mainGUI)


class LogoWidget(QWidget):
    def __init__(self):
        super().__init__()
        imageLabel = QLabel(self)
        imageLabel.setPixmap(QPixmap('./logo.png'))
        imageLabel.setGeometry(0, 0, 1000, 860)
        layout = QHBoxLayout()
        layout.addWidget(imageLabel)
        self.setLayout(layout)


class MainGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.map3D = plt.figure(linewidth=1, edgecolor='g')
        self.canvas3D = FigureCanvas(self.map3D)
        self.map3D.add_subplot(111, projection="3d")
        self.ax3D = Axes3D(self.map3D)
        self.map3D.suptitle("Map 3D - estimation")
        
        self.map2D = plt.figure(linewidth=1, edgecolor='g')
        self.canvas = FigureCanvas(self.map2D)
        self.ax2D = self.map2D.add_subplot(1, 1, 1)
        self.map2D.suptitle("Map 2D - estimation")
        
        self.map3Dpose = plt.figure(linewidth=1, edgecolor='g')
        self.canvas3Dpose = FigureCanvas(self.map3Dpose)
        self.map3Dpose.add_subplot(111, projection="3d")
        self.ax3Dpose = Axes3D(self.map3Dpose)
        self.map3Dpose.suptitle("Map 3D - Pose")
        
        self.map2Dpose = plt.figure(linewidth=1, edgecolor='g')
        self.canvasPose = FigureCanvas(self.map2Dpose)
        self.ax2Dpose = self.map2Dpose.add_subplot(1, 1, 1)
        self.map2Dpose.suptitle("Map 2D - Pose")
        
        plt.ion()

        self.simTimeInput = QTextEdit(self)
        self.simTimeInput.setPlaceholderText("Enter simulation time here ")
        self.simTimeInput.setFixedHeight(32)
        self.stop_sim = False
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 1000, 860)

        start_button = QPushButton('Prepare Environment')
        start_button.clicked.connect(self.blender_callback)

        run_button = QPushButton('Run Simulation')
        run_button.clicked.connect(self.start_callback)

        stop_button = QPushButton('Stop Simulation')
        stop_button.clicked.connect(self.stop_callback)

        layout = QVBoxLayout()
        top_box = QHBoxLayout()
        mid_box = QHBoxLayout()
        bottom_box = QHBoxLayout()

        # top part of the GUI
        top_box.addWidget(start_button)
        top_box.addWidget(self.simTimeInput)
        top_box.addWidget(run_button)
        top_box.addWidget(stop_button)

        # mid part of the GUI
        mid_box.addWidget(self.canvas)
        mid_box.addWidget(self.canvasPose)

        # bottom part of the GUI
        bottom_box.addWidget(self.canvas3D)
        bottom_box.addWidget(self.canvas3Dpose)

        layout.addLayout(top_box, 5)
        layout.addLayout(mid_box, 5)
        layout.addLayout(bottom_box, 5)

        self.setLayout(layout)
        self.show()

    def blender_callback(self):
        subprocess.call("gnome-terminal --tab -- morse run env.py", shell=True)

    def start_callback(self):
        self.simtime = int(self.simTimeInput.toPlainText())
        print("pressed")
        self.sim_thread = threading.Thread(target=self.background_tasks)
        self.sim_thread.start()

    def background_tasks(self):
        self.megamain = MegaMain(self.ax2D, self.ax3D, self.ax2Dpose, self.ax3Dpose, self.simtime)
        self.megamain.run_simulation()

    def stop_callback(self):
        self.stop_sim = True
        PROCNAME = "blender"
        for proc in psutil.process_iter():
            if proc.name() == PROCNAME:
                proc.kill()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MorseGUI()
    sys.exit(app.exec_())
