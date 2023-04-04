import sys
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *

from events import *


def window():
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("SCRobotics Science")

    window.move(500,200)
    window.setStyleSheet(
        "background-color: #FFFFFF;"
    )
    grid = QGridLayout()

    #############################################
    groupBoxButtons = QGroupBox("Packages Controls")
    startVacuum = QPushButton("Start Vacuum")
    stopVacuum = QPushButton("Stop Vacuum")
    startPump = QPushButton("Start Pump")
    stopPump = QPushButton("Stop Pump")
    recordingBox = QCheckBox("Recording")

    # startVacuum.clicked.connect(button1_clicked)
    # stopVacuum.clicked.connect(button2_clicked)

    groupButtons = QVBoxLayout()
    groupButtons.addWidget(startVacuum)
    groupButtons.addWidget(stopVacuum)
    groupButtons.addWidget(startPump)
    groupButtons.addWidget(stopPump)
    groupButtons.addWidget(recordingBox)

    groupBoxButtons.setLayout(groupButtons)

    ###############################################

    groupRangeBox = QGroupBox("Funnel Cake")
    funnelCake = QDial()

    groupRange = QVBoxLayout()
    groupRange.addWidget(funnelCake)
    groupRangeBox.setLayout(groupRange)

    ################################################

    groupSliderBox = QGroupBox("Lowering Platform")
    loweringPlat = QSlider(Qt.Vertical)
    loweringPlat.setInvertedAppearance(True)

    groupRange.addWidget(loweringPlat)

    groupSlide = QVBoxLayout()
    groupSlide.addWidget(loweringPlat)
    groupSliderBox.setLayout(groupSlide)

    ################################################

    grid.addWidget(groupBoxButtons,1,0)
    grid.addWidget(groupRangeBox,1,1)
    grid.addWidget(groupSliderBox,1,2)

    ###############################################

    window.show()
    window.setLayout(grid)
    sys.exit(app.exec())




if __name__ == '__main__':
    window()





# from PyQt5.QtWidgets import *
# from PyQt5.QtCore import Qt
# from PyQt5.QtGui import *

# from events import *



# class Window(QMainWindow):
#     def __init__(self, parent=None):
#         super(Window, self).__init__(parent)
#         self.setWindowTitle("SCRobotics Science")
#         self.move(500,200)
#         self.setStyleSheet(
#             "background-color: #FFFFFF;"
#         )
#         self.ui()

#     def ui(self):
#         self.grid = QGridLayout()

#         #############################################
#         self.groupBoxButtons = QGroupBox("Packages Controls")
#         self.startVacuum = QPushButton("Start Vacuum")
#         self.stopVacuum = QPushButton("Stop Vacuum")
#         self.startPump = QPushButton("Start Pump")
#         self.stopPump = QPushButton("Stop Pump")
#         self.recordingBox = QCheckBox("Recording")

#         # startVacuum.clicked.connect(button1_clicked)
#         # stopVacuum.clicked.connect(button2_clicked)

#         self.groupButtons = QVBoxLayout()
#         self.groupButtons.addWidget(self.startVacuum)
#         self.groupButtons.addWidget(self.stopVacuum)
#         self.groupButtons.addWidget(self.startPump)
#         self.groupButtons.addWidget(self.stopPump)
#         self.groupButtons.addWidget(self.recordingBox)

#         self.groupBoxButtons.setLayout(self.groupButtons)

#         ###############################################

#         self.groupRangeBox = QGroupBox("Funnel Cake")
#         self.funnelCake = QDial()

#         self.groupRange = QVBoxLayout()
#         self.groupRange.addWidget(self.funnelCake)
#         self.groupRangeBox.setLayout(self.groupRange)

#         ################################################

#         self.groupSliderBox = QGroupBox("Lowering Platform")
#         self.loweringPlat = QSlider(Qt.Vertical)
#         self.loweringPlat.setInvertedAppearance(True)

#         self.groupRange.addWidget(self.loweringPlat)

#         self.groupSlide = QVBoxLayout()
#         self.groupSlide.addWidget(self.loweringPlat)
#         self.groupSliderBox.setLayout(self.groupSlide)

#         ################################################

#         self.grid.addWidget(self.groupBoxButtons,1,0)
#         self.grid.addWidget(self.groupRangeBox,1,1)
#         self.grid.addWidget(self.groupSliderBox,1,2)