from PyQt5.QtCore import QDir, Qt
from PyQt5.QtWidgets import (QApplication, QDialog, QGridLayout,
                            QPushButton, QAbstractItemView, QWidget, 
                            QFileDialog, QListWidget, QListWidgetItem, QMessageBox)
import pandas as pd
from PyQt5.QtGui import *
import matplotlib.pyplot as plt

class mazu_Visualizer(QDialog):
    def __init__(self, parent=None):
        super(mazu_Visualizer, self).__init__(parent)
        self.setWindowTitle('Mazu simulator')
        self.resize(500, 500)
        
        self.wid = QWidget(self)
        self.listWidget = QListWidget(self)
        self.listWidget.itemSelectionChanged.connect(self.selectionChanged)
        self.listWidget.setSelectionMode(QAbstractItemView.ExtendedSelection)

        self.button = QPushButton('Import')
        self.button.clicked.connect(self.browse)

        self.pltbotton = QPushButton('Plot')
        self.pltbotton.clicked.connect(self.plot)

        self.clearbutton = QPushButton('Clear')
        self.clearbutton.clicked.connect(self.listWidget.clear)

        self.mpbutton = QPushButton('Multi-plot')
        self.mpbutton.clicked.connect(self.mplot)

        self.layout = QGridLayout(self)
        self.layout.addWidget(self.listWidget, 0, 1, 1, 2)
        self.layout.addWidget(self.button, 1, 0)
        self.layout.addWidget(self.pltbotton, 1, 1)
        self.layout.addWidget(self.clearbutton, 1, 2)
        self.layout.addWidget(self.mpbutton, 1, 3)
        self.layout.setAlignment(Qt.AlignHCenter)
        self.wid.setLayout(self.layout)

        self.selected = []

        self.alert_msg = 'Please select something to plot !!'

    def browse(self):
        self.fileName_choose, self.filetype = QFileDialog.getOpenFileName(self, "Find Files", QDir.currentPath(), 'CSV (*.csv)')
        self.csv = pd.read_csv(self.fileName_choose)
        self.index = self.csv.columns
        for str in self.index:
            item = QListWidgetItem(str)
            self.listWidget.addItem(item)

    def selectionChanged(self):
        self.selected.clear()
        for i in self.listWidget.selectedItems():
            self.selected.append(i.text())

    def plot(self):
        if len(self.selected) == 0:
            alert = QMessageBox()
            alert.setText(self.alert_msg)
            alert.exec_()
        else:
            for i in self.selected:
                plt.figure(i)
                plt.xlabel('Time (sec)')
                plt.ylabel(i)
                plt.plot(self.csv.loc[:, self.index[0]], self.csv.loc[:, i])
                plt.grid()
            plt.show()

    def mplot(self):
        if len(self.selected) == 0:
            alert = QMessageBox()
            alert.setText(self.alert_msg)
            alert.exec_()
        else:
            plt.figure()
            plt.xlabel('Time (sec)')
            for i in self.selected:
                plt.plot(self.csv.loc[:, self.index[0]], self.csv.loc[:, i])
            plt.grid()
            plt.show()

if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    gallery = mazu_Visualizer()
    gallery.show()
    sys.exit(app.exec_())
