# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_launcher.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(645, 526)
        self.gridLayoutWidget_2 = QtWidgets.QWidget(MainWindow)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(50, 40, 491, 261))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.labelradius = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.labelradius.setObjectName("labelradius")
        self.gridLayout_2.addWidget(self.labelradius, 1, 1, 1, 1)
        self.labelNumRobots_2 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.labelNumRobots_2.setObjectName("labelNumRobots_2")
        self.gridLayout_2.addWidget(self.labelNumRobots_2, 1, 0, 1, 1)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.gridLayoutWidget_2)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.gridLayout_2.addWidget(self.doubleSpinBox, 2, 1, 1, 1)
        self.numRobotsSpinBox_2 = QtWidgets.QSpinBox(self.gridLayoutWidget_2)
        self.numRobotsSpinBox_2.setObjectName("numRobotsSpinBox_2")
        self.gridLayout_2.addWidget(self.numRobotsSpinBox_2, 2, 0, 1, 1)
        self.launchButton_2 = QtWidgets.QPushButton(MainWindow)
        self.launchButton_2.setGeometry(QtCore.QRect(190, 310, 258, 25))
        self.launchButton_2.setObjectName("launchButton_2")
        self.startMotionButton = QtWidgets.QPushButton(MainWindow)  # Added this line
        self.startMotionButton.setGeometry(QtCore.QRect(190, 340, 258, 25))  # Set position for start motion button
        self.startMotionButton.setObjectName("startMotionButton")  # Set object name
        self.logTextEdit = QtWidgets.QTextEdit(MainWindow)  # Added this line
        self.logTextEdit.setGeometry(QtCore.QRect(50, 380, 540, 100))  # Set position and size
        self.logTextEdit.setObjectName("logTextEdit")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robot Launcher"))
        self.labelradius.setText(_translate("MainWindow", "Radius of circle"))
        self.labelNumRobots_2.setText(_translate("MainWindow", "How many robots you want to launch?"))
        self.launchButton_2.setText(_translate("MainWindow", "Launch Robots"))
        self.startMotionButton.setText(_translate("MainWindow", "Start Motion"))
