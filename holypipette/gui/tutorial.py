import logging

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTabWidget, QTextBrowser
from PyQt5 import QtCore, QtWidgets, QtGui

import time
import numpy as np
from collections import deque

__all__ = ["Tutorial"]

class Tutorial(QWidget):
    """A window that helps the user learn how to use the simulation.
    """
    
    def __init__(self, parent=None):
        super().__init__()

        #setup window
        self.setWindowTitle("Tutorial")

        #set fixed size
        self.setFixedSize(700, 700)

        # self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        #create layout with different tabpages
        self.layout = QVBoxLayout(self)

        #create label
        self.label = QLabel(self)
        self.label.setText("Welcome to the tutorial! Click on the different tabs to learn how to use the simulation.")
        self.label.setWordWrap(True)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.label)

        #create tab widget
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setTabPosition(QtWidgets.QTabWidget.South)

        #create tab pages
        self._createTabPages()

        #add tab widget to layout
        self.layout.addWidget(self.tabs)


    def _createTabPages(self):
        #create tab pages
        self._createControlsPage()
        self._createPatchingBasicsPage()
        self._createManualPatchingPage()
        self._createAutomatedPatchingPage()
        self._createPitfallsPage()

    def _createControlsPage(self):
        self.controlsPage = QTabWidget()

        #create the XBox Controller page
        self.xboxPage = QWidget()
        self.xboxPage.layout = QVBoxLayout(self.xboxPage)
        #add image
        self.xboxPage.image = QLabel(self.xboxPage)
        self.xboxPage.image.setPixmap(QtGui.QPixmap('holypipette/gui/tutorial_media/XBox.png'))
        self.xboxPage.image.setAlignment(QtCore.Qt.AlignCenter)
        self.xboxPage.layout.addWidget(self.xboxPage.image)
        self.xboxPage.image.setScaledContents(True)

        #create the keyboard controls page
        self.keyboardPage = QWidget()
        self.keyboardPage.layout = QVBoxLayout(self.keyboardPage)

        # Create a QTextBrowser to display the bulleted list
        text_browser = QTextBrowser(self)
        self.keyboardPage.layout.addWidget(text_browser)

        # Create a string with a bulleted list using HTML tags
        bulleted_list = """
        <style>
        /* Fix list spacing and appearance */
        ul {
            line-height: 1.15;
        }
        li {
            margin-bottom: 5px;
        }
        </style>

        <p style="font-size: 20px;">
            The Stage is controlled using WASD and QE keys:
        </p>

        <ul style="font-size: 20px;">
            <li>A / D: + / - Stage X</li>
            <li>W / S: + / - Stage Y</li>
            <li>Q / E: + / - Stage Z</li>
        </ul>
        <br>

        <p style="font-size: 20px;">
        The Pipette is controlled using the arrow and page up/down keys:
        </p>
        <ul style="font-size: 20px;">
            <li>Left / Right: + / - Pipette X</li>
            <li>Up / Down: + / - Pipette Y</li>
            <li>Page Up / Page Down: + / - Pipette Z</li>
        </ul>
        <br>
        <p style="font-size: 20px;">
        Additionally, you can right click anywhere on the image to center the stage on that position.
        </p>

        """
        text_browser.setHtml(bulleted_list)

        self.controlsPage.addTab(self.xboxPage, "XBox Controller")
        self.controlsPage.addTab(self.keyboardPage, "Keyboard Controls")

        self.tabs.addTab(self.controlsPage, "Controls")
    def _createPatchingBasicsPage(self):
        self.basicsPage = QWidget()
        self.basicsPage.layout = QVBoxLayout(self.basicsPage)
        self.basicsPage.label = QLabel(self.basicsPage)
        self.basicsPage.label.setText("This is the basics page.")
        self.basicsPage.label.setWordWrap(True)
        self.basicsPage.label.setAlignment(QtCore.Qt.AlignCenter)
        self.basicsPage.layout.addWidget(self.basicsPage.label)
        self.tabs.addTab(self.basicsPage, "Patching Basics")

    def _createManualPatchingPage(self):
        #create tutorial page
        self.manualPage = QWidget()
        self.manualPage.layout = QVBoxLayout(self.manualPage)

        # Create a QTextBrowser to display the bulleted list
        text_browser = QTextBrowser(self)
        self.keyboardPage.layout.addWidget(text_browser)

        # Create a string with a bulleted list using HTML tags
        bulleted_list = """
        <style>
        /* Fix list spacing and appearance */
        ul {
            line-height: 1.15;
        }
        li {
            margin-bottom: 5px;
        }
        </style>

        <p style="font-size: 20px;">
            Manual Patching Walkthrough:
        </p>
        <ul style="font-size: 20px;">
            <li>Switch to the <strong>Manual Patching</strong> tab in the right hand side of the simulator</li>
            <li>Lower the pipette and move it over the cell you want to patch</li>
            <ul>
                <li>If you lower the pipette too much, you can destroy the pipette tip.  If this happens, click <strong>Replace Pipette</strong> to change out the broken pipette.</li>
            </ul>
            <li>Once you're within ~20um of the cell in the z direction, resistance will increase by a few Mega-Ohms</li>
            <li>Click <strong>Gigaseal Pressure</strong> to create a slight negative pressure</li>
            <li>After a few seconds, resistance should spike to over a Giga-Ohm.  Now click <strong> Break-In Pressure</li> to get through the membrane</li>
            <li>Resistance should drop to a few hundred Mega-Ohms and you should see the characteristic current response.</li>
            <li>After observing the response, you can raise the pipette and click <strong>Clean Pipette</strong> to get the pipette ready to patch again.</li>
        </ul>
        <br>

        """
        text_browser.setHtml(bulleted_list)
        self.manualPage.layout.addWidget(text_browser)
        self.tabs.addTab(self.manualPage, "Manual Patching")

    def _createAutomatedPatchingPage(self):
        self.autoPage = QWidget()
        self.autoPage.layout = QVBoxLayout(self.autoPage)
        # Create a QTextBrowser to display the bulleted list
        text_browser = QTextBrowser(self)

        # Create a string with a bulleted list using HTML tags
        bulleted_list = """        
        <style>
        /* Fix list spacing and appearance */
        ul {
            line-height: 1.15;
        }
        li {
            margin-bottom: 5px;
        }
        </style>
        <p style="font-size: 20px;">
            Automated Patching Walkthrough:
        </p>

        <ul style="font-size: 20px;">
            <li>Switch to the <strong>Automatic Patching</strong> tab in the right hand side of the simulator</li>
            <li>Click <strong>Set Cell Plane</strong> with cells in focus</li>
            <li>Click <strong>Select Cell</strong> and then click on a cell to patch in the picture.</li>
            <ul>
                <li>A green circle will appear at the point you clicked. This is the target point for the pipette.</li>
            </ul>
            <li>Click <strong>Start Patch</strong>.  The system will automatically attempt to find the cell, establish a gigaseal, and break in.</li>
            <li>After breaking in and observing the current response, you can raise the pipette, and click <strong>Clean Pipette</strong> to get the pipette ready to patch again</li>
        </ul>
        <br>

        """
        text_browser.setHtml(bulleted_list)
        self.autoPage.layout.addWidget(text_browser)
        self.tabs.addTab(self.autoPage, "Auto Patching")


    def _createPitfallsPage(self):
        #create tutorial page
        self.pitfallsPage = QWidget()
        self.pitfallsPage.layout = QVBoxLayout(self.pitfallsPage)
        self.pitfallsPage.label = QLabel(self.pitfallsPage)
        self.pitfallsPage.label.setText("This is the pitfalls page.")
        self.pitfallsPage.label.setWordWrap(True)
        self.pitfallsPage.label.setAlignment(QtCore.Qt.AlignCenter)
        self.pitfallsPage.layout.addWidget(self.pitfallsPage.label)
        self.tabs.addTab(self.pitfallsPage, "Pitfalls")


        