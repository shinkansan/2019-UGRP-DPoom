"""
Dpoom Face Expression Windows 2019
"""

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtPrintSupport import *
import fall_body_1013 as fall_body
import os
import sys
import numpy as np
import argparse
import imutils
import time
import cv2
import os
import pyrealsense2 as rs
import threading
import matplotlib.pyplot as plt
import uuid
import queue



specificSet = [
'/Users/shinkansan/anaconda3/envs/HyunSoo/lib/python36.zip',
'/Users/shinkansan/anaconda3/envs/HyunSoo/lib/python3.6',
'/Users/shinkansan/anaconda3/envs/HyunSoo/lib/python3.6/lib-dynload',
'/Users/shinkansan/anaconda3/envs/HyunSoo/lib/python3.6/site-packages']

#sys.path = specificSet
MainIndex = "file:///home/dpoom2/dpoom_few/index.html"

class AboutDialog(QDialog):
    def __init__(self, *args, **kwargs):
        super(AboutDialog, self).__init__(*args, **kwargs)

        QBtn = QDialogButtonBox.Ok  # No cancel
        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        layout = QVBoxLayout()

        title = QLabel("DPoom FEW")
        font = title.font()
        font.setPointSize(20)
        title.setFont(font)

        layout.addWidget(title)

        layout.addWidget(QLabel("Version 1"))
        layout.addWidget(QLabel("Copyright TEAM DPOOM."))

        for i in range(0, layout.count()):
            layout.itemAt(i).setAlignment(Qt.AlignHCenter)

        layout.addWidget(self.buttonBox)

        self.setLayout(layout)



class MainWindow(QMainWindow):
    thread_signal = pyqtSignal()
    send_instances_signal = pyqtSignal("PyQt_PyObject")
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.status_emeregency = False
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl(MainIndex))

        self.browser.urlChanged.connect(self.update_urlbar)
        self.browser.loadFinished.connect(self.update_title)
        self.browser.loadFinished.connect(self.setDefaultExpr)
        self.setCentralWidget(self.browser)

        self.status = QStatusBar()
        self.setStatusBar(self.status)

        navtb = QToolBar("Navigation")
        navtb.setIconSize(QSize(16, 16))
        #self.addToolBar(navtb)

        back_btn = QAction("Back", self)
        back_btn.setStatusTip("Back to previous page")
        back_btn.triggered.connect(self.browser.back)
        navtb.addAction(back_btn)

        next_btn = QAction(QIcon(os.path.join('images', 'arrow-000.png')), "Forward", self)
        next_btn.setStatusTip("Forward to next page")
        next_btn.triggered.connect(self.browser.forward)
        navtb.addAction(next_btn)

        reload_btn = QAction(QIcon(os.path.join('images', 'arrow-circle-315.png')), "Reload", self)
        reload_btn.setStatusTip("Reload page")
        reload_btn.triggered.connect(self.browser.reload)
        navtb.addAction(reload_btn)

        home_btn = QAction(QIcon(os.path.join('images', 'home.png')), "Home", self)
        home_btn.setStatusTip("Go home")
        home_btn.triggered.connect(self.navigate_home)
        navtb.addAction(home_btn)

        navtb.addSeparator()


        self.urlbar = QLineEdit()
        self.urlbar.returnPressed.connect(self.navigate_to_url)
        navtb.addWidget(self.urlbar)

        stop_btn = QAction( "Stop", self)
        stop_btn.setStatusTip("Stop loading current page")
        stop_btn.triggered.connect(self.browser.stop)
        navtb.addAction(stop_btn)

        # Uncomment to disable native menubar on Mac
        # self.menuBar().setNativeMenuBar(False)

        file_menu = self.menuBar().addMenu("&File")

        open_file_action = QAction( "Open file...", self)
        open_file_action.setStatusTip("Open from file")
        open_file_action.triggered.connect(self.open_file)
        file_menu.addAction(open_file_action)

        # save_file_action = QAction(QIcon(os.path.join('images', 'disk--pencil.png')), "Save Page As...", self)
        # save_file_action.setStatusTip("Save current page to file")
        # save_file_action.triggered.connect(self.save_file)
        # file_menu.addAction(save_file_action)

        # print_action = QAction(QIcon(os.path.join('images', 'printer.png')), "Print...", self)
        # print_action.setStatusTip("Print current page")
        # print_action.triggered.connect(self.print_page)
        #file_menu.addAction(print_action)

        about_action = QAction("Specif Setting", self)
        about_action.setStatusTip("detail")  # Hungry!
        about_action.triggered.connect(self.about)
        file_menu.addAction(about_action)

        navigate_mozarella_action = QAction("Go Homepage", self)
        navigate_mozarella_action.setStatusTip("Go to Dpoom home")
        navigate_mozarella_action.triggered.connect(self.navigate_mozarella)
        file_menu.addAction(navigate_mozarella_action)
        self.showFullScreen()
        self.show()

        self.th = Worker(parent=self)
        self.th.start()

        self.th2 = YoloWorker(parent=self)
        self.th2.start()

        self.setWindowIcon(QIcon(os.path.join('images', 'ma-icon-64.png')))

    def setDefaultExpr(self):
        self.browser.page().runJavaScript("eyes.startBlinking()")
        print('set default expr')



    def setExpr(self, classN):
        emoClass = {
        0:"eyes.startBlinking()",
        1:"eyes.stopBlinking()",
        2:"eyes.blink()",
        3:"eyes.express({type: 'happy'})",
        4:"eyes.express({type: 'sad'})",
        5:"eyes.express({type: 'angry'})",
        6:"eyes.express({type: 'focused'})",
        7:"eyes.express({type: 'confused'})"
        }
        self.browser.page().runJavaScript(emoClass.get(classN))
        pass

    def declareEmergency(self):
        self.status_emeregency = not self.status_emeregency
        if self.status_emeregency:
            self.browser.page().runJavaScript('clearInterval(light)')
            self.browser.page().runJavaScript('var light = setInterval("lightning()",360);')
        else:
            self.browser.page().runJavaScript('clearInterval(light)')
            self.browser.page().runJavaScript('var light = setInterval("getBackwhite()",360);')



    def update_title(self):
        title = self.browser.page().title()
        self.setWindowTitle("Dpoom FEW")

    def navigate_mozarella(self):
        self.browser.setUrl(MainIndex)

    def about(self):
        dlg = AboutDialog()
        dlg.exec_()

    def open_file(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Open file", "",
                                                  "Hypertext Markup Language (*.htm *.html);;"
                                                  "All files (*.*)")

        if filename:
            with open(filename, 'r') as f:
                html = f.read()

            self.browser.setHtml(html)
            self.urlbar.setText(filename)

    def save_file(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Page As", "",
                                                  "Hypertext Markup Language (*.htm *html);;"
                                                  "All files (*.*)")

        if filename:
            html = self.browser.page().toHtml()
            with open(filename, 'w') as f:
                f.write(html)

    def print_page(self):
        dlg = QPrintPreviewDialog()
        dlg.paintRequested.connect(self.browser.print_)
        dlg.exec_()

    def navigate_home(self):
        self.browser.setUrl(QUrl(""))

    def navigate_to_url(self):  # Does not receive the Url
        q = QUrl(self.urlbar.text())
        if q.scheme() == "":
            q.setScheme("http")

        self.browser.setUrl(q)

    def update_urlbar(self, q):

        if q.scheme() == 'https':
            # Secure padlock icon
            pass

        else:
            # Insecure padlock icon
            pass

        #self.urlbar.setText(q.toString())
        #self.urlbar.setCursorPosition(0)




class Worker(QThread):
    #sec_changed = pyqtSignal(str)

    def __init__(self, sec=0, parent=None):
        super(Worker, self).__init__()
        self.main = parent
        self.working = True
        self.sec = sec

        # self.main.add_sec_signal.connect(self.add_sec)   # custom signal from main thread to worker thread

    def __del__(self):
        print(".... end thread.....")
        self.wait()

    def defaultAction(self):
        while(True):
            if fall_body.fallFlag:
                print("fall body detected!!!!!!!")
            elif fall_body.humanFlag:
                print("human detected !!!")

                ###cascade_1013

                emoNumber= int(np.random.uniform(3, 8))
                try:
                    emoNumber = int(emoNumber)
                except:
                    pass
                    #window.about()
                else:
                    window.setExpr(int(emoNumber))
                    time.sleep(3)
                    print('active')

    def run(self):
        self.defaultAction();

class YoloWorker(QThread):


    def __init__(self, parent=None):
        super(YoloWorker, self).__init__()
        self.main = parent
        self.working = True

    def __del__(self):
        print('yolo thread dead')
        self.wait()

    def yolo_main(self):
        print('yolo thread working')
        if self.working:
            self.working = not self.working

            fall_body.main(verbose=0)

    def run(self):
        self.yolo_main()



app = QApplication(sys.argv)
app.setApplicationName("Dpoom FEW")
app.setOrganizationName("Dpoom FEW")
app.setOrganizationDomain("github.com/shinkansan")

window = MainWindow()


app.exec_()
