# -*- coding: utf-8 -*-
import sys,os
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget
import Ui_mercury_x1
import robot_function
from PyQt5 import QtCore, QtGui, QtWidgets
#添加水星机器人库
#from pymycobot import Mercury
class MyMainForm(QMainWindow, Ui_mercury_x1.Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyMainForm, self).__init__(parent)
        self.setupUi(self)
        #添加水星机器人库
        # self.ml = Mercury('/dev/left_robot")
        # self.mr = Mercury('/dev/right_robot')
        self.action.triggered.connect(self.power_on_robot)
        self.action1.triggered.connect(self.return_to_zero)
        self.dispaly_manager = robot_function.DisplayManager(self)
        self.pushButton_fasong.clicked.connect(self.send_joint_command)
    def power_on_robot(self):
        """
        #功能：给水星机器人的双臂上电
        self.ml.power_on()
        self.mr.power_on()
        """
        print("power on robot")

        if not self.dispaly_manager:
            self.dispaly_manager = robot_function.DisplayManager(self)
        self.dispaly_manager.start()


    def update_joint_display(self,left_angles,right_angles):
        print(f"收到数据：左臂{left_angles} 右臂{right_angles}")  # 添加在方法开头
        if(len(left_angles) != 6 and len(right_angles) != 6):
            raise ValueError("关节数据长度必须为6")
        all_angles = left_angles + right_angles
        line_edits = [self.LineEdit1,self.LineEdit2,self.LineEdit3,self.LineEdit4,self.LineEdit5,self.LineEdit6,
                      self.LineEdit7,self.LineEdit8,self.LineEdit9,self.LineEdit10,self.LineEdit11,self.LineEdit12]
        for edit, value in zip(line_edits,all_angles):
            edit.setText(f"{value:.2f}°")
        
    def return_to_zero(self):
        result = self.dispaly_manager.return_to_zero_position()

        self.update_joint_display(*result)
        print("已发送回零指令")

    def send_joint_command(self):
        try:
            arm_type = "left" if self.comboBox.currentText() == "左臂" else "right"
            joint_id = self.comboBox_2.currentIndex() + 1
            degree = float(self.lineEdit_dushu.text())

            if arm_type == "left":
                #使用实际机器人时打开
                #robot_function.arm_2_angle(self.dispaly_manager.ml,joint_id,degree)
                self.dispaly_manager.worker.current_left[joint_id-1] = degree
            else:
                #使用实际机器人时打开
                #robot_function.arm_2_angle(self.dispaly_manager.mr,joint_id,degree)
                self.dispaly_manager.worker.current_right[joint_id-1] = degree

            self.dispaly_manager.update_joints()

            if not self.dispaly_manager.worker.timer.isActive():
                self.dispaly_manager.start()

        except ValueError as e:
            print("输入错误")
        
        except Exception as e:
            print("控制错误：", e)
if __name__ == '__main__':
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    # 每一pyqt5应用程序必须创建一个应用程序对象。sys.argv参数是一个列表，从命令行输入参数。
    app = QApplication(sys.argv)
    myWin = MyMainForm()
    # 显示在屏幕上
    myWin.show()
    # 系统exit()方法确保应用程序干净的退出
    # 的exec_()方法有下划线。因为执行是一个Python关键词。因此，exec_()代替
    sys.exit(app.exec_())