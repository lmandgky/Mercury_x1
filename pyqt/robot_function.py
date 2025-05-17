import Ui_mercury_x1
#from pymycobot import Mercury
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal,QObject,QTimer,QThread
import main

class DisplayWorker(QObject):
    update_signal = pyqtSignal(list, list)
    def __init__(self):
        super().__init__()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_joints)
        self.current_left = [30.0,45.5,15.3,0.0,90.0,10.2]
        self.current_right = [25.5, 40.0, 20.1, 5.5, 85.3, 8.7]
    
    def update_joints(self):
        self.update_signal.emit(self.current_left.copy(),self.current_right.copy())


class DisplayManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.worker = DisplayWorker()
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.worker.update_signal.connect(self.main_window.update_joint_display)
        self.timer_interval = 1000
        #self.thread.start()

        #self.ml = Mercury('/dev/left_robot')
        #self.mr = Mercury('/dev/right_robot')
    
    def get_joint_angles(self):
       #left = self.ml.get_angles()
       #right = self.mr.get_angles()
       #return left, right
       
        return self.worker.current_left, self.worker.current_right
    
    def update_joints(self):
        left_angles, right_angles = self.get_joint_angles()
        self.main_window.update_joint_display(left_angles,right_angles)

    

    def stop_after_update(self):
        self.should_stop =  True
    
    
    def start(self):
        if not self.thread.isRunning():
            self.thread.start()
        QTimer.singleShot(0, lambda:self.worker.timer.start(self.timer_interval))
       
        
    
    def stop(self):
        if self.worker.timer.isActive():
            self.worker.timer.stop()
            self.update_joints()


    def arm_2_angle(self,arm_obj,id:int,degree:float,speed:int=10) -> None:
        cur_angle = arm_obj.get_angle(id)
        arm_to_angle = cur_angle + degree

        if(id > 7 and id <= 0):
            raise ValueError(f"Err: '无法控制{id}关节，因为没有这个关节'")
        
        elif id == 1 and (arm_to_angle < -165 or arm_to_angle > 165):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")
        
        elif id == 2 and (arm_to_angle < -55 or arm_to_angle > 95):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")

        elif id == 3 and (arm_to_angle < -180 or arm_to_angle > 5):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")
        
        elif id == 4 and (arm_to_angle < -165 or arm_to_angle > 165):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")

        elif id == 5 and (arm_to_angle < -20 or arm_to_angle > 273):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")
        
        elif id == 6 and (arm_to_angle < -180 or arm_to_angle > 180):
            raise ValueError(f"Err: '无法控制{id}关节，关节无法达到该角度'")
        
        else:
            arm_obj.send_angle(id,arm_to_angle,speed)
            print(f"控制成功，{id}当前关节的角度：{arm_to_angle}")
    
    def return_to_zero_position(self):
        #当连接真实机器人后
        #for i in range(1,7):
        #    self.arm_2_angle(self.ml,i,0,speed=10)
        #    self.arm_2_angle(self.mr,i,0,speed=10)

        self.worker.current_left = [0.0] * 6
        self.worker.current_right = [0.0] * 6
        return self.worker.current_left.copy(),self.worker.current_right.copy()