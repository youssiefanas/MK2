import tkinter as tk
from geometry_msgs.msg import Twist, Vector3, Pose
import rospy
from std_msgs.msg import Int16, Bool
from spatialmath import SE3
from roboticstoolbox import RevoluteDH, DHRobot
import numpy as np
import matplotlib.pyplot as plt

deg = 180/np.pi
# stepper mapping
# the full revolution of the stepper is 200 steps
# the range of motion of the shoulder is from 110 to 40 degrees (theta_2)
# the range of motion of the elbow is from 90 to 30 degrees (theta_3)
# the conversion between the stepper step and the joint angle is:
min_base= -50
max_base= 50
home_base=0 
#minimum value for shoulder stepper is 0 (110 is mapped to 0)
#maximum value for shoulder stepper is (40-110)*200/360= 111
#home value for shoulder stepper is (90-110)*200/360= 11
min_shoulder=  0
max_shoulder= 111
home_shoulder= 11
#minimum value for elbow stepper is 0 (30 is mapped to 0)
#maximum value for elbow stepper is (90-30)*200/360= 33
min_elbow= 0
max_elbow= 33
home_elbow=0
# gripper mapping
max_gripper=180
min_gripper=0

# joints limits
j1_max = np.pi/2
j2_max = np.pi*(110/180)
j3_max = (-90*np.pi/180)+(60*np.pi/180)

j1_min = -np.pi/2
j2_min = np.pi*40/180
j3_min = -np.pi*(90/180)

j1_home=0
j2_home=np.pi/2
j3_home= -np.pi*(90/180)
home_position_joints=np.array([j1_home,j2_home,j3_home])


class MK2(DHRobot):
    def __init__(self):
        L0 = RevoluteDH(d=10, a=11, alpha=np.pi / 2, qlim=[j1_min, j1_max])
        L1 = RevoluteDH(d=0, a=19, alpha=0, qlim=[j2_min, j2_max])
        L2 = RevoluteDH(d=0, a=22, alpha=0, qlim=[j3_min, j3_max])
        super().__init__([L0, L1, L2], name="MK2", manufacturer="E-JUST")
        self.home = home_position_joints
        self.qz = np.zeros(3)
        self.min = np.array([j1_min, j2_min, j3_min])
        self.max = np.array([j1_max, j2_max, j3_max])
        self.addconfiguration("home", self.home)
        self.addconfiguration("qz", self.qz)


class MK2GUI:
    def __init__(self):
        self.master = tk.Tk()
        self.master.title("MK2 Arm")
        self.master.geometry("900x600")
        self.master.protocol("WM_DELETE_WINDOW", self.on_close_window)
        self.mk2 = MK2()

    
    def stepper_node_init(self):
        rospy.init_node("stepper_node", anonymous=True)
        self.stepper_readings = Vector3()
        # publish the stepper_base, stepper_shoulder, stepper_elbow to the topic with msg type Vector3
        self.stepper_joints_pub = rospy.Publisher("/stepper_joints", Vector3, queue_size=10)
        self.stepper_gripper = rospy.Publisher("/stepper_gripper", Int16, queue_size=10)

    def init_frames(self):
        frame = tk.Frame(self.master)
        frame.pack()

        # labels
        x_label = tk.Label(frame, text="x")
        y_label = tk.Label(frame, text="y")
        z_label = tk.Label(frame, text="z")

        # create three entry boxes
        self.x_entry = tk.Entry(frame)
        self.y_entry = tk.Entry(frame)
        self.z_entry = tk.Entry(frame)

        # create a button
        calculate_button = tk.Button(frame, text="calculate", command=self.calculate)

        # create clear button to clear the entry boxes and labels
        clear_button = tk.Button(frame, text="clear", command=self.clear)

        # create a slider to control each stepper
        self.base_slider = tk.Scale(
            frame,
            from_=min_base,
            to=max_base,
            orient=tk.HORIZONTAL,
            command=self.base_slider_update,
        )
        self.shoulder_slider = tk.Scale(
            frame,
            from_=min_shoulder,
            to=max_shoulder,
            orient=tk.HORIZONTAL,
            command=self.shoulder_slider_update,
        )
        self.elbow_slider = tk.Scale(
            frame,
            from_=min_elbow,
            to=max_elbow,
            orient=tk.HORIZONTAL,
            command=self.elbow_slider_update,
        )

        # create move button to move the arm to the desired position
        gripper_close_button = tk.Button(
            frame, text="gripper close", command=self.gripper_close
        )
        # create button for gripper release
        gripper_release_button = tk.Button(
            frame, text="gripper release", command=self.gripper_release
        )
        # create home button to move the arm to the home position
        home_button = tk.Button(frame, text="home", command=self.home)

        # sliders for the stepper
        base_slider_label = tk.Label(frame, text="base")
        shoulder_slider_label = tk.Label(frame, text="shoulder")
        elbow_slider_label = tk.Label(frame, text="elbow")

        # create three labels to display the angles
        self.base_label = tk.Label(frame, text="base = ")
        self.shoulder_label = tk.Label(frame, text="shoulder = ")
        self.elbow_label = tk.Label(frame, text="elbow = ")

        # create three labels to display the stepper angles
        self.stepper_base_label = tk.Label(frame, text="stepper_base = ")
        self.stepper_shoulder_label = tk.Label(frame, text="stepper_shoulder = ")
        self.stepper_elbow_label = tk.Label(frame, text="stepper_elbow = ")

        # general error message
        self.error_label = tk.Label(frame, text="error")

        # place the labels and entry boxes
        x_label.grid(row=0, column=0)
        y_label.grid(row=1, column=0)
        z_label.grid(row=2, column=0)
        self.x_entry.grid(row=0, column=1)
        self.y_entry.grid(row=1, column=1)
        self.z_entry.grid(row=2, column=1)
        calculate_button.grid(row=3, column=1)
        clear_button.grid(row=3, column=2)
        # -----------------
        home_button.grid(row=3, column=3)
        gripper_close_button.grid(row=3, column=4)
        gripper_release_button.grid(row=3, column=5)
        # move_button.grid(row = 3, column = 6)

        # sliders for steppers
        self.base_slider.grid(row=4, column=1)
        self.shoulder_slider.grid(row=5, column=1)
        self.elbow_slider.grid(row=6, column=1)

        self.base_label.grid(row=4, column=0)
        self.shoulder_label.grid(row=5, column=0)
        self.elbow_label.grid(row=6, column=0)
        self.stepper_base_label.grid(row=7, column=0)
        self.stepper_shoulder_label.grid(row=8, column=0)
        self.stepper_elbow_label.grid(row=9, column=0)
        self.error_label.grid(row=10, column=0)

    def home(self):
        self.mk2.plot(home_position_joints, dt=1)
        self.stepper_readings = Vector3()
        self.stepper_readings.x = home_base
        self.stepper_readings.y = home_shoulder
        self.stepper_readings.z = home_elbow
        self.stepper_joints_pub.publish(self.stepper_readings)
        

    def __call__(self) -> None:
        self.stepper_node_init()
        self.init_frames()
        self.master.mainloop()



    def gripper_close(self):
        stepper_gripper_msg = Int16()
        stepper_gripper_msg.data = 50
        # publish the seevo_gripper to the topic
        stepper_gripper = rospy.Publisher("/stepper_gripper", Int16, queue_size=10)
        stepper_gripper.publish(stepper_gripper_msg.data)

    def gripper_release(self):
        stepper_gripper_msg = Int16()
        stepper_gripper_msg.data = 90
        # publish the seevo_gripper to the topic
        stepper_gripper = rospy.Publisher("/stepper_gripper", Int16, queue_size=10)
        stepper_gripper.publish(stepper_gripper_msg.data)

    def base_slider_update(self, val):
        stepper_base = val
        self.stepper_base_label.config(text="stepper_base = " + str(stepper_base))
        self.mover_slider()

    def shoulder_slider_update(self, val):
        stepper_shoulder = val
        self.stepper_shoulder_label.config(text="stepper_shoulder = " + str(stepper_shoulder))
        self.mover_slider()

    def elbow_slider_update(self, val):
        stepper_elbow = val
        self.stepper_elbow_label.config(text="stepper_elbow = " + str(stepper_elbow))
        self.mover_slider()

    def mover_slider(self):
        stepper_base = self.base_slider.get()
        stepper_shoulder = self.shoulder_slider.get()
        stepper_elbow = self.elbow_slider.get()

        self.stepper_readings.x = stepper_base
        self.stepper_readings.y = stepper_shoulder
        self.stepper_readings.z = stepper_elbow
        # publish the stepper_base, stepper_shoulder, stepper_elbow to the topic with msg type Vector3
        self.stepper_joints_pub.publish(self.stepper_readings)

    def clear(self):
        self.x_entry.delete(0, "end")
        self.y_entry.delete(0, "end")
        self.z_entry.delete(0, "end")
        self.error_label.config(text="error message: ")
        self.base_label.config(text="stepper_base = ")
        self.shoulder_label.config(text="stepper_shoulder =")
        self.elbow_label.config(text="stepper_elbow = ")
        self.stepper_base_label.config(text="stepper_base = ")
        self.stepper_shoulder_label.config(text="stepper_shoulder =")
        self.stepper_elbow_label.config(text="stepper_elbow = ")

    def calculate(self):
        x = float(self.x_entry.get())
        y = float(self.y_entry.get())
        z = float(self.z_entry.get())
        # make sure that the values are in the range
        # it is not allowed to have a value out of the range
        self.error_label.config(text="error message: ")
        while x < 0 or x > 47:
            self.error_label.config(text="error: x is out of range")
            # clear the entry
            # enter again
            return

        while y < -47 or y > 47:
            self.error_label.config(text="error: y is out of range")
            return
        while z < 5 or z > 49:
            self.error_label.config(text="error: z is out of range")
            return

        g = SE3(x, y, z)
        print(g)

        # make the inverse kinematics of the robot using ik_lm_chan function
        # s=mk2.ik_lm_chan(g, q0=mk2.home, ilimit=1000, tol=1e-6, we=[1, 1, 1, 0, 0, 0])
        # joints=s[0]
        s = self.mk2.ikine_LM(g, q0=self.mk2.home)
        joints = s.q
        print(joints)
        # mk2.plot(joints,dt=1)
        self.mk2.plot(joints, dt=1)
        base_joint = joints[0]
        shoulder_joint = joints[1]
        elbow_joint = joints[2]
        # check the range of the joints
        print(joints)
        while base_joint < j1_min or base_joint > j1_max:
            print("sq0: ", base_joint)
            print("j1_min = ", j1_min, "j1_max = ", j1_max)
            self.error_label.config(text="error: base is out of range")
            return
        while shoulder_joint < j2_min or shoulder_joint > j2_max:
            print("sq1: ", shoulder_joint)
            print("j2_min = ", j2_min, "j2_max = ", j2_max)
            self.error_label.config(text="error: shoulder is out of range")
            return
        while elbow_joint < j3_min or elbow_joint > j3_max:
            self.error_label.config(text="error: elbow is out of range")
            return

        self.base_label.config(text="base = " + str(base_joint*deg))
        self.shoulder_label.config(text="shoulder = " + str(shoulder_joint*deg))
        self.elbow_label.config(text="elbow = " + str(elbow_joint*deg))

        stepper_base = self.map_range(base_joint, j1_min, j1_max, min_base, max_base)
        # check the range of the stepper_base
        # while stepper_base < min_base or stepper_base > max_base:
     
        #     self.error_label.config(text="error: stepper_base is out of range"+ str(stepper_base))
        #     return
        if stepper_base > max_base:
            stepper_base = max_base
        if stepper_base < min_base:
            stepper_base = min_base


        stepper_shoulder = self.map_range(
            shoulder_joint,
            j2_min,
            j2_max,
            min_shoulder,
            max_shoulder,
        )
        # while stepper_shoulder < min_shoulder or stepper_shoulder > max_shoulder:
        #     self.error_label.config(text="error: stepper_shoulder is out of range")
        #     return
        if stepper_shoulder > max_base:
            stepper_shoulder = max_base
        if stepper_shoulder < min_base:
            stepper_shoulder = min_base

        stepper_elbow = self.map_range(elbow_joint, j3_min, j3_max, min_elbow, max_elbow)
        # while stepper_elbow < min_elbow or stepper_elbow > max_elbow:
        #     self.error_label.config(text="error: stepper_elbow is out of range"+ str(stepper_elbow))
        #     return
        if stepper_elbow > max_base:
            stepper_elbow = max_base
        if stepper_elbow < min_base:
            stepper_elbow = min_base

        self.stepper_base_label.config(text="stepper_base = " + str(stepper_base))
        self.stepper_shoulder_label.config(text="stepper_shoulder = " + str(stepper_shoulder))
        self.stepper_elbow_label.config(text="stepper_elbow = " + str(stepper_elbow))

        self.stepper_readings.x = stepper_base
        self.stepper_readings.y = stepper_shoulder
        self.stepper_readings.z = stepper_elbow
        # publish the stepper_base, stepper_shoulder, stepper_elbow to the topic with msg type Vector3
        self.stepper_joints_pub.publish(self.stepper_readings)


    def on_close_window(self):
        plt.close()
        self.master.destroy()

    @staticmethod
    def map_range(x, old_min, old_max, new_min, new_max):
        """_summary_

        Args:
            x (_type_): _description_
            old_min (_type_): _description_
            old_max (_type_): _description_
            new_min (_type_): _description_
            new_max (_type_): _description_

        Returns:
            _type_: _description_
        """
        old_range = old_max - old_min
        new_range = new_max - new_min
        return (((x - old_min) * new_range) / old_range) + new_min


if __name__ == "__main__":
    arm = MK2()
    print(arm)
    gui = MK2GUI()
    gui()
