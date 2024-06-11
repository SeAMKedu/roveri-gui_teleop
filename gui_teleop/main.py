"""
Small GUI application for driving the TurtleBot 4 and showing the video
capture from its onboard OAK-D camera.

"""
import os
import time
from threading import Thread
from typing import Tuple

import customtkinter as ctk
import cv2
from cv_bridge import CvBridge
from PIL import Image
# ROS 2
import rclpy
from rclpy.duration import Infinite
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

RESOURCE_DIR = os.path.join(os.getcwd(), "src/gui_teleop/resource")

MOVE_FORWARD_LEFT = (1, 0, 0, 1)
MOVE_FORWARD = (1, 0, 0, 0)
MOVE_FORWARD_RIGHT = (1, 0, 0, -1)
ROTATE_LEFT = (0, 0, 0, 1)
MOVE_STOP = (0, 0, 0, 0)
ROTATE_RIGHT = (0, 0, 0, -1)
MOVE_BACKWARD_LEFT = (-1, 0, 0, -1)
MOVE_BACKWARD = (-1, 0, 0, 0)
MOVE_BACKWARD_RIGHT = (-1, 0, 0, 1)

PUBLISH_DELAY = 0.5
ANGULAR_SPEEDS = [str(round(i * 0.1, 1)) for i in range(1, 6)]
LINEAR_SPEEDS = [str(round(i * 0.1, 1)) for i in range(1, 11)]


class MessagePublisher(Node):
    """Publish message to the ROS 2 topic."""
    def __init__(self):
        super().__init__(node_name="msgpub")

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)


class ButtonsFrame(ctk.CTkFrame):
    """Frame that contains buttons for sending move commands to the TurtleBot."""
    def __init__(self, master: ctk.CTk, node: MessagePublisher):
        super().__init__(master)

        self.node = node
        self.angular_speed = 0.3
        self.linear_speed = 0.5
        self.batt_state = 0.0

        self.row = 1
        self.col = 0
        self.publish_message = False

        self.label3 = ctk.CTkLabel(self, text=f"Battery State: {self.batt_state} %")
        self.label3.grid(row=0, column=0, padx=(5, 5), pady=(5, 5), columnspan=3)

        # Define a method (and its arguments) to be called when the mouse
        # button is clicked.
        self.on_mouse_click = [
            lambda event: self.send_move_command(event, MOVE_FORWARD_LEFT),
            lambda event: self.send_move_command(event, MOVE_FORWARD),
            lambda event: self.send_move_command(event, MOVE_FORWARD_RIGHT),
            lambda event: self.send_move_command(event, ROTATE_LEFT),
            lambda event: self.send_move_command(event, MOVE_STOP),
            lambda event: self.send_move_command(event, ROTATE_RIGHT),
            lambda event: self.send_move_command(event, MOVE_BACKWARD_LEFT),
            lambda event: self.send_move_command(event, MOVE_BACKWARD),
            lambda event: self.send_move_command(event, MOVE_BACKWARD_RIGHT),
        ]

        # Create 9 buttons with an arrow image inside.
        for i in range(9):
            # Read the image file from the "resource" folder.
            image = Image.open(os.path.join(RESOURCE_DIR, f"arrow{i}.png"))
            arrow = ctk.CTkImage(dark_image=image, size=(46, 46))
            # Create a button.
            button = ctk.CTkButton(self, width=50, height=50, text="", image=arrow)
            if self.row == 0:
                button.grid(row=self.row, column=self.col, padx=(5, 5), pady=(50, 5))
            else:
                button.grid(row=self.row, column=self.col, padx=(5, 5), pady=(5, 5))
            # Bind the button to the left mouse button.
            button.bind("<ButtonPress-1>", self.on_mouse_click[i])
            button.bind("<ButtonRelease-1>", self.on_mouse_release)
            self.col += 1
            if self.col == 3:
                self.row += 1
                self.col = 0
        
        self.label1 = ctk.CTkLabel(self, text="Angular Speed")
        self.label1.grid(row=11, column=0, padx=(5, 5), pady=(10, 0), columnspan=3)
        self.menu1 = ctk.CTkOptionMenu(self, values=ANGULAR_SPEEDS, command=self.set_angular_speed)
        self.menu1.grid(row=12, column=0, padx=(5, 5), pady=(0, 5), columnspan=3)
        self.menu1.set(str(self.angular_speed))

        self.label2 = ctk.CTkLabel(self, text="Linear Speed")
        self.label2.grid(row=13, column=0, padx=(5, 5), pady=(5, 0), columnspan=3)
        self.menu2 = ctk.CTkOptionMenu(self, values=LINEAR_SPEEDS, command=self.set_linear_speed)
        self.menu2.grid(row=14, column=0, padx=(5, 5), pady=(0, 10), columnspan=3)
        self.menu2.set(str(self.linear_speed))

    def publish_twist_message(self, target: Tuple[int, int, int, int]):
        """Send a Twist message to the TurtleBot."""
        twist_msg = Twist()
        twist_msg.linear.x = target[0] * self.linear_speed
        twist_msg.linear.y = target[1] * self.linear_speed
        twist_msg.linear.z = target[2] * self.linear_speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = target[3] * self.angular_speed
        while self.publish_message:
            self.node.pub.publish(twist_msg)
            time.sleep(PUBLISH_DELAY)

    def send_move_command(self, event, target):
        """Called when the left mouse button is clicked."""
        self.publish_message = True
        publish_thread = Thread(target=self.publish_twist_message, args=(target,))
        publish_thread.start()
    
    def on_mouse_release(self, event):
        """Called when the left mouse button is released."""
        self.publish_message = False

    def set_angular_speed(self, choice: str):
        """Called when the option is selected from the option list."""
        self.angular_speed = float(choice)

    def set_linear_speed(self, choice: str):
        """Called when the option is selected from the option list."""
        self.linear_speed = float(choice)

    def show_batt_state(self, msg: BatteryState):
        """Called when new battery state is published."""
        self.batt_state = round(100 * msg.percentage, 1)
        self.label3.configure(text=f'Battery State: {self.batt_state} %')


class VideoCaptureFrame(ctk.CTkFrame):
    """Frame for showing the video capture from the OAK-D camera."""
    def __init__(self, master: ctk.CTk):
        super().__init__(master)

        self.cvbridge = CvBridge()

        self.videocap = ctk.CTkLabel(self, text="")
        self.videocap.grid(row=0, column=0)

    def show_video_capture(self, imgmsg: ImageMsg):
        """Show the video capture on the GUI."""
        image_cv2 = self.cvbridge.imgmsg_to_cv2(imgmsg, imgmsg.encoding)
        image_rgb = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_rgb)
        image_ctk = ctk.CTkImage(dark_image=image_pil, size=(300, 300))
        self.videocap.configure(image=image_ctk)
        self.videocap.image = image_ctk


class DockingFrame(ctk.CTkFrame):
    """Frame for controlling the rover's dock/undock actions."""
    def __init__(self, master: ctk.CTk):
        super().__init__(master)

        self.navigator = TurtleBot4Navigator()

        self.button1 = ctk.CTkButton(self, text="Dock", command=self.dock_action)
        self.button1.grid(row=0, column=0, padx=(5, 5), pady=(5, 5))

        self.button2 = ctk.CTkButton(self, text="Undock", command=self.undock_action)
        self.button2.grid(row=0, column=1, padx=(5, 5), pady=(5, 5))

    def dock_action(self):
        """Initiate the docking action."""
        if not self.navigator.getDockedStatus():
            self.navigator.dock()

    def undock_action(self):
        """Initiate the undocking action."""
        if self.navigator.getDockedStatus():
            self.navigator.undock()


class Application(ctk.CTk):
    """GUI application."""
    def __init__(self, node: MessagePublisher):
        super().__init__()

        self.node = node

        self.title("GUI Teleop for TurtleBot 4")
        self.geometry(f"{600}x{380}")

        self.btnframe = ButtonsFrame(self, self.node)
        self.btnframe.grid(row=0, column=0, padx=5, pady=5, sticky="nsew", rowspan=2)

        self.capframe = VideoCaptureFrame(self)
        self.capframe.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        self.docframe = DockingFrame(self)
        self.docframe.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")


class MessageSubscriber(Node):
    """Subsribes messages from ROS2 topics."""
    def __init__(self, app: Application):
        super().__init__(node_name="msgsub")

        # Quality of service profile for the image messages.
        self.image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Infinite,
            deadline=Infinite,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Infinite,
        )
        # Subscribe to the OAK-D camera data topic.
        self.sub1 = self.create_subscription(
            msg_type=ImageMsg,
            topic="oakd/rgb/preview/image_raw",
            callback=app.capframe.show_video_capture,
            qos_profile=self.image_qos_profile
        )
        # Subscribe to the battery state topic.
        self.sub2 = self.create_subscription(
            msg_type=BatteryState,
            topic="battery_state",
            callback=app.btnframe.show_batt_state,
            qos_profile=qos_profile_sensor_data
        )


def main(args=None):
    """Run GUI application and ROS 2 nodes."""
    rclpy.init(args=args)
    pub = MessagePublisher()
    app = Application(pub)
    sub = MessageSubscriber(app)
    thr = Thread(target=rclpy.spin, args=(sub,))
    thr.start()
    app.mainloop() # GUI app must be run in the main thread
    pub.destroy_node()
    sub.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
