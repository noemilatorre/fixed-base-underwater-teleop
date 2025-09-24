#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QPushButton
from std_msgs.msg import String
from lai_msgs.msg import PickAndPlaceGoal  

class RobotActionGUI(QWidget):
    def __init__(self):
        super().__init__()

        rospy.init_node('franka_action_gui', anonymous=True)
        
        self.latest_goal = None

        # Publisher per Pick&Place (modalità auto)
        self.goal_pub = rospy.Publisher("/gui_goal_msg", PickAndPlaceGoal, queue_size=1)

        # Publisher per la modalità di controllo (manual / auto / follow)
        self.control_mode_pub = rospy.Publisher("/control_mode", String, queue_size=1)

        # Subscriber per ricevere i goal filtrati
        rospy.Subscriber("filtered_obj_position", PickAndPlaceGoal, self.filtered_obj_callback)

        # Setup GUI
        self.setWindowTitle("Franka Action GUI")
        self.setGeometry(100, 100, 350, 120)
        layout = QVBoxLayout()

        self.combo = QComboBox()
        self.combo.addItems([
            "Rotate valve",
            "Pick and Place Detected Cube",
            "Follow moving cube",
            "Insert connector",
            "Place and activate sensor",
            "Retrieve sensor",
            "Remove debris",
            "Return to base",
            "Inspect area",
            "Capture image",
            "Map environment",
            "Hold position",
        ])
        layout.addWidget(self.combo)

        self.button = QPushButton("Execute Operation")
        self.button.clicked.connect(self.execute_action)
        layout.addWidget(self.button)

        self.setLayout(layout)

    def filtered_obj_callback(self, msg):
        self.latest_goal = msg
        rospy.loginfo(
            f"Filtered goal received: pick=({msg.pick_position.x:.3f}, "
            f"{msg.pick_position.y:.3f}, {msg.pick_position.z:.3f})"
        )

    def execute_action(self):
        operation = self.combo.currentText()

        if operation == "Pick and Place Detected Cube":
            if self.latest_goal is None:
                rospy.logwarn("No goal available from topic filtered_obj_position.")
                return
            rospy.loginfo("Publishing goal on /gui_goal_msg for auto mode...")
            self.control_mode_pub.publish(String("auto"))  # Cambio modalità in auto
            self.goal_pub.publish(self.latest_goal)

        elif operation == "Follow moving cube":
            self.control_mode_pub.publish(String("follow"))
            rospy.loginfo("FOLLOW mode...")

        else:
            rospy.logwarn(f"Operation '{operation}' not implemented yet.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RobotActionGUI()
    gui.show()
    sys.exit(app.exec_())
