#!/usr/bin/env python3
import rospy
from ros_ultralytics.msg import obj_info, center as center_msg, bounding
import math

class FakeObjPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/obj_info", obj_info, queue_size=10)
        rospy.loginfo("FakeObjPublisher node started, publishing fake object data on obj_info...")

        self.rate = rospy.Rate(10)  # 10 Hz per movimento fluido
        self.t = 0.0  # tempo simulato
        self.dt = 0.05  # passo di integrazione
        self.radius = 0.2  # raggio del semicerchio
        self.center_x = 0.5  # centro del semicerchio in X
        self.center_y = 0.0  # centro del semicerchio in Y
        self.cz = 0.3        # altezza fissa del cubo
        self.speed = 0.5     # velocità angolare del movimento (rad/s)

    def generate_fake_data(self):
        msg_obj = obj_info()

        # Calcolo posizione sul semicerchio (seno per Y, X fisso + offset circolare)
        cx = self.center_x + self.radius * math.cos(self.t * self.speed)  # avanti-indietro
        cy = self.center_y + self.radius * math.sin(self.t * self.speed)  # lato a lato
        cz = self.cz

        # Bounding box del cubo
        size = 0.03
        x1, y1, z1 = cx - size/2, cy - size/2, cz - size/2
        x2, y2, z2 = cx + size/2, cy + size/2, cz + size/2

        # Messaggi
        center = center_msg(cx=cx, cy=cy, cz=cz)
        bbox = bounding()
        bbox.x1, bbox.y1, bbox.z1 = x1, y1, z1
        bbox.x2, bbox.y2, bbox.z2 = x2, y2, z2

        msg_obj.centers.append(center)
        msg_obj.bboxs.append(bbox)
        msg_obj.labels.append("cube_0")
        msg_obj.confidences.append(1.0)
        msg_obj.colors.append("GREEN")
        msg_obj.classes.append("cube_green")

        self.pub.publish(msg_obj)
        rospy.loginfo(f"[FakeObj] cube_0: center=({cx:.2f}, {cy:.2f}, {cz:.2f})")

        self.t += self.dt  # incremento tempo

    def run(self):
        while not rospy.is_shutdown():
            self.generate_fake_data()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fake_obj_publisher")
    fake_publisher = FakeObjPublisher()
    fake_publisher.run()
