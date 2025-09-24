#!/usr/bin/env python3
import rospy
from ros_ultralytics.msg import obj_info, center as center_msg, bounding
from visualization_msgs.msg import Marker
import random

class FakeObjPublisher:
    def __init__(self):
        # Publisher del topic obj_info
        self.pub = rospy.Publisher("/obj_info", obj_info, queue_size=10)
        rospy.loginfo("FakeObjPublisher node started, publishing fake object data on obj_info...")
        
        # Frequenza di pubblicazione
        self.rate = rospy.Rate(0.5)  # 0.5 Hz
        
        # Counter per tenere traccia degli oggetti
        self.obj_id = 0

    def generate_fake_data(self):
        # Crea un messaggio obj_info simulato
        msg_obj = obj_info()
        
        # Simuliamo alcuni oggetti con diverse posizioni
        num_objects = random.randint(1, 2)  # Simuliamo da 1 a 3 oggetti per volta
        
        for _ in range(num_objects):
            # Genera dati casuali per posizione e altre informazioni
            label = f"cube_{self.obj_id}"  # Eseguiamo un contatore sugli oggetti
              # Coordinate realistiche per un tavolo
            cx = random.uniform(0.3, 0.7)    # X: davanti al robot
            cy = random.uniform(-0.3, 0.3)   # Y: lateralmente
            cz = random.uniform(0.3, 0.3)    # Z: ALTEZZA TAVOLO (0.6-0.7m)

            # Simula bounding box 3D come +/- un offset dal centro
            size = 0.03  # 3 cm come nel visualizzatore
            x1, y1, z1 = cx - size/2, cy - size/2, cz - size/2
            x2, y2, z2 = cx + size/2, cy + size/2, cz + size/2

            # Costruisci messaggi
            center = center_msg(cx=cx, cy=cy, cz=cz)

            bbox = bounding()
            bbox.x1 = x1
            bbox.y1 = y1
            bbox.z1 = z1
            bbox.x2 = x2
            bbox.y2 = y2
            bbox.z2 = z2

            msg_obj.centers.append(center)
            msg_obj.bboxs.append(bbox)
            msg_obj.labels.append(label)
            msg_obj.confidences.append(1.0)
            msg_obj.colors.append("GREEN")
            msg_obj.classes.append(f"{label}_green")

            self.obj_id += 1

            rospy.loginfo(f"[FakeObj] {label}: center=({cx:.2f}, {cy:.2f}, {cz:.2f})")

        self.pub.publish(msg_obj)


    def run(self):
        while not rospy.is_shutdown():
            self.generate_fake_data()  # Genera e pubblica dati fake
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fake_obj_publisher")
    fake_publisher = FakeObjPublisher()
    fake_publisher.run()
