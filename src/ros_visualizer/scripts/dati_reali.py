#!/usr/bin/env python3
import rospy
import yaml
import os
from ros_ultralytics.msg import obj_info, center, bounding


class RealDataFakePublisher:
    def __init__(self, log_file):
        rospy.init_node("real_data_fake_publisher")
        self.pub = rospy.Publisher("/obj_info", obj_info, queue_size=10)
        self.rate = rospy.Rate(5)  #  Hz
        self.data_index = 0

        # Carica i dati reali dal file
        self.real_data = self.load_real_data(log_file)
        rospy.loginfo("RealDataFakePublisher started with %d real data points", len(self.real_data))

    def load_real_data(self, log_file):
        """Carica i dati dal log.txt e converte in lista di dict compatibile con obj_info"""
        if not os.path.exists(log_file):
            rospy.logerr("File %s non trovato!", log_file)
            return []

        with open(log_file, "r") as f:
            data_blocks = list(yaml.safe_load_all(f))

        parsed_data = []
        for block in data_blocks:
            if block is None:
                continue
            parsed_data.append({
                "labels": block.get("labels", []),
                "confidences": block.get("confidences", []),
                "centers": [center(**c) for c in block.get("centers", [])],
                "bboxs": [bounding(**b) for b in block.get("bboxs", [])],
                "colors": block.get("colors", []),
                "classes": block.get("classes", []),
            })
        return parsed_data

    def publish_data(self):
        while not rospy.is_shutdown() and self.real_data:
            current_data = self.real_data[self.data_index]

            msg = obj_info()
            msg.labels = current_data["labels"]
            msg.confidences = current_data["confidences"]
            msg.centers = current_data["centers"]
            msg.bboxs = current_data["bboxs"]
            msg.colors = current_data["colors"]
            msg.classes = current_data["classes"]

            self.pub.publish(msg)

            rospy.loginfo("Published data %d: %s @ (%.3f, %.3f, %.3f)",
                         self.data_index, msg.colors[0],
                         msg.centers[0].cx, msg.centers[0].cy, msg.centers[0].cz)

            # Passa al prossimo dato
            self.data_index = (self.data_index + 1) % len(self.real_data)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        log_path = "/home/noemi/franka_ws/src/ros_visualizer/log.txt"
        publisher = RealDataFakePublisher(log_path)
        publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
