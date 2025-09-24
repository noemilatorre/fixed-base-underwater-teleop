#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from ros_ultralytics.msg import obj_info
from collections import deque
import math

class ObjVisualizer:
    def __init__(self, delay_sec=0.2):
        rospy.init_node("obj_marker_delay")

        # Subscriber e Publisher
        self.sub = rospy.Subscriber("obj_info", obj_info, self.callback)
        self.pub = rospy.Publisher("/marker_topic", Marker, queue_size=10)

        # Colori per label
        # self.label_colors = {
        #     'green': (0.0, 1.0, 0.0),
        #     'red': (1.0, 0.0, 0.0),
        #     'blue': (0.0, 0.0, 1.0),
        #     'yellow': (1.0, 1.0, 0.0),
        # }

        # SOLO CUBI VERDI - sempre attivo
        self.filter_green_only = True
        
        # Colore per i cubi verdi
        self.green_color = (0.0, 1.0, 0.0)  # RGB per verde


        # Buffer per il ritardo
        self.delay_sec = rospy.get_param("~robot_delay", delay_sec)
        self.buffer = deque()

        # Per tracciare quali marker sono attualmente visibili
        self.last_ids = set()

        rospy.loginfo("ObjVisualizer node started")
        rospy.loginfo(f"Publishing markers with {self.delay_sec}s delay...")

        # Timer per pubblicare i messaggi bufferizzati
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 50Hz

   
    def callback(self, msg):
        # Aggiungi il messaggio al buffer con timestamp corrente
        timestamp = rospy.Time.now()
        self.buffer.append((timestamp, msg))

        # Conta solo i cubi verdi per il log
        green_count = sum(1 for i in range(min(len(msg.colors), len(msg.centers))) 
                      if msg.colors[i] == "GREEN") 
        rospy.loginfo(f"[CALLBACK] {green_count} cubi verdi rilevati, buffer size: {len(self.buffer)}")

    def timer_callback(self, event):
        now = rospy.Time.now()
        while self.buffer and (now - self.buffer[0][0]).to_sec() >= self.delay_sec:
            _, msg_to_publish = self.buffer.popleft()
            self.publish_markers(msg_to_publish)

    def publish_markers(self, msg):
        current_ids = set()
        published_count = 0

        for i, c in enumerate(msg.centers):
            if i >= len(msg.colors) or msg.colors[i] != "GREEN":
                continue

            cx, cy, cz = c.cx, c.cy, c.cz
            label = msg.labels[i]
            color_name = msg.colors[i] if i < len(msg.colors) else "unknown"

            marker_id = i  # puoi usare un id più robusto se implementi tracking
            current_ids.add(marker_id)

            self.publish_marker(label, cx, cy, cz, color_name, marker_id)

        # Rimuovi marker che non compaiono più
        to_delete = self.last_ids - current_ids
        for marker_id in to_delete:
            self.delete_marker(marker_id)

        # Aggiorna set di marker visibili
        self.last_ids = current_ids

        if published_count > 0:
            rospy.loginfo(f"Pubblicati {published_count} marker verdi")


    def publish_marker(self, label, cx, cy, cz, color_name, marker_id):
        marker = Marker()
        marker.header.frame_id = "fr3_link0"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "green_cube"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = cx
        marker.pose.position.y = cy
        marker.pose.position.z = cz
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # r, g, b = self.label_colors.get(color_name, (0.5, 0.5, 0.5))
        # marker.color.r = r
        # marker.color.g = g
        # marker.color.b = b
        # marker.color.a = 0.8

        # SEMPRE VERDE
        marker.color.r = self.green_color[0]
        marker.color.g = self.green_color[1]
        marker.color.b = self.green_color[2]
        marker.color.a = 0.8  # Trasparenza

        rospy.loginfo(f"→ Marker {label} colore {color_name} @ ({cx:.3f}, {cy:.3f}, {cz:.3f})")
        self.pub.publish(marker)

    def delete_marker(self, marker_id):
        marker = Marker()
        marker.header.frame_id = "fr3_link0"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "green_cube"
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.pub.publish(marker)
        rospy.loginfo(f" Marker {marker_id} eliminato da RViz")

if __name__ == "__main__":
    visualizer = ObjVisualizer(delay_sec=0.0)  #0.1 #0.5    
    rospy.spin()
