#!/usr/bin/env python3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import csv
import os

ros2bag_path = "../bag_files/teleop/centerline_drive_0502_1050"
db_path = os.path.join(ros2bag_path, "centerline_drive_0502_1050_0.db3")
output_path = "../bag_files/teleop/extracted_data/centerline_drive_data_0502_1050.csv"
topic = "/odom"

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get the message type for the topic
cursor.execute("SELECT type FROM topics WHERE name=?", (topic,))
msg_type_str = cursor.fetchone()[0]
msg_type = get_message(msg_type_str)

cursor.execute(
    "SELECT data FROM messages WHERE topic_id = (SELECT id FROM topics WHERE name = ?)",
    (topic,),
)
rows = cursor.fetchall()

with open(output_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    for row in rows:
        msg = deserialize_message(row[0], msg_type)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        writer.writerow([x, y])

conn.close()