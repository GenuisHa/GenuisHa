#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk
import time


class UcarCommander(object):
    def __init__(self, root):
        rospy.init_node("button_node")
        self.start_publisher = rospy.Publisher(
            "/nav_start", Bool, queue_size=1)
        self.root = root
        self.root.title("UcarCommander")
        self.root.geometry("400x200")

        # create user interface
        self.create_gui()

        self.is_nav_start = False

        self.root.mainloop()

    def create_gui(self):
        self.big_timer_started = False
        self.big_timer_label = ttk.Label(
            self.root, text="00:00:00", font=("Arial", 64))
        self.big_timer_label.place(relx=0.5, rely=0.2, anchor="center")
        # button to start the navigation
        self.draw_button = ttk.Button(
            self.root, text="Draw", command=self.draw_button_clicked)
        self.draw_button.place(relx=0.4, rely=0.45)
        self.stop_button = ttk.Button(
            self.root, text="Stop", command=self.stop_button_clicked)
        self.stop_button.place(relx=0.4, rely=0.6)
        self.error_label = ttk.Label(
            self.root, text="", style="ErrorLabel.TLabel")
        self.error_label.place(relx=0.4, rely=0.75)

    # Link Start!!!
    def draw_button_clicked(self):
        if not self.is_nav_start:
            print("Action Stations!!!")
            self.error_label.config(text="Action Stations!!!")
            self.is_nav_start = True
            data = Bool()
            data.data = True
            self.start_publisher.publish(data)
            self.start_big_timer()
        else:
            print("Starting failed: already running")
            self.error_label.config(
                text="Starting failed: already running")

    def stop_button_clicked(self):
        # 处理Stop按钮点击事件
        if self.big_timer_started:
            self.is_nav_start = False
            data = Bool()
            data.data = True
            self.start_publisher.publish(data)
            self.stop_big_timer()
        else:
            self.error_label.config(text="No group is drawing")

    def start_big_timer(self):
        # 启动大计时器
        self.big_timer_started = True
        self.big_timer_start_time = time.time()
        self.root.update()
        self.run()

    def big_timer_running(self):
        elapsed_time = time.time() - self.big_timer_start_time
        formatted_time = time.strftime(
            "%H:%M:%S", time.gmtime(elapsed_time*60))
        self.big_timer_label.config(text=formatted_time)
        self.root.update()

    def stop_big_timer(self):
        # 停止大计时器
        self.big_timer_started = False
        self.big_timer_label.config(text="00:00:00")
        self.error_label.config(text="Stopped!!!")
        self.root.update()

    # You wonder what this thing is? Are you fucking teasing me?
    def run(self):
        while self.big_timer_started:
            self.big_timer_running()


# If you need this note, I will drag your head down and kick it as a ball.
def main():
    root = tk.Tk()
    node = UcarCommander(root)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
