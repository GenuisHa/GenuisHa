#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import ttkthemes
import random
import time


class AccumTimerApp(object):
    def __init__(self, root):
        self.root = root
        self.root.title("AccumTimer")
        self.root.geometry("800x600")

        self.group_number = None
        self.big_timer_started = False
        self.end_position = None
        self.results = []
        self.small_timer_started = False
        self.small_timer_start_time = None

        # 创建用户界面
        self.create_gui()

        # 加载CSS主题
        self.load_css_theme()

        # 清空Result.md
        self.file_address = "./config/"
        self.file = open(self.file_address + "Result.md", "w")
        self.file.write("")
        self.file.close()

        self.root.mainloop()

    def create_gui(self):
        # 大计时器标签
        self.big_timer_label = ttk.Label(
            self.root, text="00:00:00", font=("Arial", 64))
        self.big_timer_label.place(relx=0.5, rely=0.2, anchor="center")

        # 输入组号的标签、输入框、Draw按钮、错误提示标签
        self.group_entry_label = ttk.Label(
            self.root, text="Enter Group Number (1-20):", font=("Arial", 10))
        self.group_entry_label.place(relx=0.2, rely=0.35)
        self.group_entry = ttk.Entry(self.root)
        self.group_entry.place(relx=0.41, rely=0.342)
        self.draw_button = ttk.Button(
            self.root, text="Draw", command=self.draw_button_clicked)
        self.draw_button.place(relx=0.6, rely=0.342)
        self.stop_button = ttk.Button(
            self.root, text="Stop", command=self.stop_button_clicked)
        self.stop_button.place(relx=0.6, rely=0.4)
        self.group_error_label = ttk.Label(
            self.root, text="", style="ErrorLabel.TLabel")
        self.group_error_label.place(relx=0.72, rely=0.35)

        # 显示End位置的标签
        self.end_label = ttk.Label(self.root, text="", font=("Arial", 20))
        self.end_label.place(relx=0.5, rely=0.45, anchor="center")

        # 结果列表相关小组件
        self.result_label = ttk.Label(
            self.root, text="Results:", font=("Arial", 20))
        self.result_label.place(relx=0.1, rely=0.5)
        self.result_listbox = tk.Listbox(self.root, width=20, height=10)
        self.result_listbox.place(relx=0.1, rely=0.55)
        self.confirm_button = ttk.Button(
            self.root, text="Confirm the result", command=self.confirm_button_clicked)
        self.confirm_button.place(relx=0.3, rely=0.65)
        self.confirm_error_label = ttk.Label(
            self.root, text="", style="ErrorLabel.TLabel")
        self.confirm_error_label.place(relx=0.3, rely=0.72)

        # 小计时器标签
        self.small_timer_label = ttk.Label(
            self.root, text="00:00:00", font=("Arial", 24))
        self.small_timer_label.place(relx=0.8, rely=0.9)

    def load_css_theme(self):
        # 使用ttkthemes库加载主题
        style = ttkthemes.ThemedStyle(self.root)
        style.set_theme("yaru")

    def draw_button_clicked(self):
        # 处理Draw按钮点击事件
        group_number = self.group_entry.get()
        if group_number.isdigit() and 1 <= int(group_number) <= 20:
            self.group_number = f"D{int(group_number):02}"
            self.group_error_label.config(text="")
            self.end = random.randint(0, 2)
            self.end_position = ["End 1", "End 2", "End 3"][self.end]
            self.end_label.config(text=self.end_position)
            if not self.small_timer_started:
                self.start_small_timer()
            self.start_big_timer()
        else:
            self.group_error_label.config(text="Invalid group number")

    def stop_button_clicked(self):
        # 处理Stop按钮点击事件
        if self.big_timer_started:
            self.stop_big_timer()
        else:
            self.group_error_label.config(text="No group is drawing")

    def start_big_timer(self):
        # 启动大计时器
        self.big_timer_started = True
        self.big_timer_start_time = time.time()
        self.flow()

    def start_small_timer(self):
        # 启动小计时器
        self.small_timer_started = True
        self.small_timer_start_time = time.time()

    def big_timer_running(self):
        elapsed_time = time.time() - self.big_timer_start_time
        formatted_time = time.strftime(
            "%H:%M:%S", time.gmtime(elapsed_time*60))
        self.big_timer_label.config(text=formatted_time)
        self.root.update()

    def small_timer_running(self):
        elapsed_time = time.time() - self.small_timer_start_time
        formatted_time = time.strftime(
            "%H:%M:%S", time.gmtime(elapsed_time*60))
        self.small_timer_label.config(text=formatted_time)
        self.root.update()
        if elapsed_time > 900:  # 15 minutes
            # if elapsed_time > 10:  # 15 minutes
            print("time out")
            self.conform_the_result()

    def stop_big_timer(self):
        # 停止大计时器
        self.end = -2
        self.result_listbox.insert(
            tk.END, f"{self.group_number}: {self.big_timer_label.cget('text')}: {self.end_position}")
        self.big_timer_started = False
        self.results.append(
            (self.group_number, self.big_timer_label.cget("text")))
        self.results.sort(key=lambda x: x[1])

    def stop_small_timer(self):
        # 停止小计时器
        self.small_timer_started = False

    def conform_the_result(self):
        self.confirm_error_label.config(text="")
        self.big_timer_started = False
        self.stop_small_timer()
        try:
            file = open(self.file_address + "Result.md", "a+")
            file.seek(0)
            s = file.readlines()
            for num in s:
                try:
                    if str(self.results[0][0]) in str(num):
                        self.confirm_error_label.config(
                            text="The group has already been confirmed")
                        break
                except IndexError:
                    self.confirm_error_label.config(
                        text="No grades have been recorded")
            else:
                try:
                    file.write(
                        f"- {self.results[0][0]}: {self.results[0][1]}\n")
                except IndexError:
                    self.confirm_error_label.config(
                        text="No grades have been recorded")
            file.close()
        except FileNotFoundError:
            self.confirm_error_label.config(text="Result.md not found")
        self.group_number = None
        self.big_timer_label.config(text="00:00:00")
        self.end_label.config(text="")
        self.group_entry.delete(0, tk.END)
        self.result_listbox.delete(0, tk.END)
        self.small_timer_label.config(text="00:00:00")

    def confirm_button_clicked(self):
        # 处理Confirm按钮点击事件
        if not self.group_number:
            self.confirm_error_label.config(text="Invalid group number")
        elif len(self.results) <= 2:
            self.confirm_error_label.config(
                text="The number of grades is insufficient")
        else:
            self.conform_the_result()

    def flow(self):
        while self.small_timer_started:
            self.small_timer_running()
            if self.big_timer_started:
                self.big_timer_running()


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = AccumTimerApp(root)
    except KeyboardInterrupt:
        exit(0)
