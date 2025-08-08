#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time

class CheckTimer:
    def __init__(self,name):
        self._start = None
        self.name = name


    def start(self):
        self._start = time.time()

    def check(self):
        if self._start is None:
            raise RuntimeError("Timer has not been started yet.")
        end_time = time.time()
        elapsed = end_time - self._start
        print(f"[{self.name}] [Timer] 경과 시간: {elapsed:.6f}초")
        return elapsed
