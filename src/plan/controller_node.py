#!/usr/bin/env python3
"""
Controller Node - Behavior Tree Decision Making
"""

import py_trees
from py_trees.common import Status
from py_trees.composites import Sequence, Selector
import time


class BatteryCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name="BatteryCheck", threshold=15.0):
        super().__init__(name)
        self.threshold = threshold
        self.battery = 100.0
    
    def update(self):
        if self.battery > self.threshold:
            return Status.SUCCESS
        return Status.FAILURE


def create_behavior_tree():
    root = Selector("Root", memory=False)
    
    battery_check = BatteryCheck(name="BatteryCheck")
    
    root.add_child(battery_check)
    
    return root


def main():
    print("Controller Node - Starting...")
    
    tree = create_behavior_tree()
    tree.setup_with_descendants()
    
    print(py_trees.display.unicode_tree(tree, show_status=True))
    
    while True:
        tree.tick_once()
        time.sleep(1)


if __name__ == "__main__":
    main()
