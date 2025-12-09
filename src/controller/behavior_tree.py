#!/usr/bin/env python3
"""
Charlie Robot Behavior Tree 
Robot searches in 3 targets, finds and activates valve, returns home.
"""

import py_trees
import time
import random
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status
from py_trees.decorators import FailureIsSuccess
from py_trees.common import ParallelPolicy

#CONDITION: Battery Class to check level Battery >20 --> goCharge
class BatteryRequired(py_trees.behaviour.Behaviour):

    def __init__(self):
        super().__init__(name="BatteryRequired")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("battery", access=py_trees.common.Access.READ)
    
    def update(self):
        batt = self.robot.get("battery")
        if batt > 20:
            self.logger.info(f"  Battery OK: {batt:.0f}%")
            self.feedback_message = f"Battery OK: {batt:.0f}%"
            return Status.SUCCESS
        self.logger.info(f"  Battery LOW: {batt:.0f}% go Charge")
        self.feedback_message = f"Battery LOW: {batt:.0f}%"
        return Status.FAILURE

#ACTION: charge battery
class GoCharge(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="GoCharge")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        batt = min(100, self.robot.get("battery") + 50)
        self.robot.set("battery", batt)
        self.logger.info(f"  Charging... battery to {batt:.0f}%")
        self.feedback_message = f"Charged to {batt:.0f}%"
        return Status.SUCCESS

#Action: calculate nearest target (array of targets positions)
class CalculateTarget(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="CalculateTarget")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("targets", access=py_trees.common.Access.WRITE)
        self.robot.register_key("pos", access=py_trees.common.Access.READ)
        self.robot.register_key("target", access=py_trees.common.Access.WRITE)
    
    def update(self):
        targets = self.robot.get("targets")
        if not targets:
            self.logger.info(f"  No more targets")
            self.feedback_message = "No targets"
            return Status.FAILURE
        
        pos = self.robot.get("pos")
        closest = min(targets, key=lambda t: abs(t["pos"] - pos))
        remaining = [t for t in targets if t != closest]
        
        self.robot.set("target", closest)
        self.robot.set("targets", remaining)
        self.logger.info(f"  Target {closest['id']} selected")
        self.feedback_message = f"Target {closest['id']} selected"
        return Status.SUCCESS

#CONDITION: at target position?
class AtTarget(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="AtTarget")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("pos", access=py_trees.common.Access.READ)
        self.robot.register_key("target", access=py_trees.common.Access.READ)
    
    def update(self):
        pos = self.robot.get("pos")
        target = self.robot.get("target")
        if pos == target["pos"]:
            self.logger.info(f"  Arrived at {target['id']}")
            self.feedback_message = f"Arrived at {target['id']}"
            return Status.SUCCESS
        return Status.FAILURE

#ACTION: move a step tothe target with lose of battery
class MoveToTarget(py_trees.behaviour.Behaviour):

    def __init__(self):
        super().__init__(name="MoveToTarget")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("pos", access=py_trees.common.Access.WRITE)
        self.robot.register_key("target", access=py_trees.common.Access.READ)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        pos = self.robot.get("pos")
        target = self.robot.get("target")
        batt = self.robot.get("battery")
        
        if pos < target["pos"]:
            pos += 1
        elif pos > target["pos"]:
            pos -= 1
        
        batt = max(0, batt - 1)
        self.robot.set("pos", pos)
        self.robot.set("battery", batt)
        self.logger.info(f"  Moving to pos {pos}")
        self.feedback_message = f"Moving to {pos} (battery {batt:.1f}%)"
        return Status.SUCCESS

#ACTION: search objects during movement
class SearchObj(py_trees.behaviour.Behaviour):

    def __init__(self):
        super().__init__(name="SearchOnPath")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("found", access=py_trees.common.Access.WRITE)
    #generate random events on path, 25% person, 35% obstacle, 45% nothing
    def update(self):
        rand = random.random()
        if rand < 0.25:
            self.robot.set("found", "person")
            self.logger.info(f"  Person detected on path!")
            self.feedback_message = "Person detected"
            #if found a person, in blackboard set found to  -person-
            return Status.SUCCESS
        elif rand < 0.35:
            self.robot.set("found", "obstacle")
            self.logger.info(f"  Obstacle detected on path!")
            self.feedback_message = "Obstacle detected"
            #if found an obstacle, in blackboard set found to  -obstacle-
            return Status.SUCCESS
        self.robot.set("found", None)
        self.logger.info(f"  Path clear")
        self.feedback_message = "Path clear"
        # if nothing found, set found to -none-, the path is clear
        return Status.SUCCESS

#ACTION:simulation of the camera recognition for person
class RecognitionPerson(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="RecognitionPerson")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.robot.get("found") == "person" else Status.FAILURE

#ACTION: robot signal pearson found
class SignalPerson(py_trees.behaviour.Behaviour):
  
    def __init__(self):
        super().__init__(name="SignalPearson")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("signals", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.robot.get("signals")
        signals.append("PersonFound")
        self.robot.set("signals", signals)
        self.logger.info(f"  Signal: Person found!")
        self.feedback_message = "Person signaled"
        return Status.SUCCESS

#ACTION: go around the person
class GoAroundP(py_trees.behaviour.Behaviour):

    def __init__(self):
        super().__init__(name="GoAroundP")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        batt = max(0, self.robot.get("battery") - 1)
        self.robot.set("battery", batt)
        self.logger.info(f"  Avoided!")
        self.feedback_message = "Avoided"
        return Status.SUCCESS

#ACTION:simulation of the camera recognition for obstacle
class RecognitionObstacle(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="RecognitionObstacle")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.robot.get("found") == "obstacle" else Status.FAILURE


#ACTION: go around the obstacle
class GoAroundO(py_trees.behaviour.Behaviour):

    def __init__(self):
        super().__init__(name="GoAroundO")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        batt = max(0, self.robot.get("battery") - 2)
        self.robot.set("battery", batt)
        self.logger.info(f"  Avoided!")
        self.feedback_message = "Avoided"
        return Status.SUCCESS

#ACTION: simulation of the camera recognition for valve
class RecognitionValve(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="RecognitionValve")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("target", access=py_trees.common.Access.READ)
    
    def update(self):
        target = self.robot.get("target")
        if target.get("valve"):
            self.logger.info(f"  VALVE FOUND at {target['id']}!")
            self.feedback_message = "Valve found"
            return Status.SUCCESS
        self.logger.info(f"  No valve at {target['id']}, continue search")
        self.feedback_message = "No valve here"
        return Status.FAILURE

#ACTION: activate valve
class ActiveValve(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="ActValve")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("signals", access=py_trees.common.Access.WRITE)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        batt = max(0, self.robot.get("battery") - 2)
        self.robot.set("battery", batt)
        signals = self.robot.get("signals")
        signals.append("ValveActivated")
        self.robot.set("signals", signals)
        self.logger.info(f"  Valve activated!")
        self.feedback_message = "Valve activated"
        return Status.SUCCESS

#ACTION: go home, mission complete daje
class GoHome(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="GoHome")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("pos", access=py_trees.common.Access.WRITE)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        pos = self.robot.get("pos")
        batt = max(0, self.robot.get("battery") - pos)
        pos = 0
        self.robot.set("pos", pos)
        self.robot.set("battery", batt)
        self.logger.info(f"  Returning home (pos {pos})")
        self.feedback_message = f"Returning home (pos {pos})"
        return Status.SUCCESS
        

# Build the behavior tree
def build_tree():
    
    selector0 = Selector("Battery", memory=False)
    selector0.add_children([BatteryRequired(), GoCharge()])
    sequence1 = Sequence("SearchObj", memory=False)
    selector2 = Selector("Person/Obstacle", memory=False)
    sequence2 = Sequence("SeqPerson", memory=False)
    sequence2.add_children([RecognitionPerson(), SignalPerson(), GoAroundP()])
    sequence3 = Sequence("SeqObstacle", memory=False)
    sequence3.add_children([RecognitionObstacle(),GoAroundO()]) ############
    selector2.add_children([sequence2, sequence3])
    sequence1.add_children([SearchObj(), selector2])
    selector1 = Selector("InPosition", memory=False)
    parallel = Parallel("GoTarget", policy=ParallelPolicy.SuccessOnAll())
    decorators1 = decorators.FailureIsSuccess(name="decorator" , child=sequence1)
    selector1.add_children([AtTarget(), parallel])
    parallel.add_children([MoveToTarget(),decorators1])

    # Main loop
    main = Sequence("MainLoop", memory=False)
    main.add_children([
        selector0,
        CalculateTarget(),
        selector1,
        RecognitionValve(),
        ActiveValve(),
        GoHome()
    ])
    return main

# Simulation function
def simulate(name, battery, targets):

    print(f"\n{'='*70}")
    print(f"{name}")
    print(f"{'='*70}")
    print(f"Start: Battery {battery}%, Position 0")
    print(f"Targets: {[(t['id'], t['pos'], t['valve']) for t in targets]}\n")
    
    root = build_tree()
    
    charlie = py_trees.blackboard.Client(name="Robot")
    charlie.register_key("battery", access=py_trees.common.Access.WRITE)
    charlie.register_key("pos", access=py_trees.common.Access.WRITE)
    charlie.register_key("target", access=py_trees.common.Access.WRITE)
    charlie.register_key("targets", access=py_trees.common.Access.WRITE)
    charlie.register_key("found", access=py_trees.common.Access.WRITE)
    charlie.register_key("signals", access=py_trees.common.Access.WRITE)
    
    charlie.set("battery", battery)
    charlie.set("pos", 0)
    charlie.set("target", None)
    charlie.set("targets", targets.copy())
    charlie.set("found", None)
    charlie.set("signals", [])
    
    root.setup_with_descendants()
    
    for tick in range(1, 10):
        root.tick_once()
        
        batt = charlie.get("battery")
        pos = charlie.get("pos")
        target = charlie.get("target")
        signals = charlie.get("signals")
        
        tgt_id = target['id'] if target else "-"
        print(f"T{tick:3d}: Battery {batt:5.1f}% | Pos {pos:2d} | Target {tgt_id:2s} | Signals {signals}")
        # Print tree state
        print("\nTree State:")
        try:
            print(py_trees.display.unicode_tree(root, show_status=True))
        except Exception:
            
            print(f"  Root: {root.name}")
        
        
        if "ValveActivated" in signals:
            break
        
        time.sleep(0.02)
    else:
        print("no")


def main():
    print("\n" + "╔" + "="*68 + "╗")
    print("║" + "CHARLIE ROBOT - BEHAVIOR TREE SIMULATION".center(68) + "║")
    print("╚" + "="*68 + "╝")
    
    targets = [
        {"id": "T1", "pos": 5, "valve": False},
        {"id": "T2", "pos": 12, "valve": True},
        {"id": "T3", "pos": 8, "valve": False},
    ]
    
    # Scenario 1
    simulate("SCENARIO 1: High Battery (90%)", 90.0, targets)
    
    # Scenario 2
    simulate("SCENARIO 2: Low Battery (20%)", 20.0, targets)
    
    # Scenario 3
    simulate("SCENARIO 3: normal", 50.0, targets)


if __name__ == "__main__":
    main()
