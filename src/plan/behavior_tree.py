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

#ACTION: move a step to the target with loss of battery
#Returns RUNNING while moving, SUCCESS when at target
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
        
        # Già al target? → SUCCESS
        if pos == target["pos"]:
            self.logger.info(f"  Arrived at target pos {pos}")
            self.feedback_message = f"At target {pos}"
            return Status.SUCCESS
        
        # Muovi di un passo verso target
        if pos < target["pos"]:
            pos += 1
        else:
            pos -= 1
        
        batt = max(0, batt - 1)
        self.robot.set("pos", pos)
        self.robot.set("battery", batt)
        self.logger.info(f"  Moving to pos {pos}")
        self.feedback_message = f"Moving to {pos} (battery {batt:.1f}%)"
        return Status.RUNNING  # Continua fino a raggiungere target

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

#ACTION: robot signal person found
class SignalPerson(py_trees.behaviour.Behaviour):
  
    def __init__(self):
        super().__init__(name="SignalPerson")  # Fixed typo
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

#ACTION: go home step by step, mission complete daje
class GoHome(py_trees.behaviour.Behaviour):
    
    def __init__(self):
        super().__init__(name="GoHome")
        self.robot = self.attach_blackboard_client(name=self.name)
        self.robot.register_key("pos", access=py_trees.common.Access.WRITE)
        self.robot.register_key("battery", access=py_trees.common.Access.WRITE)
    
    def update(self):
        pos = self.robot.get("pos")
        
        # Arrivato a casa?
        if pos == 0:
            self.logger.info(f"  Home reached!")
            self.feedback_message = "Home reached"
            return Status.SUCCESS
        
        # Muovi di un passo verso home
        if pos > 0:
            pos -= 1
        else:
            pos += 1
        
        batt = max(0, self.robot.get("battery") - 1)
        self.robot.set("pos", pos)
        self.robot.set("battery", batt)
        self.logger.info(f"  Moving home (pos {pos})")
        self.feedback_message = f"Moving home (pos {pos})"
        return Status.RUNNING  # Continua fino a pos=0
        

# Build the behavior tree
def build_tree():
    
    # === FallBack0: Battery management ===
    fallback0 = Selector("FallBack0_Battery", memory=False, children=[
        BatteryRequired(),
        GoCharge()
    ])
    
    # === Sequence2: Handle Person ===
    sequence2 = Sequence("Sequence2_Person", memory=False, children=[
        RecognitionPerson(),
        SignalPerson(),
        GoAroundP()
    ])
    
    # === Sequence3: Handle Obstacle ===
    sequence3 = Sequence("Sequence3_Obstacle", memory=False, children=[
        RecognitionObstacle(),
        GoAroundO()
    ])
    
    # === FallBack2: Person / Obstacle ===
    fallback2 = Selector("FallBack2_HandleFound", memory=False, children=[
        sequence2,
        sequence3
    ])
    
    # === Sequence1: SearchObj → FallBack2 ===
    sequence1 = Sequence("Sequence1_Search", memory=False, children=[
        SearchObj(),
        fallback2
    ])
    
    # === Decorator1: FailureIsSuccess per non bloccare il Parallel ===
    decorator1 = decorators.FailureIsSuccess(
        name="Decorator1_SearchLoop",
        child=sequence1
    )
    
    # === Parallel0: MoveToTarget + Decorator1 ===
    # SuccessOnAll: aspetta che MoveToTarget arrivi al target
    parallel0 = Parallel(
        "Parallel0_MoveAndSearch",
        policy=ParallelPolicy.SuccessOnAll(),
        children=[MoveToTarget(), decorator1]
    )
    
    # === FallBack1: AtTarget? / Parallel0 ===
    fallback1 = Selector("FallBack1_GoToTarget", memory=False, children=[
        AtTarget(),
        parallel0
    ])
    
    # === Sequence0: Main sequence (dentro il loop) ===
    # memory=True per ricordare il progresso quando un figlio ritorna RUNNING
    sequence0 = Sequence("Sequence0_Main", memory=True, children=[
        fallback0,           # Battery check
        CalculateTarget(),   # Select next target
        fallback1,           # Go to target (with search)
        RecognitionValve(),  # Check valve (FAILURE → loop retry)
        ActiveValve(),       # Activate valve (only if valve found)
        GoHome()             # Return home (only after valve activated)
    ])
    
    # === Decorator0: LoopUntilSuccess ===
    # Retry con num_failures=-1 ripete all'infinito finché non c'è SUCCESS
    decorator0 = decorators.Retry(
        name="Decorator0_LoopUntilSuccess",
        child=sequence0,
        num_failures=-1  # -1 = infinite retries until success
    )
    
    return decorator0

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
