import time
import py_trees
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import ParallelPolicy

class BatteryRequired(py_trees.behaviour.Behaviour):
    def __init__(self, name="BatteryRequired", threshold=15.0):
        super(BatteryRequired, self).__init__(name)
        self.threshold = threshold
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("battery", access=py_trees.common.Access.READ)

    def update(self):
        battery = self.blackboard.get("battery")
        if battery > self.threshold:
            self.feedback_message = f"Battery OK ({battery}%)"
            return Status.SUCCESS
        self.feedback_message = f"Low Battery ({battery}%)"
        return Status.FAILURE

class FireDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name="FireDetected"):
        super(FireDetected, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("fire", access=py_trees.common.Access.READ)

    def update(self):
        visible = self.blackboard.get("fire")
        if visible:
            self.feedback_message = "FIRE DETECTED!"
            return Status.SUCCESS
        else:
            return Status.FAILURE

class ValveVisible(py_trees.behaviour.Behaviour):
    def __init__(self, name="ValveVisible"):
        super(ValveVisible, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("valve_visible", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.get("valve_visible"):
            return Status.SUCCESS
        return Status.FAILURE

class PersonVisible(py_trees.behaviour.Behaviour):
    def __init__(self, name="PersonVisible"):
        super(PersonVisible, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("person_visible", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.get("person_visible"):
            return Status.SUCCESS
        return Status.FAILURE

class GoHome(py_trees.behaviour.Behaviour):
    def __init__(self, name="GoHome"):
        super(GoHome, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("valve_visible", access=py_trees.common.Access.READ)

    def update(self):
        valve_visible = self.blackboard.get("valve_visible")
        if valve_visible:
            self.logger.info("ACTION -> GoHomeMissionCompleted")
            return Status.SUCCESS
        # Valve not found, cannot go home (silent FAILURE)
        return Status.FAILURE

class GoCharge(py_trees.behaviour.Behaviour):
    def __init__(self, name="GoCharge", threshold=15.0):
        super(GoCharge, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("battery", access=py_trees.common.Access.READ)
        self.threshold = threshold
    def update(self):
        battery = self.blackboard.get("battery")
        if battery < self.threshold:
            self.logger.info("ACTION -> GoCharge")
            return Status.SUCCESS
        # Battery OK, no need to charge (silent FAILURE)
        return Status.FAILURE

class SignalFire(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> SignalFire (Emergency)")
        return Status.SUCCESS

class ChangeDirection(py_trees.behaviour.Behaviour):
    def __init__(self, name="ChangeDirection"):
        super(ChangeDirection, self).__init__(name)
    def update(self):
        self.logger.info(f"ACTION -> {self.name} (Stopping Robot)")
        return Status.SUCCESS

class ApproachValve(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> ApproachValve")
        return Status.SUCCESS

class TurnValve(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> TurnValve")
        return Status.SUCCESS

class SignalValve(py_trees.behaviour.Behaviour):
    def __init__(self, name="SignalValve"):
        super(SignalValve, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("valve_turned", access=py_trees.common.Access.WRITE)

    def update(self):
        self.logger.info("ACTION -> SignalValve")
        self.blackboard.set("valve_turned", True)
        return Status.SUCCESS

class SignalPerson(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> SignalPerson")
        return Status.SUCCESS

class PerceptionCamera(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> PerceptionCamera")
        return Status.SUCCESS

class CalculateExploring(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> CalculateExploring")
        return Status.SUCCESS

class Idle(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("INFO -> ResultCheckNegative")
        return Status.SUCCESS

class ExploreStep(py_trees.behaviour.Behaviour):
    def update(self):
        self.logger.info("ACTION -> ExploreStep")
        return Status.SUCCESS

def create_rescue_tree():
    root = Selector("Root", memory=False)

    # 1. The Mission
    main_seq = Sequence("MainSequence", memory=False)

    # 1.1 Fire Safety
    fire_safety = Sequence("FireSafety", memory=False)
    fire_handler = Sequence("FireHandler", memory=False)
    fire_handler.add_children([
        FireDetected(name="FireDetected"),
        SignalFire(name="SignalFire"),
        ChangeDirection(name="ChangeDirection")
    ])
    
    fire_inverter = py_trees.decorators.Inverter(name="InverterFireHandler", child=fire_handler)
    
    fire_safety.add_children([fire_inverter, Idle(name="NoFireContinue")])

    # 1.2 Create the main mission (Parallel)
    # Both tasks execute in parallel
    # Use SuccessOnAll so both get a chance to run
    mission_parallel = Parallel("MissionParallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    
    # Primary: Valve
    valve_task = Sequence("ValveTask", memory=False)
    valve_task.add_children([
        ValveVisible(name="ValveFound"),
        ApproachValve(name="MoveToValve"),
        TurnValve(name="TurnValve"),
        SignalValve(name="SignalConfirm")  # Sets valve_turned=True
    ])

    # Secondary: Person  
    person_task = Sequence("PersonTask", memory=False)
    person_task.add_children([
        PersonVisible(name="PersonFound"),
        SignalPerson(name="SignalPerson")  # Sets person_found=True
    ])
    
    # Add Idle fallback so tasks that fail don't block the parallel
    valve_fallback = Selector("ValveFallback", memory=False)
    valve_fallback.add_children([valve_task, Idle(name="NoValve")])
    
    person_fallback = Selector("PersonFallback", memory=False)
    person_fallback.add_children([person_task, Idle(name="NoPerson")])
    
    mission_parallel.add_children([valve_fallback, person_fallback])

    # 1.3 Exploration
    explore_seq = Sequence("Exploration", memory=False)
    
    move_decision = Parallel("MoveDecisionParallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    move_decision.add_children([
        PerceptionCamera(name="PerceptionCamera"),
        CalculateExploring(name="CalculateExploring")
    ])
    explore_seq.add_child(move_decision)
    explore_seq.add_child(ExploreStep(name="ExploreRoom"))

    # Assemble Main Sequence
    main_seq.add_children([fire_safety, mission_parallel, explore_seq])

    # 2. Root Logic
    # MissionTree = Battery Check + GoCharge Guard + Mission + GoHome
    
    # Mission (Battery Check + Main Sequence + GoHome)
    mission_branch = Sequence("MissionTreeSequence", memory=False)
    charge_inverter = py_trees.decorators.Inverter(name="ChargeInverter", child=GoCharge(name="GoCharge"))

    mission_branch.add_children([
        BatteryRequired(name="BatteryRequired"),
        charge_inverter,
        main_seq,
        GoHome(name="GoHome")  # GoHome only if mission completes (valve_turned=True)
    ])

    # Root contains only MissionTree
    root.add_children([mission_branch])

    return root

def main():
    print("="*50)
    print("Rescue Robot Behavior Tree (Charlie)")
    print("="*50)

    root = create_rescue_tree()
    
    bb = py_trees.blackboard.Client(name="RescueBot")
    bb.register_key("battery", access=py_trees.common.Access.WRITE)
    bb.register_key("fire", access=py_trees.common.Access.WRITE)
    bb.register_key("valve_visible", access=py_trees.common.Access.WRITE)
    bb.register_key("person_visible", access=py_trees.common.Access.WRITE)
    bb.register_key("valve_turned", access=py_trees.common.Access.WRITE)

    # Default State
    bb.set("battery", 100.0)
    bb.set("fire", False)
    bb.set("valve_visible", False)
    bb.set("person_visible", False)
    bb.set("valve_turned", False)

    root.setup_with_descendants()
    
    print("\nTree Architecture:")
    print(py_trees.display.unicode_tree(root, show_status=True))
    print()

    # Simulation Scenarios
    scenarios = [
        {"desc": "Normal Exploration", "fire": False, "valve": False, "person": False, "bat": 100},
        {"desc": "Valve Found", "fire": False, "valve": True, "person": False, "bat": 90},
        {"desc": "Person Found", "fire": False, "valve": False, "person": True, "bat": 80},
        {"desc": "Fire Emergency", "fire": True, "valve": False, "person": False, "bat": 70},
        {"desc": "Valve & person Found", "fire": False, "valve": True, "person": True, "bat": 70},
        {"desc": "Low Battery", "fire": False, "valve": False, "person": False, "bat": 10},
    ]

    for i, case in enumerate(scenarios, 1):
        print(f"\n--- Scenario {i}: {case['desc']} ---")
        bb.set("battery", case["bat"])
        bb.set("fire", case["fire"])
        bb.set("valve_visible", case["valve"])
        bb.set("person_visible", case["person"])
        bb.set("valve_turned", False)  # Reset valve flag for each scenario
        
        print(f"Sensors ->Battery: {case['bat']} | Fire: {case['fire']} | Person: {case['person']} | Valve: {case['valve']}")

        root.tick_once()
        print(py_trees.display.unicode_tree(root, show_status=True))
        time.sleep(1)

    print("\nDemo completed.")

if __name__ == "__main__":
    main()
