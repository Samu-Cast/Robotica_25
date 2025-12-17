"""
    Charlie Robot - Plan Module
    BT-based decision making for ROS2

    Subscribes to:
        /sense/world_state   - JSON: {obstacles, detections, robot_state, battery}

    Publishes to:
        /plan/action        - String: action command (MOVE_TO_GOAL, AVOID, STOP, etc.)
        /plan/goal_pose     - String: JSON goal {x, y, theta}
"""

import json
import random
import random
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status, ParallelPolicy

##########################################################################
#DA decidere le posizone e la quantità dei targhet
KNOW_TARGETS = { 
    'green': {'x': 2.0, 'y': 3.0, 'theta': 0.0}, #theta è la direzione di arrivo del robot (dove punta) lo facciamo puntare verso il prossimo target 
    '1': {'x': 2.0, 'y': 3.0, 'theta': 0.0}, #possibilità di di avere taret con numero oltre che colori
    'blue': {'x': 5.0, 'y': 9.0, 'theta': 1.57},
    '2': {'x': 5.0, 'y': 9.0, 'theta': 1.57},
    'red': {'x': 10.0, 'y': 15.0, 'theta': 0.0}, # -> posizione della valvola
    '3': {'x': 10.0, 'y': 15.0, 'theta': 0.0},
    # aggiungi altri target qui
}
##########################################################################

class BatteryCheck(py_trees.behaviour.Behaviour):
    """Verifica se batteria > 20%"""
    def __init__(self):
        super().__init__(name="BatteryCheck")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("battery", access=py_trees.common.Access.READ)
    
    def update(self):
        batt = self.bb.get("battery")
        return Status.SUCCESS if batt > 20 else Status.FAILURE


class GoCharge(py_trees.behaviour.Behaviour):
    """Vai alla stazione di ricarica e ricarica la batteria"""
    def __init__(self):
        super().__init__(name="GoCharge")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("battery", access=py_trees.common.Access.WRITE)
        self.bb.register_key("obstacles", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
    
    def update(self):
        obstacles = self.bb.get("obstacles") or []
        battery = self.bb.get("battery")
        
        #Posizione stazione di ricarica: punto di spawn
        goal_pose = {
            'x': 0.0,
            'y': 0.0,
            'theta': 3.14159  #π radianti = 180°
        }
        self.bb.set("goal_pose", goal_pose)
        
        #Evita ostacoli se presenti
        has_obstacle = any(obs.get('distance', 999) < 0.5 for obs in obstacles)
        action = "AVOID_OBSTACLE" if has_obstacle else "MOVE_TO_GOAL"
        self.bb.set("plan_action", action)
        
        # Simula ricarica randomizzata
        carica = random.randint(30, 100 - int(battery))
        new_battery = min(100, battery + carica)
        self.bb.set("battery", new_battery)
        
        return Status.SUCCESS if new_battery >= 80 else Status.RUNNING


class CalculateTarget(py_trees.behaviour.Behaviour):
    """Seleziona prossimo target da KNOWN_TARGETS"""
    def __init__(self):
        super().__init__(name="CalculateTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)

    def update(self):
        visited = self.bb.get("visited_targets") or []

        # Cerca il primo target non visitato
        for target_name, target_data in KNOWN_TARGETS.items():
            if target_name not in visited:
                # Seleziona questo target
                self.bb.set("current_target", {
                    'name': target_name,
                    'x': target_data['x'],
                    'y': target_data['y'],
                    'theta': target_data['theta']
                })
                return Status.SUCCESS

        # Tutti i target già visitati
        return Status.FAILURE


class AtTarget(py_trees.behaviour.Behaviour):
    """Verifica se siamo al target + reset odometria"""
    def __init__(self):
        super().__init__(name="AtTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("current_target", access=py_trees.common.Access.READ)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)  # colore rilevato dalla camera
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("reset_odom", access=py_trees.common.Access.WRITE)  # per resettare odometria

    def update(self):
        target = self.bb.get("current_target")
        if not target:
            return Status.FAILURE
        
        # Verifica colore visivo (rilevato dalla camera)
        detected_color = self.bb.get("detected_color")  # es: 'green', 'blue', 'red'
        target_name = target.get("name")  # es: 'green'
        
        if detected_color != target_name:
            # Colore non corrisponde, non siamo al target
            return Status.FAILURE
        
        # 1. Reset odometria alla posizione nota del target
        reset_pose = {
            'x': target['x'],
            'y': target['y'],
            'theta': target['theta']
        }
        self.bb.set("reset_odom", reset_pose)  # Sense leggerà questo per resettare
        
        # 2. Marca target come visitato
        visited = self.bb.get("visited_targets") or []
        if target_name not in visited:
            visited.append(target_name)
            self.bb.set("visited_targets", visited)
        
        return Status.SUCCESS


class MoveToTarget(py_trees.behaviour.Behaviour):
    """Muovi verso target"""
    def __init__(self):
        super().__init__(name="MoveToTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("obstacles", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key("current_target", access=py_trees.common.Access.READ)
    
    def update(self):
        obstacles = self.bb.get("obstacles") or []
        target = self.bb.get("current_target")
        
        # Pubblica goal usando coordinate x,y,theta dal target
        if target and 'x' in target and 'y' in target and 'theta' in target:
            goal_pose = {
                'x': target['x'],
                'y': target['y'],
                'theta': target['theta']
            }
            self.bb.set("goal_pose", goal_pose)
        
        # Decidi azione
        has_obstacle = any(obs.get('distance', 999) < 0.5 for obs in obstacles)
        action = "AVOID_OBSTACLE" if has_obstacle else "MOVE_TO_GOAL"
        self.bb.set("plan_action", action)
        
        return Status.RUNNING


class SearchObj(py_trees.behaviour.Behaviour):
    """Cerca oggetti (person, valve)"""
    def __init__(self):
        super().__init__(name="SearchObj")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("detections", access=py_trees.common.Access.READ)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)
    
    def update(self):
        detections = self.bb.get("detections") or {}
        
        # Priorità: valve > person > obstacle
        if detections.get("valve"):
            self.bb.set("found", "valve")
        elif detections.get("person"):
            self.bb.set("found", "person")
        elif detections.get("obstacle"):
            self.bb.set("found", "obstacle")
        else:
            self.bb.set("found", None)
        
        return Status.SUCCESS


class RecognitionPerson(py_trees.behaviour.Behaviour):
    """Verifica se found == person"""
    def __init__(self):
        super().__init__(name="RecognitionPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "person" else Status.FAILURE


class SignalPerson(py_trees.behaviour.Behaviour):
    """Segnala persona trovata"""
    def __init__(self):
        super().__init__(name="SignalPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.bb.get("signals") or []
        signals.append("PersonFound")
        self.bb.set("signals", signals)
        return Status.SUCCESS


class GoAroundP(py_trees.behaviour.Behaviour):
    """Evita persona"""
    def __init__(self):
        super().__init__(name="GoAroundP")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
    
    def update(self):
        self.bb.set("plan_action", "AVOID_OBSTACLE")
        return Status.SUCCESS


class RecognitionObstacle(py_trees.behaviour.Behaviour):
    """Verifica se found == obstacle"""
    def __init__(self):
        super().__init__(name="RecognitionObstacle")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "obstacle" else Status.FAILURE


class GoAroundO(py_trees.behaviour.Behaviour):
    """Evita ostacolo"""
    def __init__(self):
        super().__init__(name="GoAroundO")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
    
    def update(self):
        self.bb.set("plan_action", "AVOID_OBSTACLE")
        return Status.SUCCESS


class RecognitionValve(py_trees.behaviour.Behaviour):
    """Verifica se valve trovata"""
    def __init__(self):
        super().__init__(name="RecognitionValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "valve" else Status.FAILURE


class ActiveValve(py_trees.behaviour.Behaviour):
    """Attiva valvola"""
    def __init__(self):
        super().__init__(name="ActiveValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("mission_complete", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.bb.get("signals") or []
        signals.append("ValveActivated")
        self.bb.set("signals", signals)
        self.bb.set("plan_action", "ACTIVATE_VALVE")
        self.bb.set("mission_complete", True)
        return Status.SUCCESS


class GoHome(py_trees.behaviour.Behaviour):
    """Torna a home"""
    def __init__(self):
        super().__init__(name="GoHome")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("room_color", access=py_trees.common.Access.READ)
        self.bb.register_key("home_color", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
    
    def update(self):
        current = self.bb.get("room_color") or [0,0,0]
        home = self.bb.get("home_color") or [128,128,128]
        diff = sum(abs(c-h) for c,h in zip(current, home)) / 3
        
        if diff < 30:
            self.bb.set("plan_action", "STOP")
            return Status.SUCCESS
        
        self.bb.set("plan_action", "MOVE_TO_GOAL")
        return Status.RUNNING


def build_tree():
    """Costruisce BT minimale ma completo"""
    
    # Battery management
    battery = Selector("Battery", memory=False, children=[
        BatteryCheck(),
        GoCharge()
    ])
    
    # Person handling sequence
    person_seq = Sequence("PersonHandler", memory=False, children=[
        RecognitionPerson(),
        SignalPerson(),
        GoAroundP()
    ])
    
    # Obstacle handling sequence
    obstacle_seq = Sequence("ObstacleHandler", memory=False, children=[
        RecognitionObstacle(),
        GoAroundO()
    ])
    
    # Found handler (person or obstacle)
    found_handler = Selector("FoundHandler", memory=False, children=[
        person_seq,
        obstacle_seq
    ])
    
    # Search while moving
    search = Sequence("Search", memory=False, children=[
        SearchObj(),
        found_handler
    ])
    
    # Decorator for search (don't block parallel)
    search_dec = decorators.FailureIsSuccess(
        name="SearchDecorator",
        child=search
    )
    
    # Move with search in parallel
    move_search = Parallel("MoveAndSearch", 
        policy=ParallelPolicy.SuccessOnAll(),
        children=[MoveToTarget(), search_dec]
    )
    
    # Go to target
    goto = Selector("GoToTarget", memory=False, children=[
        AtTarget(),
        move_search
    ])
    
    # Main sequence
    main = Sequence("Main", memory=True, children=[
        battery,
        CalculateTarget(),
        goto,
        RecognitionValve(),
        ActiveValve(),
        GoHome()
    ])
    
    # Loop until success
    root = decorators.Retry("LoopUntilSuccess", child=main, num_failures=-1)
    
    return root


class PlanNode(Node):
    """
        Plan ROS2 Node
    """
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Plan Node Starting ===')
        
        # BT setup
        self.tree = build_tree()
        self.bb = py_trees.blackboard.Client(name="PlanNode")
        
        # Blackboard keys
        for key in ['battery', 'obstacles', 'detections', 'room_color', 'home_color',
                    'targets', 'current_target', 'found', 'signals', 'plan_action', 
                    'goal_pose', 'mission_complete']:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)
        
        self._init_blackboard()
        self.tree.setup_with_descendants()
        
        # Subscriptions
        self.create_subscription(String, '/sense/world_state', self._world_cb, 10)
        
        # Publishers
        self.action_pub = self.create_publisher(String, '/plan/action', 10)
        self.goal_pub = self.create_publisher(String, '/plan/goal_pose', 10)
        
        # Timer
        self.create_timer(0.1, self._tick)
        
        self.get_logger().info('Plan Node ready')
    
    def _init_blackboard(self):
        """Init blackboard"""
        self.bb.set("battery", 100.0)
        self.bb.set("obstacles", [])
        self.bb.set("detections", {})
        self.bb.set("room_color", [128,128,128])
        self.bb.set("home_color", [128,128,128])
        self.bb.set("targets", [
            {'id': 'Room1', 'color': [255,0,0]},
            {'id': 'Room2', 'color': [0,255,0]},
            {'id': 'Room3', 'color': [0,0,255]},
        ])
        self.bb.set("current_target", None)
        self.bb.set("visited_targets", [])
        self.bb.set("found", None)
        self.bb.set("signals", [])
        self.bb.set("plan_action", "IDLE")
        self.bb.set("goal_pose", None)
        self.bb.set("mission_complete", False)
    
    def _world_cb(self, msg):
        """Riceve world state da Sense"""
        try:
            data = json.loads(msg.data)
            self.bb.set("obstacles", data.get("obstacles", []))
            self.bb.set("detections", data.get("detections", {}))
            self.bb.set("battery", data.get("battery", 100.0))
            if 'room_color' in data:
                self.bb.set("room_color", data['room_color'])
        except json.JSONDecodeError:
            pass
    
    def _tick(self):
        """BT tick + publish"""
        self.tree.tick_once()
        
        # Publish action
        action = self.bb.get("plan_action") or "IDLE"
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)
        
        # Publish goal (if exists)
        goal = self.bb.get("goal_pose")
        if goal:
            goal_msg = String()
            goal_msg.data = json.dumps(goal)
            self.goal_pub.publish(goal_msg)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available")
        return
    
    rclpy.init(args=args)
    node = PlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
