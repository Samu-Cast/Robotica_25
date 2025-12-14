"""
    Test plan_node.py con dati fittizi.
    Importa classi da plan_node e testa ogni metodo.
"""

import py_trees
from py_trees.common import Status
import sys
import os

# Path alla cartella plan/ (parent di tests/)
plan_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, plan_dir)

from plan_node import (
    BatteryRequired, GoCharge, CalculateTarget, AtTarget,
    MoveToTarget, RecognitionValve, ActiveValve, GoHome,
    SearchObj, RecognitionPerson, SignalPerson, GoAroundP,
    RecognitionObstacle, GoAroundO,
    build_behavior_tree, BLACKBOARD_KEYS
)

py_trees.blackboard.Blackboard.enable_activity_stream()


def setup_blackboard():
    """Crea blackboard con tutti i valori di default."""
    bb = py_trees.blackboard.Client(name="Test")
    for key in BLACKBOARD_KEYS:
        bb.register_key(key, access=py_trees.common.Access.WRITE)
    
    bb.set("battery", 80.0)
    bb.set("pos", {'x': 0, 'y': 0, 'theta': 0})
    bb.set("obstacles", [1.5, 1.0, 1.0])
    bb.set("detections", {})
    bb.set("room_color", [128, 128, 128])
    bb.set("home_color", [128, 128, 128])
    bb.set("found", None)  # Oggetto trovato (person/obstacle/None)
    bb.set("targets", [
        {'id': 'Room1', 'color': [255, 0, 0]},
        {'id': 'Room2', 'color': [0, 255, 0]},
    ])
    bb.set("target", None)
    bb.set("signals", [])
    bb.set("valve_found", False)
    bb.set("cmd_vel", {})
    return bb


def test_all_nodes():
    """Testa ogni nodo BT con dati fake."""
    bb = setup_blackboard()
    
    print("=== Test Nodi BT ===\n")
    
    # BatteryRequired
    node = BatteryRequired()
    node.setup()
    bb.set("battery", 80.0)
    assert node.update() == Status.SUCCESS
    bb.set("battery", 10.0)
    assert node.update() == Status.FAILURE
    print("✓ BatteryRequired")
    
    
    # GoCharge (nuovo comportamento: GOING_HOME -> CHARGING -> RETURNING)
    node = GoCharge()
    node.setup()
    bb.set("battery", 20.0)
    bb.set("room_color", [128, 128, 128])  # Già a home
    bb.set("home_color", [128, 128, 128])
    bb.set("obstacles", [1.0, 1.0, 1.0])
    node.initialise()  # Inizializza stato
    # Tick 1: a home -> passa a CHARGING
    status = node.update()
    assert status == Status.RUNNING
    # Tick 2: ricarica a 50% -> passa a RETURNING (poiché saved_color == home)
    status = node.update()
    assert bb.get("battery") == 50.0
    print("OK GoCharge")
    
    # CalculateTarget
    node = CalculateTarget()
    node.setup()
    bb.set("targets", [{'id': 'T1', 'color': [255,0,0]}])
    bb.set("target", None)
    assert node.update() == Status.SUCCESS
    assert bb.get("target")['id'] == 'T1'
    print("✓ CalculateTarget")
    
    # AtTarget
    node = AtTarget()
    node.setup()
    bb.set("target", {'id': 'T1', 'color': [255, 0, 0]})
    bb.set("room_color", [250, 10, 5])  # Match
    assert node.update() == Status.SUCCESS
    bb.set("room_color", [0, 0, 255])   # No match
    assert node.update() == Status.FAILURE
    print("✓ AtTarget")
    
    # MoveToTarget
    node = MoveToTarget()
    node.setup()
    bb.set("obstacles", [2.0, 1.0, 1.0])
    assert node.update() == Status.RUNNING
    assert bb.get("cmd_vel")['linear'] > 0
    print("✓ MoveToTarget")
    
    # RecognitionValve
    node = RecognitionValve()
    node.setup()
    bb.set("detections", {})
    assert node.update() == Status.FAILURE
    bb.set("detections", {'valve': True})
    assert node.update() == Status.SUCCESS
    print("✓ RecognitionValve")
    
    # ActiveValve
    node = ActiveValve()
    node.setup()
    bb.set("signals", [])
    bb.set("valve_found", False)
    node.update()
    assert bb.get("valve_found") == True
    assert "ValveActivated" in bb.get("signals")
    print("✓ ActiveValve")
    
    # GoHome
    node = GoHome()
    node.setup()
    bb.set("room_color", [255, 0, 0])
    bb.set("obstacles", [2.0, 1.0, 1.0])
    assert node.update() == Status.RUNNING
    bb.set("room_color", [128, 128, 128])
    assert node.update() == Status.SUCCESS
    print("OK GoHome")
    
    # SearchObj - cerca persona
    node = SearchObj()
    node.setup()
    bb.set("detections", {'person': True})
    node.update()
    assert bb.get("found") == "person"
    print("OK SearchObj (person)")
    
    # SearchObj - cerca ostacolo
    bb.set("detections", {'obstacle': True})
    node.update()
    assert bb.get("found") == "obstacle"
    print("OK SearchObj (obstacle)")
    
    # SearchObj - niente
    bb.set("detections", {})
    node.update()
    assert bb.get("found") is None
    print("OK SearchObj (clear)")
    
    # RecognitionPerson
    node = RecognitionPerson()
    node.setup()
    bb.set("found", "person")
    assert node.update() == Status.SUCCESS
    bb.set("found", None)
    assert node.update() == Status.FAILURE
    print("OK RecognitionPerson")
    
    # SignalPerson
    node = SignalPerson()
    node.setup()
    bb.set("signals", [])
    node.update()
    assert "PersonFound" in bb.get("signals")
    print("OK SignalPerson")
    
    # GoAroundP
    node = GoAroundP()
    node.setup()
    node.update()
    assert bb.get("cmd_vel")['angular'] > 0
    print("OK GoAroundP")
    
    # RecognitionObstacle
    node = RecognitionObstacle()
    node.setup()
    bb.set("found", "obstacle")
    assert node.update() == Status.SUCCESS
    bb.set("found", None)
    assert node.update() == Status.FAILURE
    print("OK RecognitionObstacle")
    
    # GoAroundO
    node = GoAroundO()
    node.setup()
    node.update()
    assert bb.get("cmd_vel")['angular'] < 0
    print("OK GoAroundO")


def test_behavior_tree():
    """Testa decisioni BT con diversi scenari di input."""
    print("\n=== Test Behavior Tree - Decisioni ===\n")
    
    # SCENARIO 1: Batteria bassa → ricarica (GoCharge ha 3 stati)
    print("SCENARIO 1: Batteria bassa → GoCharge attivato")
    bb = setup_blackboard()
    bb.set("battery", 10.0)
    bb.set("room_color", [128, 128, 128])  # Già a home
    tree = build_behavior_tree()
    tree.setup_with_descendants()
    tree.tick_once()  # Tick 1: va a charging station (già lì)
    tree.tick_once()  # Tick 2: ricarica
    # Ora dovrebbe essere almeno 40%
    assert bb.get("battery") >= 40.0
    print(f"  10% → {bb.get('battery'):.0f}% OK")
    
    # SCENARIO 2: Batteria ok → seleziona target e muove
    print("\nSCENARIO 2: Batteria ok → seleziona e muove")
    bb = setup_blackboard()
    bb.set("battery", 80.0)
    bb.set("room_color", [100, 100, 100])  # Non al target
    tree = build_behavior_tree()
    tree.setup_with_descendants()
    tree.tick_once()
    assert bb.get("target")['id'] == 'Room1'
    assert bb.get("cmd_vel")['linear'] > 0
    print(f"  Target: {bb.get('target')['id']}, moving ✓")
    
    # SCENARIO 3: Ostacolo → evita
    print("\nSCENARIO 3: Ostacolo → evita")
    bb.set("obstacles", [0.3, 2.0, 0.5])
    tree.tick_once()
    assert bb.get("cmd_vel")['linear'] == 0.0
    assert bb.get("cmd_vel")['angular'] > 0  # Gira a sinistra
    print(f"  Evita ostacolo: angular={bb.get('cmd_vel')['angular']:.2f} ✓")
    
    # SCENARIO 4: Arrivato + valve trovata → attiva
    print("\nSCENARIO 4: Al target + valve → attiva")
    bb.set("room_color", [250, 10, 5])  # Match Room1
    bb.set("obstacles", [2.0, 1.0, 1.0])
    bb.set("detections", {'valve': True})
    tree.tick_once()
    tree.tick_once()
    assert bb.get("valve_found") == True
    print(f"  Valve attivata: {bb.get('valve_found')} ✓")
    
    # SCENARIO 5: GoHome (nuovo tree per testare GoHome isolato)
    print("\nSCENARIO 5: GoHome - corridoio")
    bb = setup_blackboard()
    bb.set("valve_found", True)  # Simula valve già trovata
    bb.set("room_color", [255, 0, 0])  # Rosso, lontano da home grigio
    bb.set("obstacles", [2.0, 1.0, 1.0])
    from plan_node import GoHome
    gohome = GoHome()
    gohome.setup()
    status = gohome.update()
    assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
    assert bb.get("cmd_vel")['linear'] > 0
    print(f"  Going home: linear={bb.get('cmd_vel')['linear']:.2f}, status=RUNNING ✓")
    
    # SCENARIO 6: GoHome - arrivato
    print("\nSCENARIO 6: GoHome - arrivato a casa")
    bb.set("room_color", [128, 128, 128])  # Home color
    status = gohome.update()
    assert status == Status.SUCCESS
    assert bb.get("cmd_vel")['linear'] == 0.0
    print(f"  Home! status=SUCCESS OK")
    
    # SCENARIO 7: Person detection durante movimento
    print("\nSCENARIO 7: Person detection")
    bb = setup_blackboard()
    bb.set("detections", {'person': True})
    node = SearchObj()
    node.setup()
    node.update()
    assert bb.get("found") == "person"
    node2 = RecognitionPerson()
    node2.setup()
    assert node2.update() == Status.SUCCESS
    node3 = SignalPerson()
    node3.setup()
    node3.update()
    assert "PersonFound" in bb.get("signals")
    print(f"  Person detected and signaled OK")
    
    # SCENARIO 8: Obstacle detection durante movimento
    print("\nSCENARIO 8: Obstacle detection")
    bb = setup_blackboard()
    bb.set("detections", {'obstacle': True})
    node = SearchObj()
    node.setup()
    node.update()
    assert bb.get("found") == "obstacle"
    node2 = RecognitionObstacle()
    node2.setup()
    assert node2.update() == Status.SUCCESS
    node3 = GoAroundO()
    node3.setup()
    node3.update()
    assert bb.get("cmd_vel")['angular'] != 0
    print(f"  Obstacle detected and avoided OK")


if __name__ == "__main__":
    print("=" * 50)
    print("TEST PLAN_NODE + BEHAVIOR TREE")
    print("=" * 50)
    
    test_all_nodes()
    test_behavior_tree()
    
    print("\n" + "=" * 50)
    print("ALL TESTS PASSED ✓")
    print("=" * 50)
