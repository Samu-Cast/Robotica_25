"""
    Test Suite per Plan Module - Charlie Robot
    
    Questa classe di test valida tutti i behavior del modulo Plan
    senza logica complessa. Ogni test:
    1. Configura la blackboard con dati specifici
    2. Esegue il behavior
    3. Verifica l'output atteso
    
    Usage:
        python3 test_plan_behaviors.py
        
    Oppure con pytest:
        pytest test_plan_behaviors.py -v
"""

import sys
import json
import py_trees
from py_trees.common import Status

# Import dei behavior da testare
from plan_node import (
    BatteryCheck, GoCharge, CalculateTarget, AtTarget, MoveToTarget,
    SearchObj, RecognitionPerson, RecognitionObstacle, RecognitionValve,
    SignalPerson, GoAroundP, GoAroundO, ActiveValve, GoHome,
    build_tree, KNOWN_TARGETS
)


class TestPlanBehaviors:
    """Test suite completa per tutti i behavior del Plan module"""
    
    def setup_method(self):
        """Setup eseguito prima di ogni test - pulisce la blackboard"""
        py_trees.blackboard.Blackboard.clear()
        self.bb = py_trees.blackboard.Client(name="TestClient")
        
        # Registra tutte le chiavi necessarie
        keys = [
            'battery', 'obstacles', 'detections', 'room_color', 'home_color',
            'current_target', 'visited_targets', 'found', 'signals', 
            'plan_action', 'goal_pose', 'mission_complete', 'detected_color',
            'reset_odom', 'distance_left', 'distance_center', 'distance_right',
            'robot_position', 'yolo_valve'
        ]
        for key in keys:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)
        
        # Inizializza con valori di default
        self.bb.set("distance_left", 999.0)
        self.bb.set("distance_center", 999.0)
        self.bb.set("distance_right", 999.0)
        self.bb.set("detected_color", None)
        self.bb.set("robot_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.bb.set("visited_targets", [])
        self.bb.set("yolo_valve", False)
        self.bb.set("reset_odom", None)
    
    def teardown_method(self):
        """Cleanup dopo ogni test"""
        py_trees.blackboard.Blackboard.clear()
    
    # ========================================================================
    # BATTERY BEHAVIORS
    # ========================================================================
    
    def test_battery_check_success(self):
        """Test BatteryCheck con batteria sufficiente (>20%)"""
        print("\n[TEST] BatteryCheck - Batteria OK (50%)")
        
        # Setup
        self.bb.set("battery", 50.0)
        behavior = BatteryCheck()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        print("✓ BatteryCheck ritorna SUCCESS con batteria al 50%")
    
    def test_battery_check_failure(self):
        """Test BatteryCheck con batteria bassa (<=20%)"""
        print("\n[TEST] BatteryCheck - Batteria Bassa (15%)")
        
        # Setup
        self.bb.set("battery", 15.0)
        behavior = BatteryCheck()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.FAILURE, f"Expected FAILURE, got {status}"
        print("✓ BatteryCheck ritorna FAILURE con batteria al 15%")
    
    def test_go_charge_without_obstacles(self):
        """Test GoCharge senza ostacoli - ricarica completa"""
        print("\n[TEST] GoCharge - Ricarica senza ostacoli")
        
        # Setup
        self.bb.set("battery", 15.0)
        self.bb.set("obstacles", [])
        behavior = GoCharge()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        new_battery = self.bb.get("battery")
        goal_pose = self.bb.get("goal_pose")
        action = self.bb.get("plan_action")
        
        assert new_battery > 15.0, "Batteria dovrebbe aumentare"
        assert goal_pose == {'x': 0.0, 'y': 0.0, 'theta': 3.14159}, "Goal pose errato"
        # Con sensori a 999.0 (tutti liberi), dovrebbe andare dritto
        assert action == "MOVE_FORWARD", f"Expected MOVE_FORWARD, got {action}"
        print(f"✓ Batteria ricaricata da 15% a {new_battery}%")
        print(f"✓ Goal pose impostato a stazione di ricarica: {goal_pose}")
        print(f"✓ Action: {action}")
    
    def test_go_charge_with_obstacles(self):
        """Test GoCharge con ostacoli vicini"""
        print("\n[TEST] GoCharge - Ricarica con ostacoli")
        
        # Setup
        self.bb.set("battery", 20.0)
        # Imposta sensore centrale bloccato, sinistro libero
        self.bb.set("distance_left", 2.0)
        self.bb.set("distance_center", 0.3)
        self.bb.set("distance_right", 0.2)
        behavior = GoCharge()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        # Con centro bloccato e sinistra libera, dovrebbe girare a sinistra
        assert action == "TURN_LEFT", f"Expected TURN_LEFT, got {action}"
        print(f"✓ Action corretto con ostacolo: {action}")
    
    # ========================================================================
    # TARGET NAVIGATION BEHAVIORS
    # ========================================================================
    
    def test_calculate_target_first_target(self):
        """Test CalculateTarget - seleziona primo target non visitato"""
        print("\n[TEST] CalculateTarget - Primo target")
        
        # Setup
        self.bb.set("visited_targets", [])
        behavior = CalculateTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        current_target = self.bb.get("current_target")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert current_target is not None, "Target non impostato"
        assert 'name' in current_target, "Target senza nome"
        assert 'x' in current_target and 'y' in current_target, "Target senza coordinate"
        
        print(f"✓ Target selezionato: {current_target['name']}")
        print(f"  Coordinate: x={current_target['x']}, y={current_target['y']}, theta={current_target['theta']}")
    
    def test_calculate_target_skip_visited(self):
        """Test CalculateTarget - salta target già visitati"""
        print("\n[TEST] CalculateTarget - Salta visitati")
        
        # Setup - marca 'green' come visitato
        first_target_name = list(KNOWN_TARGETS.keys())[0]
        self.bb.set("visited_targets", [first_target_name])
        behavior = CalculateTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        current_target = self.bb.get("current_target")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert current_target['name'] != first_target_name, "Non dovrebbe selezionare target visitato"
        
        print(f"✓ Target visitato '{first_target_name}' saltato")
        print(f"✓ Nuovo target selezionato: {current_target['name']}")
    
    def test_calculate_target_all_visited(self):
        """Test CalculateTarget - tutti i target visitati"""
        print("\n[TEST] CalculateTarget - Tutti visitati")
        
        # Setup - marca tutti come visitati
        all_targets = list(KNOWN_TARGETS.keys())
        self.bb.set("visited_targets", all_targets)
        behavior = CalculateTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.FAILURE, f"Expected FAILURE, got {status}"
        print("✓ Ritorna FAILURE quando tutti i target sono visitati")
    
    def test_at_target_correct_color(self):
        """Test AtTarget - wall alignment and valve check"""
        print("\n[TEST] AtTarget - Aligned and red = valve")
        
        # Setup - robot near wall, aligned (left == right), red color
        target = {'name': 'red', 'x': 10.0, 'y': 15.0, 'theta': 0.0}
        self.bb.set("current_target", target)
        self.bb.set("detected_color", "red")
        self.bb.set("distance_center", 0.5)  # Near wall
        self.bb.set("distance_left", 0.6)    # Aligned
        self.bb.set("distance_right", 0.6)   # Aligned
        self.bb.set("visited_targets", [])
        self.bb.set("yolo_valve", True)       # YOLO confirms valve
        behavior = AtTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        visited = self.bb.get("visited_targets")
        reset_odom = self.bb.get("reset_odom")
        found = self.bb.get("found")
        
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert "red" in visited, "Target non marcato come visitato"
        assert found == "valve", "Dovrebbe trovare valvola"
        
        print("✓ Target 'red' marcato come visitato")
        print(f"✓ Found: {found}")
    
    def test_at_target_not_aligned(self):
        """Test AtTarget - robot not aligned with wall"""
        print("\n[TEST] AtTarget - Non allineato")
        
        # Setup - robot near wall but not aligned
        target = {'name': 'green', 'x': 2.0, 'y': 3.0, 'theta': 0.0}
        self.bb.set("current_target", target)
        self.bb.set("distance_center", 0.5)  # Near wall
        self.bb.set("distance_left", 0.3)    # Not aligned
        self.bb.set("distance_right", 0.8)   # Not aligned
        self.bb.set("visited_targets", [])
        behavior = AtTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify - should return RUNNING while aligning
        action = self.bb.get("plan_action")
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        assert action in ["TURN_LEFT", "TURN_RIGHT"], f"Expected turn action, got {action}"
        
        print(f"✓ Robot si sta allineando: {action}")
    
    def test_move_to_target(self):
        """Test MoveToTarget - calculates closest target"""
        print("\n[TEST] MoveToTarget - Closest target calculation")
        
        # Setup - robot at origin, should go to closest target (green at 2,3)
        self.bb.set("robot_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.bb.set("visited_targets", [])
        self.bb.set("obstacles", [])
        behavior = MoveToTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        goal_pose = self.bb.get("goal_pose")
        action = self.bb.get("plan_action")
        current_target = self.bb.get("current_target")
        
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        # Closest target from origin is 'green' at (2,3)
        assert current_target['name'] == 'green', f"Expected closest 'green', got {current_target['name']}"
        assert goal_pose == {'x': 2.0, 'y': 3.0, 'theta': 0.0}, f"Goal pose errato: {goal_pose}"
        
        print(f"✓ Closest target selected: {current_target['name']}")
        print(f"✓ Goal pose: {goal_pose}")
        print(f"✓ Action: {action}")
    
    def test_move_to_target_with_obstacle(self):
        """Test MoveToTarget - con ostacolo vicino"""
        print("\n[TEST] MoveToTarget - Con ostacolo")
        
        # Setup
        target = {'name': 'blue', 'x': 5.0, 'y': 9.0, 'theta': 1.57}
        self.bb.set("current_target", target)
        # Imposta sensore centrale bloccato, destra libera
        self.bb.set("distance_left", 0.2)
        self.bb.set("distance_center", 0.4)
        self.bb.set("distance_right", 2.0)
        behavior = MoveToTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        # Con centro bloccato e destra libera, dovrebbe girare a destra
        assert action == "TURN_RIGHT", f"Expected TURN_RIGHT, got {action}"
        print(f"✓ Action corretto con ostacolo: {action}")
    
    # ========================================================================
    # OBJECT DETECTION BEHAVIORS
    # ========================================================================
    
    def test_search_obj_valve_priority(self):
        """Test SearchObj - priorità valve"""
        print("\n[TEST] SearchObj - Priorità valve")
        
        # Setup - tutti gli oggetti presenti
        self.bb.set("detections", {
            "valve": True,
            "person": True,
            "obstacle": True
        })
        behavior = SearchObj()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        found = self.bb.get("found")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert found == "valve", f"Expected 'valve', got {found}"
        print("✓ Valve ha priorità massima")
    
    def test_search_obj_person_priority(self):
        """Test SearchObj - priorità person (senza valve)"""
        print("\n[TEST] SearchObj - Priorità person")
        
        # Setup
        self.bb.set("detections", {
            "person": True,
            "obstacle": True
        })
        behavior = SearchObj()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        found = self.bb.get("found")
        assert found == "person", f"Expected 'person', got {found}"
        print("✓ Person ha priorità su obstacle")
    
    def test_search_obj_obstacle_only(self):
        """Test SearchObj - solo obstacle"""
        print("\n[TEST] SearchObj - Solo obstacle")
        
        # Setup
        self.bb.set("detections", {"obstacle": True})
        behavior = SearchObj()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        found = self.bb.get("found")
        assert found == "obstacle", f"Expected 'obstacle', got {found}"
        print("✓ Obstacle rilevato correttamente")
    
    def test_search_obj_nothing_found(self):
        """Test SearchObj - nessun oggetto"""
        print("\n[TEST] SearchObj - Nessun oggetto")
        
        # Setup
        self.bb.set("detections", {})
        behavior = SearchObj()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        found = self.bb.get("found")
        assert found is None, f"Expected None, got {found}"
        print("✓ Nessun oggetto trovato (found = None)")
    
    def test_recognition_person_success(self):
        """Test RecognitionPerson - persona trovata"""
        print("\n[TEST] RecognitionPerson - SUCCESS")
        
        # Setup
        self.bb.set("found", "person")
        behavior = RecognitionPerson()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        print("✓ Ritorna SUCCESS quando found='person'")
    
    def test_recognition_person_failure(self):
        """Test RecognitionPerson - persona non trovata"""
        print("\n[TEST] RecognitionPerson - FAILURE")
        
        # Setup
        self.bb.set("found", "obstacle")
        behavior = RecognitionPerson()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.FAILURE, f"Expected FAILURE, got {status}"
        print("✓ Ritorna FAILURE quando found!='person'")
    
    def test_recognition_obstacle_success(self):
        """Test RecognitionObstacle - ostacolo trovato"""
        print("\n[TEST] RecognitionObstacle - SUCCESS")
        
        # Setup
        self.bb.set("found", "obstacle")
        behavior = RecognitionObstacle()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        print("✓ Ritorna SUCCESS quando found='obstacle'")
    
    def test_recognition_valve_success(self):
        """Test RecognitionValve - valve trovata"""
        print("\n[TEST] RecognitionValve - SUCCESS")
        
        # Setup
        self.bb.set("found", "valve")
        behavior = RecognitionValve()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        print("✓ Ritorna SUCCESS quando found='valve'")
    
    # ========================================================================
    # ACTION BEHAVIORS
    # ========================================================================
    
    def test_signal_person(self):
        """Test SignalPerson - segnalazione persona"""
        print("\n[TEST] SignalPerson")
        
        # Setup
        self.bb.set("signals", [])
        behavior = SignalPerson()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        signals = self.bb.get("signals")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert "PersonFound" in signals, "Segnale 'PersonFound' non aggiunto"
        print(f"✓ Segnale aggiunto: {signals}")
    
    def test_go_around_person(self):
        """Test GoAroundP - evita persona"""
        print("\n[TEST] GoAroundP")
        
        # Setup
        behavior = GoAroundP()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert action == "AVOID_OBSTACLE", f"Expected AVOID_OBSTACLE, got {action}"
        print(f"✓ Action impostato: {action}")
    
    def test_go_around_obstacle(self):
        """Test GoAroundO - evita ostacolo"""
        print("\n[TEST] GoAroundO")
        
        # Setup
        behavior = GoAroundO()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert action == "AVOID_OBSTACLE", f"Expected AVOID_OBSTACLE, got {action}"
        print(f"✓ Action impostato: {action}")
    
    def test_active_valve(self):
        """Test ActiveValve - attivazione valvola"""
        print("\n[TEST] ActiveValve")
        
        # Setup
        self.bb.set("signals", [])
        behavior = ActiveValve()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        signals = self.bb.get("signals")
        action = self.bb.get("plan_action")
        mission_complete = self.bb.get("mission_complete")
        
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert "ValveActivated" in signals, "Segnale 'ValveActivated' non aggiunto"
        assert action == "ACTIVATE_VALVE", f"Expected ACTIVATE_VALVE, got {action}"
        assert mission_complete == True, "Mission non marcata come completa"
        
        print(f"✓ Segnale: {signals}")
        print(f"✓ Action: {action}")
        print(f"✓ Mission complete: {mission_complete}")
    
    def test_go_home_at_home(self):
        """Test GoHome - già a casa (posizione 0,0)"""
        print("\n[TEST] GoHome - Già a casa")
        
        # Setup - robot vicino a home (0,0)
        self.bb.set("robot_position", {'x': 0.1, 'y': 0.1, 'theta': 0.0})
        behavior = GoHome()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert action == "STOP", f"Expected STOP, got {action}"
        print("✓ Riconosciuto arrivo a casa (distanza < 0.3m)")
        print(f"✓ Action: {action}")
    
    def test_go_home_moving(self):
        """Test GoHome - in movimento verso casa"""
        print("\n[TEST] GoHome - In movimento")
        
        # Setup - robot lontano da home
        self.bb.set("robot_position", {'x': 5.0, 'y': 5.0, 'theta': 0.0})
        behavior = GoHome()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        goal_pose = self.bb.get("goal_pose")
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        assert action == "MOVE_TO_GOAL", f"Expected MOVE_TO_GOAL, got {action}"
        assert goal_pose == {'x': 0.0, 'y': 0.0, 'theta': 0.0}, "Goal pose dovrebbe essere home (0,0)"
        print("✓ In movimento verso casa (distanza > 0.3m)")
        print(f"✓ Action: {action}")
        print(f"✓ Goal pose: {goal_pose}")
    
    # ========================================================================
    # INTEGRATION TEST
    # ========================================================================
    
    def test_behavior_tree_structure(self):
        """Test build_tree - verifica struttura BT"""
        print("\n[TEST] Behavior Tree - Struttura")
        
        # Execute
        tree = build_tree()
        
        # Verify
        assert tree is not None, "Tree non creato"
        assert hasattr(tree, 'tick_once'), "Tree non ha metodo tick_once"
        
        print("✓ Behavior Tree creato correttamente")
        print(f"✓ Root node type: {type(tree).__name__}")
    
    def test_full_scenario_battery_low(self):
        """Test scenario completo - batteria bassa"""
        print("\n[TEST] Scenario - Batteria bassa")
        
        # Setup
        self.bb.set("battery", 15.0)
        self.bb.set("obstacles", [])
        self.bb.set("detections", {})
        self.bb.set("visited_targets", [])
        
        tree = build_tree()
        tree.setup_with_descendants()
        
        # Execute
        tree.tick_once()
        
        # Verify - dovrebbe andare a ricaricare
        action = self.bb.get("plan_action")
        goal_pose = self.bb.get("goal_pose")
        battery = self.bb.get("battery")
        
        print(f"✓ Batteria iniziale: 15% -> finale: {battery}%")
        print(f"✓ Action: {action}")
        print(f"✓ Goal pose: {goal_pose}")
        assert battery > 15.0, "Batteria dovrebbe aumentare"
    
    def test_full_scenario_navigation(self):
        """Test scenario completo - navigazione normale"""
        print("\n[TEST] Scenario - Navigazione normale")
        
        # Setup
        self.bb.set("battery", 80.0)
        self.bb.set("obstacles", [])
        self.bb.set("detections", {})
        self.bb.set("visited_targets", [])
        self.bb.set("detected_color", None)
        
        tree = build_tree()
        tree.setup_with_descendants()
        
        # Execute
        tree.tick_once()
        
        # Verify - dovrebbe calcolare target e muoversi
        current_target = self.bb.get("current_target")
        goal_pose = self.bb.get("goal_pose")
        
        print(f"✓ Target selezionato: {current_target}")
        print(f"✓ Goal pose: {goal_pose}")
        assert current_target is not None, "Dovrebbe selezionare un target"


def run_all_tests():
    """Esegue tutti i test e stampa risultati"""
    print("=" * 70)
    print("TEST SUITE - PLAN MODULE BEHAVIORS")
    print("=" * 70)
    
    test_suite = TestPlanBehaviors()
    
    # Lista di tutti i metodi di test
    test_methods = [
        method for method in dir(test_suite) 
        if method.startswith('test_') and callable(getattr(test_suite, method))
    ]
    
    passed = 0
    failed = 0
    errors = []
    
    for test_name in test_methods:
        try:
            # Setup
            test_suite.setup_method()
            
            # Run test
            test_method = getattr(test_suite, test_name)
            test_method()
            
            # Teardown
            test_suite.teardown_method()
            
            passed += 1
            
        except AssertionError as e:
            failed += 1
            errors.append((test_name, str(e)))
            print(f"\n✗ FAILED: {test_name}")
            print(f"  Error: {e}")
            
        except Exception as e:
            failed += 1
            errors.append((test_name, str(e)))
            print(f"\n✗ ERROR: {test_name}")
            print(f"  Exception: {e}")
    
    # Risultati finali
    print("\n" + "=" * 70)
    print("RISULTATI FINALI")
    print("=" * 70)
    print(f"Test eseguiti: {passed + failed}")
    print(f"✓ Passati: {passed}")
    print(f"✗ Falliti: {failed}")
    
    if errors:
        print("\nErrori dettagliati:")
        for test_name, error in errors:
            print(f"  - {test_name}: {error}")
    
    print("=" * 70)
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
