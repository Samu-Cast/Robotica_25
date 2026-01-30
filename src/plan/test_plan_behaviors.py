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
from behaviors import (
    BatteryCheck, GoCharge, CalculateTarget, AtTarget, MoveToTarget,
    SearchObj, RecognitionPerson, RecognitionObstacle, RecognitionValve,
    SignalPerson, GoAroundP, GoAroundO, ActiveValve, GoHome, InitialRetreat,
    build_tree, KNOWN_TARGETS
)


class TestPlanBehaviors:
    """Test suite completa per tutti i behavior del Plan module"""
    
    def setup_method(self):
        """Setup eseguito prima di ogni test - pulisce la blackboard"""
        py_trees.blackboard.Blackboard.clear()
        self.bb = py_trees.blackboard.Client(name="TestClient")
        
        # Registra tutte le chiavi necessarie (aggiornato per nuovo behaviors.py)
        keys = [
            'battery', 'obstacles', 'detections', 'room_color', 'home_color',
            'current_target', 'visited_targets', 'found', 'signals', 
            'plan_action', 'goal_pose', 'mission_complete', 'detected_color',
            'reset_odom', 'distance_left', 'distance_center', 'distance_right',
            'robot_position', 'home_position', 'odom_correction', 'startup_complete',
            'detection_zone', 'detection_distance', 'detection_confidence',
            'color_area'
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
        self.bb.set("reset_odom", None)
        self.bb.set("home_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.bb.set("odom_correction", {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0})
        self.bb.set("current_target", None)
        self.bb.set("startup_complete", True)  # Skip InitialRetreat in tests
        self.bb.set("detection_zone", None)
        self.bb.set("detection_distance", 999.0)
        self.bb.set("detection_confidence", 0.0)
        self.bb.set("battery", 100.0)
        self.bb.set("signals", [])
        self.bb.set("found", None)
        self.bb.set("mission_complete", False)
        self.bb.set("plan_action", None)  # Importante per MoveToTarget e altri
    
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
        # home_position is set to (0,0,0) in setup_method
        assert goal_pose == {'x': 0.0, 'y': 0.0, 'theta': 0.0}, "Goal pose errato"
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
        """Test AtTarget - robot close to target with matching color = SUCCESS"""
        print("\n[TEST] AtTarget - Close + Color Match = SUCCESS")
        
        # Setup - robot close to target (sensor < 0.30m) AND color matches target name
        # AtTarget checks: min(d_left, d_center, d_right) < 0.30 AND detected_color == target['name']
        target = {'name': 'red', 'x': -6.5, 'y': -3.0, 'theta': 3.14}
        self.bb.set("current_target", target)
        self.bb.set("robot_position", {'x': -6.3, 'y': -3.0, 'theta': 3.14})  # Very close
        self.bb.set("detected_color", "red")  # Color matches target name!
        self.bb.set("detection_zone", "center")  # Target centered in camera
        self.bb.set("distance_center", 0.25)   # < PROXIMITY_THRESHOLD (0.30)
        self.bb.set("distance_left", 0.40)
        self.bb.set("distance_right", 0.40)
        self.bb.set("visited_targets", [])
        behavior = AtTarget()
        behavior.setup_with_descendants()
        
        # Execute - should succeed on first tick since conditions are met
        status = behavior.update()
        
        # Verify
        visited = self.bb.get("visited_targets")
        found = self.bb.get("found")
        
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert "red" in visited, "Target non marcato come visitato"
        assert found == "valve", "Dovrebbe trovare valvola (red = valve)"
        
        print("✓ Target 'red' marcato come visitato")
        print(f"✓ Found: {found}")
    
    def test_at_target_not_aligned(self):
        """Test AtTarget - color detected but off-center, needs centering (RUNNING)"""
        print("\n[TEST] AtTarget - Color off-center = RUNNING")
        
        # Setup - robot NEAR target (< 1m), color matches but is on LEFT side
        # AtTarget should return RUNNING and set action to MOVE_FRONT_LEFT
        target = {'name': 'green', 'x': -3.2, 'y': -5.0, 'theta': -1.57}
        self.bb.set("current_target", target)
        self.bb.set("robot_position", {'x': -3.0, 'y': -4.5, 'theta': -1.57})  # Within 1m
        self.bb.set("detected_color", "green")  # Color matches target
        self.bb.set("detection_zone", "left")   # Color is on LEFT - needs centering
        self.bb.set("distance_center", 0.8)     # Not close enough for completion
        self.bb.set("distance_left", 0.6)
        self.bb.set("distance_right", 1.0)
        self.bb.set("visited_targets", [])
        self.bb.set("plan_action", None)
        behavior = AtTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify - should return RUNNING while centering with MOVE_FRONT_LEFT
        action = self.bb.get("plan_action")
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        assert action == "MOVE_FRONT_LEFT", f"Expected MOVE_FRONT_LEFT (color on left), got {action}"
        
        print(f"✓ Robot si sta centrando sul colore: {action}")
    
    def test_move_to_target(self):
        """Test MoveToTarget - navigates to preset target"""
        print("\n[TEST] MoveToTarget - Navigation to target")
        
        # Setup - MoveToTarget requires current_target to be set (by CalculateTarget)
        target = {'name': 'blue', 'x': 0.35, 'y': -4.0, 'theta': 0.0}
        self.bb.set("current_target", target)
        self.bb.set("robot_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.bb.set("visited_targets", [])
        self.bb.set("obstacles", [])
        self.bb.set("goal_pose", None)  # Will be set by behavior
        self.bb.set("plan_action", None)  # Will be set by behavior
        behavior = MoveToTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        goal_pose = self.bb.get("goal_pose")
        action = self.bb.get("plan_action")
        
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        assert goal_pose == target, f"Goal pose errato: {goal_pose}"
        assert action is not None, "Action non impostato"
        
        print(f"✓ Goal pose: {goal_pose}")
        print(f"✓ Action: {action}")
    
    def test_move_to_target_with_obstacle(self):
        """Test MoveToTarget - con ostacolo vicino"""
        print("\n[TEST] MoveToTarget - Con ostacolo")
        
        # Setup - MUST set plan_action BEFORE creating behavior (otherwise key conflict)
        self.bb.set("plan_action", None)
        target = {'name': 'blue', 'x': 5.0, 'y': 9.0, 'theta': 1.57}
        self.bb.set("current_target", target)
        # Imposta sensore sinistro bloccato, centro parzialmente bloccato, destra libera
        self.bb.set("distance_left", 0.2)   # < SIDE_OBSTACLE_DIST (0.4)
        self.bb.set("distance_center", 0.4) # < FRONT_OBSTACLE_DIST (0.6)
        self.bb.set("distance_right", 2.0)
        behavior = MoveToTarget()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        # Con sinistro e centro bloccati, destra libera -> TURN_RIGHT
        assert action == "TURN_RIGHT", f"Expected TURN_RIGHT, got {action}"
        print(f"✓ Action corretto con ostacolo: {action}")
    
    # ========================================================================
    # OBJECT DETECTION BEHAVIORS
    # ========================================================================
    
    def test_search_obj_valve_priority(self):
        """Test SearchObj - priorità valve (detected_color rosso)"""
        print("\n[TEST] SearchObj - Priorità valve (detected_color=red)")
        
        # Setup - red color = valve (priorità 1 in SearchObj)
        self.bb.set("detected_color", "red")
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
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert found == "valve", f"Expected 'valve', got {found}"
        print("✓ Valve ha priorità massima (detected_color=red)")
    
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
        """Test GoAroundP - evita persona based on detection_zone"""
        print("\n[TEST] GoAroundP")
        
        # Setup - person detected on left, should turn right
        self.bb.set("detection_zone", "left")
        self.bb.set("plan_action", None)
        behavior = GoAroundP()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert action == "TURN_RIGHT", f"Expected TURN_RIGHT (person on left), got {action}"
        print(f"✓ Action impostato: {action}")
    
    def test_go_around_obstacle(self):
        """Test GoAroundO - evita ostacolo based on detection_zone"""
        print("\n[TEST] GoAroundO")
        
        # Setup - obstacle detected on right, should turn left
        self.bb.set("detection_zone", "right")
        self.bb.set("plan_action", None)
        behavior = GoAroundO()
        behavior.setup_with_descendants()
        
        # Execute
        status = behavior.update()
        
        # Verify
        action = self.bb.get("plan_action")
        assert status == Status.SUCCESS, f"Expected SUCCESS, got {status}"
        assert action == "TURN_LEFT", f"Expected TURN_LEFT (obstacle on right), got {action}"
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
        """Test GoHome - già a casa ma deve fare retreat prima"""
        print("\n[TEST] GoHome - Già a casa (retreat phase)")
        
        # Setup - robot vicino a home (0,0)
        # GoHome ha una fase di retreat (3 secondi) prima di navigare
        self.bb.set("robot_position", {'x': 0.1, 'y': 0.1, 'theta': 0.0})
        self.bb.set("plan_action", None)  # Sarà settato dal behavior
        behavior = GoHome()
        behavior.setup_with_descendants()
        
        # Execute - primo tick dovrebbe essere in retreat
        status = behavior.update()
        
        # Verify - in retreat phase
        action = self.bb.get("plan_action")
        assert status == Status.RUNNING, f"Expected RUNNING (retreat phase), got {status}"
        assert action == "MOVE_BACKWARD", f"Expected MOVE_BACKWARD (retreat), got {action}"
        print("✓ GoHome in fase di retreat iniziale")
        print(f"✓ Action: {action}")
    
    def test_go_home_moving(self):
        """Test GoHome - in retreat poi movimento verso casa"""
        print("\n[TEST] GoHome - Retreat then move")
        
        # Setup - robot lontano da home
        self.bb.set("robot_position", {'x': 5.0, 'y': 5.0, 'theta': 0.0})
        self.bb.set("plan_action", None)
        self.bb.set("goal_pose", None)
        behavior = GoHome()
        behavior.setup_with_descendants()
        
        # Execute - primo tick = retreat
        status = behavior.update()
        
        # Verify - dovrebbe essere in retreat
        action = self.bb.get("plan_action")
        assert status == Status.RUNNING, f"Expected RUNNING, got {status}"
        assert action == "MOVE_BACKWARD", f"Expected MOVE_BACKWARD (retreat phase), got {action}"
        print("✓ In fase retreat (prima di navigare verso casa)")
        print(f"✓ Action: {action}")
    
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
