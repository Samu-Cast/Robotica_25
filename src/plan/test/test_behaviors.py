"""
    Test Suite per behaviors.py - Charlie Robot
    
    Testa helper functions e behavior nodes per coverage >80%.
    
    Usage:
        pytest test_behaviors.py -v --cov=behaviors
"""

import math
import py_trees
from py_trees.common import Status #SUCCESS, FAILURE, RUNNING

from behaviors import (
    BatteryCheck, GoCharge, CalculateTarget, AtTarget, MoveToTarget,
    SearchObj, RecognitionPerson, RecognitionObstacle, RecognitionValve,
    SignalPerson, GoAroundP, GoAroundO, ActiveValve, InitialRetreat,
    GoToHuman, build_tree, KNOWN_TARGETS,
    #Helper functions - funzioni pure senza side effects
    calculate_best_direction, calculate_closest_target, 
    calculate_direction_to_target, check_wall_alignment
)


class TestHelperFunctions:
    """Test per helper functions"""
    
    #calculate_best_direction (IR 3-band: 0.20=far, 0.12=medium, 0.05=danger)
    def test_best_dir_all_blocked(self):
        """PANIC MODE: tutti sensori a 0.05m (danger) -> gira su se stesso"""
        assert calculate_best_direction(0.05, 0.05, 0.05) == 'AVOID_OBSTACLE'
    
    def test_best_dir_center_blocked_left_free(self):
        """Centro bloccato (0.05), sinistra libera (0.20) -> gira a sinistra"""
        assert calculate_best_direction(0.20, 0.05, 0.12) == 'TURN_LEFT'
    
    def test_best_dir_center_blocked_right_free(self):
        """Centro bloccato (0.05), destra libera (0.20) -> gira a destra"""
        assert calculate_best_direction(0.12, 0.05, 0.20) == 'TURN_RIGHT'
    
    def test_best_dir_left_too_close(self):
        """Troppo vicino a muro sinistro (0.05) -> allontanati (gira destra)"""
        assert calculate_best_direction(0.05, 0.20, 0.20) == 'TURN_RIGHT'
    
    def test_best_dir_right_too_close(self):
        """Troppo vicino a muro destro (0.05) -> allontanati (gira sinistra)"""
        assert calculate_best_direction(0.20, 0.20, 0.05) == 'TURN_LEFT'
    
    def test_best_dir_all_clear(self):
        """Tutto libero (0.20) -> vai dritto"""
        assert calculate_best_direction(0.20, 0.20, 0.20) == 'MOVE_FORWARD'
    
    #calculate_closest_target
    def test_closest_target_finds(self):
        """Trova un target quando ce ne sono disponibili"""
        result = calculate_closest_target({'x': 0, 'y': 0, 'theta': 0}, [])
        assert result is not None and 'name' in result
    
    def test_closest_target_skips_visited(self):
        """Salta target già visitati (lista visited_targets)"""
        first = list(KNOWN_TARGETS.keys())[0]
        result = calculate_closest_target({'x': 0, 'y': 0, 'theta': 0}, [first])
        assert result is None or result['name'] != first
    
    def test_closest_target_all_visited(self):
        """Ritorna None se tutti i target sono stati visitati"""
        result = calculate_closest_target({'x': 0, 'y': 0, 'theta': 0}, list(KNOWN_TARGETS.keys()))
        assert result is None
    
    #calculate_direction_to_target
    def test_direction_ahead(self):
        """Target davanti (stesso angolo) -> vai dritto"""
        assert calculate_direction_to_target({'x': 0, 'y': 0, 'theta': 0}, {'x': 5, 'y': 0}) == 'MOVE_FORWARD'
    
    def test_direction_left(self):
        """Target a sinistra (angolo positivo) -> gira sinistra"""
        assert calculate_direction_to_target({'x': 0, 'y': 0, 'theta': 0}, {'x': 1, 'y': 5}) == 'MOVE_FRONT_LEFT'
    
    def test_direction_right(self):
        """Target a destra (angolo negativo) -> gira destra"""
        assert calculate_direction_to_target({'x': 0, 'y': 0, 'theta': 0}, {'x': 1, 'y': -5}) == 'MOVE_FRONT_RIGHT'
    
    #check_wall_alignment
    def test_wall_aligned(self):
        """Sensori uguali -> allineato"""
        assert check_wall_alignment(1.0, 1.0) == True
    
    def test_wall_not_aligned(self):
        """Sensori diversi (>5cm) -> non allineato"""
        assert check_wall_alignment(1.0, 1.2) == False
    
    def test_wall_none(self):
        """Sensore None -> gestione errore, ritorna False"""
        assert check_wall_alignment(None, 1.0) == False


class TestBehaviors:
    """Test per behavior nodes"""
    
    def setup_method(self):
        """
        SETUP: eseguito PRIMA di ogni singolo test.
        
        IMPORTANTE:
        - Pulisce blackboard per isolare i test
        - Registra tutte le chiavi necessarie
        - Inizializza valori di default
        """
        py_trees.blackboard.Blackboard.clear() #Pulisci stato globale
        self.bb = py_trees.blackboard.Client(name="Test")
        
        #Registra chiavi con permesso di scrittura
        keys = ['battery', 'obstacles', 'detections', 'current_target', 'visited_targets',
                'found', 'signals', 'plan_action', 'goal_pose', 'mission_complete',
                'detected_color', 'distance_left', 'distance_center', 'distance_right',
                'robot_position', 'home_position', 'odom_correction', 'startup_complete',
                'detection_zone', 'detection_distance', 'detection_confidence',
                'color_area', 'human_position', 'targets', 'origin_offset']
        for k in keys:
            self.bb.register_key(k, access=py_trees.common.Access.WRITE)
        self._init_defaults()
    
    def _init_defaults(self):
        """Inizializza blackboard con valori sensati per i test"""
        #Sensori di distanza (999 = nessun ostacolo)
        self.bb.set("distance_left", 999.0)
        self.bb.set("distance_center", 999.0)
        self.bb.set("distance_right", 999.0)
        #Posizione robot (odometria)
        self.bb.set("robot_position", {'x': 0, 'y': 0, 'theta': 0})
        self.bb.set("visited_targets", [])
        self.bb.set("home_position", {'x': 0, 'y': 0, 'theta': 0})
        self.bb.set("odom_correction", {'dx': 0, 'dy': 0, 'dtheta': 0})
        self.bb.set("origin_offset", {'x': 0, 'y': 0, 'theta': 0})
        self.bb.set("startup_complete", True) #Salta retreat iniziale
        self.bb.set("battery", 100.0)
        self.bb.set("signals", [])
        self.bb.set("mission_complete", False)
        self.bb.set("current_target", None)
        self.bb.set("plan_action", None)
        self.bb.set("goal_pose", None)
        self.bb.set("detected_color", None)
        self.bb.set("detections", {})
        self.bb.set("found", None)
        self.bb.set("detection_zone", None)
        self.bb.set("detection_distance", 999.0)
        self.bb.set("detection_confidence", 0.0)
        self.bb.set("color_area", 0)
        self.bb.set("human_position", None)
        self.bb.set("targets", {})
        self.bb.set("obstacles", [])
    
    def teardown_method(self):
        """TEARDOWN: eseguito DOPO ogni test per pulizia"""
        py_trees.blackboard.Blackboard.clear()
    
    #Battery
    def test_battery_check_ok(self):
        """BatteryCheck: SUCCESS se batteria > 20%"""
        self.bb.set("battery", 50.0)
        b = BatteryCheck()
        b.setup_with_descendants() #Inizializza il nodo
        assert b.update() == Status.SUCCESS
    
    def test_battery_check_low(self):
        """BatteryCheck: FAILURE se batteria <= 20% (triggera ricarica)"""
        self.bb.set("battery", 15.0)
        b = BatteryCheck()
        b.setup_with_descendants()
        assert b.update() == Status.FAILURE
    
    def test_go_charge(self):
        """GoCharge: naviga verso stazione e ricarica"""
        self.bb.set("battery", 15.0)
        b = GoCharge()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("battery") > 15.0 #Batteria aumentata
    
    #Target Navigation
    def test_calculate_target(self):
        """CalculateTarget: seleziona prossimo target non visitato"""
        self.bb.set("visited_targets", [])
        b = CalculateTarget()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
        assert self.bb.get("current_target") is not None #Target impostato
    
    def test_calculate_target_all_visited(self):
        """CalculateTarget: FAILURE se tutti visitati (missione quasi completa)"""
        self.bb.set("visited_targets", list(KNOWN_TARGETS.keys()))
        b = CalculateTarget()
        b.setup_with_descendants()
        assert b.update() == Status.FAILURE
    
    def test_at_target_success(self):
        """
        AtTarget: SUCCESS quando robot è al target con colore corretto.
        
        Condizioni di successo:
        1. Sensore di prossimità <= 0.08m (PROXIMITY_THRESHOLD)
        2. Colore rilevato == nome target (es. "red" per target rosso)
        """
        target = {'name': 'red', 'x': -6.5, 'y': -3.0, 'theta': 3.14}
        self.bb.set("current_target", target)
        self.bb.set("detected_color", "red") #Colore giusto
        self.bb.set("detection_zone", "center") #Centrato
        self.bb.set("distance_center", 0.05) #Danger zone (prossimo al muro)
        b = AtTarget()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
        assert "red" in self.bb.get("visited_targets") #Marcato visitato
    
    def test_at_target_centering(self):
        """
        AtTarget: RUNNING mentre centra il colore nella camera.
        
        Se colore rilevato ma non centrato -> comanda rotazione
        """
        self.bb.set("current_target", {'name': 'green', 'x': -3.2, 'y': -5.0, 'theta': -1.57})
        self.bb.set("robot_position", {'x': -3.0, 'y': -4.5, 'theta': -1.57})
        self.bb.set("detected_color", "green")
        self.bb.set("detection_zone", "left") #Colore a sinistra -> gira sinistra
        self.bb.set("distance_center", 0.20) #Far (non ancora al target)
        b = AtTarget()
        b.setup_with_descendants()
        assert b.update() == Status.RUNNING
        assert self.bb.get("plan_action") == "MOVE_FRONT_LEFT"
    
    def test_move_to_target(self):
        """MoveToTarget: RUNNING mentre naviga verso target"""
        self.bb.set("current_target", {'name': 'blue', 'x': 5, 'y': -4, 'theta': 0})
        b = MoveToTarget()
        b.setup_with_descendants()
        assert b.update() == Status.RUNNING
    
    def test_move_to_target_obstacle(self):
        """
        MoveToTarget: obstacle avoidance con sensori ultrasonici.
        
        Sensori bloccati -> calcola direzione di fuga
        """
        self.bb.set("current_target", {'name': 'blue', 'x': 5, 'y': 0, 'theta': 0})
        self.bb.set("distance_left", 0.05) #Ostacolo a sinistra (danger)
        self.bb.set("distance_center", 0.12) #Ostacolo davanti (medium)
        self.bb.set("distance_right", 0.20) #Destra libera (far)
        b = MoveToTarget()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("plan_action") == "TURN_RIGHT" #Evita girando a destra
    
    #Object Detection
    def test_search_obj_valve(self):
        """SearchObj: rosso = valve (obiettivo missione)"""
        self.bb.set("detected_color", "red")
        b = SearchObj()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("found") == "valve"
    
    def test_search_obj_person(self):
        """SearchObj: persona ha priorità su ostacolo"""
        self.bb.set("detections", {"person": True})
        b = SearchObj()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("found") == "person"
    
    def test_recognition_person(self):
        """RecognitionPerson: SUCCESS se found == 'person'"""
        self.bb.set("found", "person")
        b = RecognitionPerson()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
    
    def test_recognition_obstacle(self):
        """RecognitionObstacle: SUCCESS se found == 'obstacle'"""
        self.bb.set("found", "obstacle")
        b = RecognitionObstacle()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
    
    def test_recognition_valve(self):
        """RecognitionValve: SUCCESS se found == 'valve'"""
        self.bb.set("found", "valve")
        b = RecognitionValve()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
    
    #Actions
    def test_signal_person(self):
        """SignalPerson: aggiunge 'PersonFound' ai segnali"""
        self.bb.set("signals", [])
        b = SignalPerson()
        b.setup_with_descendants()
        b.update()
        assert "PersonFound" in self.bb.get("signals")
    
    def test_go_around_person_left(self):
        """
        GoAroundP: evitamento smart basato su detection_zone.
        
        Persona a sinistra -> gira a destra per evitarla
        """
        self.bb.set("detection_zone", "left")
        self.bb.set("robot_position", {'x': 1, 'y': 1, 'theta': 0})
        b = GoAroundP()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("plan_action") == "TURN_RIGHT"
    
    def test_go_around_obstacle_right(self):
        """GoAroundO: ostacolo a destra -> gira a sinistra"""
        self.bb.set("detection_zone", "right")
        b = GoAroundO()
        b.setup_with_descendants()
        b.update()
        assert self.bb.get("plan_action") == "TURN_LEFT"
    
    def test_active_valve(self):
        """
        ActiveValve: attiva valvola e completa missione.
        
        - Aggiunge segnale 'ValveActivated'
        - Imposta mission_complete = True
        """
        self.bb.set("signals", [])
        b = ActiveValve()
        b.setup_with_descendants()
        b.update()
        assert "ValveActivated" in self.bb.get("signals")
        assert self.bb.get("mission_complete") == True
    
    #InitialRetreat
    def test_initial_retreat_already_done(self):
        """InitialRetreat: SUCCESS se startup già completato"""
        self.bb.set("startup_complete", True)
        b = InitialRetreat()
        b.setup_with_descendants()
        assert b.update() == Status.SUCCESS
    
    def test_initial_retreat_saves_home(self):
        """
        InitialRetreat: fase RETREAT poi ROTATE poi RESET_ORIGIN.
        Verifica che la fase 1 manda MOVE_BACKWARD.
        """
        self.bb.set("startup_complete", False)
        self.bb.set("robot_position", {'x': 1.0, 'y': 2.0, 'theta': 0.5})
        b = InitialRetreat()
        b.setup_with_descendants()
        result = b.update()
        assert result == Status.RUNNING
        assert self.bb.get("plan_action") == "MOVE_BACKWARD"
    
    
    def test_go_to_human_no_position(self):
        """GoToHuman: FAILURE se posizione umano non salvata"""
        self.bb.set("human_position", None)
        b = GoToHuman()
        b.setup_with_descendants()
        assert b.update() == Status.FAILURE
    
    def test_go_to_human_retreat(self):
        """GoToHuman: prima fase è retreat (allontanarsi dal muro)"""
        self.bb.set("human_position", {'x': 5, 'y': 5, 'theta': 0})
        b = GoToHuman()
        b.setup_with_descendants()
        assert b.update() == Status.RUNNING
        assert self.bb.get("plan_action") == "MOVE_BACKWARD"
    
    #Integration
    def test_build_tree(self):
        """build_tree: crea l'intero Behavior Tree"""
        tree = build_tree()
        assert tree is not None
        assert hasattr(tree, 'tick_once') #Metodo per eseguire un ciclo
    
    def test_full_navigation_scenario(self):
        """
        Scenario completo: tick del BT con batteria OK.
        
        Il BT dovrebbe:
        1. Verificare batteria (OK)
        2. Selezionare un target
        3. Iniziare navigazione
        """
        self.bb.set("battery", 80.0)
        tree = build_tree()
        tree.setup_with_descendants()
        tree.tick_once() #Esegui un ciclo del BT
        assert self.bb.get("current_target") is not None


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, '-v'])
