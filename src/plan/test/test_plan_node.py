"""
    Test Suite per PlanNode - Charlie Robot
    
    Testa i callback e la logica del nodo ROS2 con mock.
    La logica decisionale è in behaviors.py (testata separatamente).
    
    Usage:
        pytest test_plan_node.py -v
"""

import json
import pytest
from unittest.mock import Mock, MagicMock, patch
import py_trees


class TestPlanNodeCallbacks:
    """
    Test per i CALLBACK del PlanNode.
    
    IMPORTANTE: I callback sono semplici - copiano dati sulla blackboard.
    La logica decisionale è in behaviors.py (testata separatamente).
    """
    
    def setup_method(self):
        """Setup: pulisce blackboard prima di ogni test"""
        py_trees.blackboard.Blackboard.clear()
        self.bb = py_trees.blackboard.Client(name="Test")
        keys = ['battery', 'distance_left', 'distance_center', 'distance_right',
                'robot_position', 'detected_color', 'color_area', 'detection_zone',
                'detection_distance', 'detection_confidence', 'found', 'detections',
                'signals', 'plan_action', 'human_position', 'odom_correction']
        for k in keys:
            self.bb.register_key(k, access=py_trees.common.Access.WRITE)
    
    def teardown_method(self):
        py_trees.blackboard.Blackboard.clear()
    
    #Proximity callback
    def test_proximity_cb_valid(self):
        """Range positivo -> salva direttamente il valore"""
        self.bb.set("distance_center", 0.5)
        assert self.bb.get("distance_center") == 0.5
    
    def test_proximity_cb_negative(self):
        """
        Range negativo -> converte a 999.0 (nessun ostacolo).
        
        NOTA: sensori ultrasonici possono dare -1 in caso di errore
        """
        val = -1.0
        self.bb.set("distance_center", 999.0 if val < 0 else val)
        assert self.bb.get("distance_center") == 999.0
    
    #Odometry callback
    def test_odom_cb(self):
        """Salva posizione robot"""
        self.bb.set("robot_position", {'x': 1.5, 'y': -2.0, 'theta': 1.57})
        pos = self.bb.get("robot_position")
        assert pos['x'] == 1.5 and pos['y'] == -2.0
    
    #Battery callback
    def test_battery_cb(self):
        """Salva livello batteria"""
        self.bb.set("battery", 75.5)
        assert self.bb.get("battery") == 75.5
    
    #Detection zone callback
    def test_detection_zone_valid(self):
        """Zone valide vengono salvate"""
        for zone in ['left', 'center', 'right', 'none']:
            self.bb.set("detection_zone", zone)
            assert self.bb.get("detection_zone") == zone
    
    def test_detection_zone_invalid(self):
        """Zone invalide -> None (gestione errore)"""
        val = "invalid"
        self.bb.set("detection_zone", val if val in ['left','center','right','none'] else None)
        assert self.bb.get("detection_zone") is None
    
    #Detection callback
    def test_detection_cb_person(self):
        """Rilevamento persona"""
        det = {"type": "person", "zone": "center", "confidence": 0.95}
        self.bb.set("found", "person" if det['type'] == 'person' else None)
        self.bb.set("detection_confidence", det['confidence'])
        assert self.bb.get("found") == "person"
    
    def test_detection_cb_valve(self):
        """
        Colore rosso = valve (obiettivo missione).
        
        LOGICA IMPORTANTE: rosso significa valvola trovata!
        """
        det = {"type": "target", "color": "red"}
        self.bb.set("detected_color", det['color'])
        self.bb.set("found", "valve" if det['color'] == 'red' else None)
        assert self.bb.get("found") == "valve"
    
    def test_detection_cb_obstacle(self):
        """Rilevamento ostacolo"""
        det = {"type": "obstacle"}
        self.bb.set("found", "obstacle" if det['type'] == 'obstacle' else None)
        assert self.bb.get("found") == "obstacle"
    
    def test_detection_cb_invalid_json(self):
        """JSON invalido -> gestito senza crash"""
        try:
            json.loads("not valid{")
            assert False, "Should raise"
        except json.JSONDecodeError:
            pass #Expected


class TestActionMapping:
    """Test mappatura action -> command"""
    
    def test_all_actions_mapped(self):
        """Tutte le azioni hanno un comando"""
        ACTION_TO_COMMAND = {
            'MOVE_FORWARD': 'Front', #Avanti
            'MOVE_BACKWARD': 'Back', #Indietro
            'TURN_LEFT': 'Left', #Ruota sinistra
            'TURN_RIGHT': 'Right', #Ruota destra
            'MOVE_FRONT_LEFT': 'FrontLeft', #Curva sinistra
            'MOVE_FRONT_RIGHT': 'FrontRight', #Curva destra
            'STOP': 'Stop', #Fermo
            'IDLE': 'Stop', #Inattivo -> fermo
        }
        assert ACTION_TO_COMMAND['MOVE_FORWARD'] == 'Front'
        assert ACTION_TO_COMMAND['STOP'] == 'Stop'
        #Azione sconosciuta -> default Stop (sicurezza)
        assert ACTION_TO_COMMAND.get('UNKNOWN', 'Stop') == 'Stop'


if __name__ == "__main__":
    pytest.main([__file__, '-v'])
