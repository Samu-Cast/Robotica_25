"""
    Test Suite per Sense Module - Charlie Robot
    
    Questa classe di test valida le funzionalità del modulo Sense
    senza dipendenze ROS2 (test isolati delle funzioni helper).
    
    Usage:
        python3 test_sense_node.py
        
    Oppure con pytest:
        pytest test_sense_node.py -v
"""

import sys
import math


class MockSenseNode:
    """
    Mock del SenseNode per testare le funzioni helper senza ROS2.
    Replica solo le costanti e i metodi di utilità.
    """
    
    # Minimum bbox area to consider a detection (filters out far detections)
    MIN_BBOX_AREA = 2000
    
    # Distance estimation calibration
    BBOX_DISTANCE_SCALE = 40000.0  # pixels² at 1 meter
    
    def __init__(self):
        self.image_width = 640
        self.image_height = 480
        self.ir_data = {
            'front': 0,
            'front_left': 0,
            'front_right': 0,
        }
    
    def _calculate_bbox_area(self, bbox, format='xywh'):
        """Calculate area of bounding box"""
        if format == 'xywh':
            x, y, w, h = bbox
            return abs(w * h)
        else:
            x1, y1, x2, y2 = bbox
            return abs(x2 - x1) * abs(y2 - y1)
    
    def _get_bbox_center(self, bbox, format='xywh'):
        """Get center of bounding box"""
        if format == 'xywh':
            x, y, w, h = bbox
            return (x + w / 2, y + h / 2)
        else:
            x1, y1, x2, y2 = bbox
            return ((x1 + x2) / 2, (y1 + y2) / 2)
    
    def _is_centered(self, bbox, format='xywh'):
        """Check if bbox center is in the middle third of the image"""
        cx, cy = self._get_bbox_center(bbox, format)
        left_third = self.image_width / 3
        right_third = 2 * self.image_width / 3
        return left_third <= cx <= right_third
    
    def _get_horizontal_zone(self, bbox, format='xywh'):
        """Determine which horizontal zone the detection is in"""
        cx, cy = self._get_bbox_center(bbox, format)
        left_third = self.image_width / 3
        right_third = 2 * self.image_width / 3
        
        if cx < left_third:
            return 'left'
        elif cx > right_third:
            return 'right'
        else:
            return 'center'
    
    def _get_proximity_for_zone(self, zone):
        """Get the IR intensity value for a given zone"""
        if zone == 'left':
            return self.ir_data.get('front_left', 0)
        elif zone == 'right':
            return self.ir_data.get('front_right', 0)
        else:
            return self.ir_data.get('front', 0)
    
    def _estimate_distance_from_bbox(self, bbox_area):
        """Estimate distance based on bounding box area"""
        if bbox_area <= 0:
            return float('inf')
        return math.sqrt(self.BBOX_DISTANCE_SCALE / bbox_area)


class TestSenseNode:
    """Test suite per le funzioni helper del Sense module"""
    
    def setup_method(self):
        """Setup eseguito prima di ogni test"""
        self.node = MockSenseNode()
    
    # ========================================================================
    # BBOX AREA CALCULATION
    # ========================================================================
    
    def test_bbox_area_xywh(self):
        """Test calcolo area bbox formato xywh"""
        print("\n[TEST] BBox Area - Formato xywh")
        
        bbox = [100, 100, 50, 80]  # x, y, width, height
        area = self.node._calculate_bbox_area(bbox, format='xywh')
        
        expected = 50 * 80  # 4000
        assert area == expected, f"Expected {expected}, got {area}"
        print(f"✓ Area calcolata: {area} px²")
    
    def test_bbox_area_xyxy(self):
        """Test calcolo area bbox formato xyxy (YOLO)"""
        print("\n[TEST] BBox Area - Formato xyxy")
        
        bbox = [100, 100, 200, 180]  # x1, y1, x2, y2
        area = self.node._calculate_bbox_area(bbox, format='xyxy')
        
        expected = (200 - 100) * (180 - 100)  # 100 * 80 = 8000
        assert area == expected, f"Expected {expected}, got {area}"
        print(f"✓ Area calcolata: {area} px²")
    
    # ========================================================================
    # BBOX CENTER CALCULATION
    # ========================================================================
    
    def test_bbox_center_xywh(self):
        """Test calcolo centro bbox formato xywh"""
        print("\n[TEST] BBox Center - Formato xywh")
        
        bbox = [100, 100, 50, 80]  # x, y, width, height
        cx, cy = self.node._get_bbox_center(bbox, format='xywh')
        
        expected_cx = 100 + 50 / 2  # 125
        expected_cy = 100 + 80 / 2  # 140
        assert cx == expected_cx, f"Expected cx={expected_cx}, got {cx}"
        assert cy == expected_cy, f"Expected cy={expected_cy}, got {cy}"
        print(f"✓ Centro: ({cx}, {cy})")
    
    def test_bbox_center_xyxy(self):
        """Test calcolo centro bbox formato xyxy"""
        print("\n[TEST] BBox Center - Formato xyxy")
        
        bbox = [100, 100, 200, 180]  # x1, y1, x2, y2
        cx, cy = self.node._get_bbox_center(bbox, format='xyxy')
        
        expected_cx = (100 + 200) / 2  # 150
        expected_cy = (100 + 180) / 2  # 140
        assert cx == expected_cx, f"Expected cx={expected_cx}, got {cx}"
        assert cy == expected_cy, f"Expected cy={expected_cy}, got {cy}"
        print(f"✓ Centro: ({cx}, {cy})")
    
    # ========================================================================
    # HORIZONTAL ZONE DETECTION
    # ========================================================================
    
    def test_zone_left(self):
        """Test rilevamento zona sinistra"""
        print("\n[TEST] Zone Detection - Left")
        
        # Bbox con centro nella parte sinistra (< 213 px su 640)
        bbox = [50, 100, 60, 80]  # centro a x=80
        zone = self.node._get_horizontal_zone(bbox, format='xywh')
        
        assert zone == 'left', f"Expected 'left', got '{zone}'"
        print(f"✓ Zona rilevata: {zone}")
    
    def test_zone_center(self):
        """Test rilevamento zona centrale"""
        print("\n[TEST] Zone Detection - Center")
        
        # Bbox con centro nella parte centrale (213-426 px su 640)
        bbox = [280, 100, 80, 80]  # centro a x=320
        zone = self.node._get_horizontal_zone(bbox, format='xywh')
        
        assert zone == 'center', f"Expected 'center', got '{zone}'"
        print(f"✓ Zona rilevata: {zone}")
    
    def test_zone_right(self):
        """Test rilevamento zona destra"""
        print("\n[TEST] Zone Detection - Right")
        
        # Bbox con centro nella parte destra (> 426 px su 640)
        bbox = [500, 100, 80, 80]  # centro a x=540
        zone = self.node._get_horizontal_zone(bbox, format='xywh')
        
        assert zone == 'right', f"Expected 'right', got '{zone}'"
        print(f"✓ Zona rilevata: {zone}")
    
    # ========================================================================
    # IS CENTERED CHECK
    # ========================================================================
    
    def test_is_centered_true(self):
        """Test oggetto centrato"""
        print("\n[TEST] Is Centered - True")
        
        bbox = [280, 100, 80, 80]  # centro a x=320 (metà immagine)
        is_centered = self.node._is_centered(bbox, format='xywh')
        
        assert is_centered == True, f"Expected True, got {is_centered}"
        print("✓ Oggetto correttamente identificato come centrato")
    
    def test_is_centered_false(self):
        """Test oggetto non centrato"""
        print("\n[TEST] Is Centered - False")
        
        bbox = [50, 100, 60, 80]  # centro a x=80 (sinistra)
        is_centered = self.node._is_centered(bbox, format='xywh')
        
        assert is_centered == False, f"Expected False, got {is_centered}"
        print("✓ Oggetto correttamente identificato come non centrato")
    
    # ========================================================================
    # PROXIMITY FOR ZONE
    # ========================================================================
    
    def test_proximity_zone_mapping(self):
        """Test mappatura zona -> sensore IR"""
        print("\n[TEST] Proximity Zone Mapping")
        
        # Setup IR data
        self.node.ir_data = {
            'front': 500,
            'front_left': 1000,
            'front_right': 300,
        }
        
        # Test each zone
        assert self.node._get_proximity_for_zone('left') == 1000
        assert self.node._get_proximity_for_zone('center') == 500
        assert self.node._get_proximity_for_zone('right') == 300
        
        print("✓ left -> front_left (1000)")
        print("✓ center -> front (500)")
        print("✓ right -> front_right (300)")
    
    # ========================================================================
    # DISTANCE ESTIMATION
    # ========================================================================
    
    def test_distance_estimation_close(self):
        """Test stima distanza per oggetto vicino (bbox grande)"""
        print("\n[TEST] Distance Estimation - Close Object")
        
        # Bbox grande = oggetto vicino
        bbox_area = 40000  # ~1 metro secondo calibrazione
        distance = self.node._estimate_distance_from_bbox(bbox_area)
        
        expected = 1.0  # sqrt(40000/40000) = 1m
        assert abs(distance - expected) < 0.01, f"Expected ~{expected}m, got {distance}m"
        print(f"✓ Distanza stimata: {distance:.2f}m (area={bbox_area} px²)")
    
    def test_distance_estimation_far(self):
        """Test stima distanza per oggetto lontano (bbox piccolo)"""
        print("\n[TEST] Distance Estimation - Far Object")
        
        # Bbox piccolo = oggetto lontano
        bbox_area = 10000  # ~2 metri
        distance = self.node._estimate_distance_from_bbox(bbox_area)
        
        expected = 2.0  # sqrt(40000/10000) = 2m
        assert abs(distance - expected) < 0.01, f"Expected ~{expected}m, got {distance}m"
        print(f"✓ Distanza stimata: {distance:.2f}m (area={bbox_area} px²)")
    
    def test_distance_estimation_zero_area(self):
        """Test stima distanza con area zero"""
        print("\n[TEST] Distance Estimation - Zero Area")
        
        distance = self.node._estimate_distance_from_bbox(0)
        
        assert distance == float('inf'), f"Expected inf, got {distance}"
        print("✓ Area zero -> distanza infinita")
    
    # ========================================================================
    # MIN BBOX AREA FILTER
    # ========================================================================
    
    def test_min_bbox_area_filter(self):
        """Test filtro area minima per detection"""
        print("\n[TEST] Min BBox Area Filter")
        
        small_bbox = [100, 100, 40, 40]  # area = 1600 < MIN (2000)
        large_bbox = [100, 100, 60, 60]  # area = 3600 > MIN (2000)
        
        small_area = self.node._calculate_bbox_area(small_bbox, format='xywh')
        large_area = self.node._calculate_bbox_area(large_bbox, format='xywh')
        
        assert small_area < self.node.MIN_BBOX_AREA, "Small bbox dovrebbe essere filtrato"
        assert large_area >= self.node.MIN_BBOX_AREA, "Large bbox dovrebbe passare"
        
        print(f"✓ Small bbox ({small_area} px²) < MIN ({self.node.MIN_BBOX_AREA} px²) -> filtered")
        print(f"✓ Large bbox ({large_area} px²) >= MIN ({self.node.MIN_BBOX_AREA} px²) -> accepted")


def run_all_tests():
    """Esegue tutti i test e stampa risultati"""
    print("=" * 70)
    print("TEST SUITE - SENSE MODULE")
    print("=" * 70)
    
    test_suite = TestSenseNode()
    
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
