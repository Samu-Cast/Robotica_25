"""
Test simulazione mappa - Flusso completo delle decisioni.
Simula il robot nelle varie stanze e mostra le azioni del BT.
"""

import py_trees                      #libreria Behavior Tree
from py_trees.common import Status   #stati: SUCCESS, FAILURE, RUNNING
import sys                           #per modificare path
import os                            #per gestire percorsi file

#calcola path alla cartella plan/ (parent di tests/)
plan_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, plan_dir)         #aggiunge plan/ al path Python

from plan_node import build_behavior_tree, BLACKBOARD_KEYS  #importa da plan_node

py_trees.blackboard.Blackboard.enable_activity_stream()     #abilita log blackboard


# MAPPA (da map.yaml)
MAP = {
    'corridor': {'color': [128, 128, 128], 'has': None},      # Home/Grigio
    'room_a':   {'color': [180, 80, 80],   'has': None},   # Rosso - VALVE
    'room_b':   {'color': [80, 80, 180],   'has': 'person'},  # Blu
    'room_c':   {'color': [80, 180, 80],   'has': 'valve'},      # Verde
}


def setup_simulation():
    """Setup blackboard con dati mappa."""
    bb = py_trees.blackboard.Client(name="MapSim")  #crea client blackboard
    for key in BLACKBOARD_KEYS:                      #registra tutte le chiavi
        bb.register_key(key, access=py_trees.common.Access.WRITE)
    
    #crea lista targets dalle stanze (escluso corridor)
    targets = []
    for room_id, data in MAP.items():        #itera su stanze
        if room_id != 'corridor':            #esclude home
            targets.append({
                'id': room_id,               #nome stanza
                'color': data['color'],      #colore RGB
                'person': data['has'] == 'person'  #True se ha persona
            })
    
    #imposta stato iniziale (robot parte nel corridor)
    bb.set("battery", 100.0)                          #batteria piena
    bb.set("pos", {'x': 0, 'y': 0, 'theta': 0})       #posizione iniziale
    bb.set("obstacles", [1.0, 1.0, 1.0])              #nessun ostacolo
    bb.set("detections", {})                          #nessuna detection
    bb.set("room_color", MAP['corridor']['color'])    #colore stanza = grigio
    bb.set("home_color", MAP['corridor']['color'])    #SALVA colore home!
    bb.set("targets", targets)                        #lista stanze da visitare
    bb.set("target", None)                            #nessun target ancora
    bb.set("signals", [])                             #nessun segnale
    bb.set("valve_found", False)                      #missione non completata
    bb.set("cmd_vel", {})                             #robot fermo
    
    return bb


def get_room_name(color):
    """Trova nome stanza dal colore RGB."""
    for name, data in MAP.items():                                  #itera stanze
        diff = sum(abs(c - d) for c, d in zip(color, data['color'])) / 3  #differenza media
        if diff < 30:                                               #tolleranza colore
            return name
    return "unknown"


def simulate():
    """Simula missione completa."""
    print("=" * 60)
    print(" SIMULAZIONE MAPPA")
    print("=" * 60)
    
    print("\nMAPPA:")
    for name, data in MAP.items():
        obj = f" [{data['has'].upper()}]" if data['has'] else ""
        home = " (HOME)" if name == 'corridor' else ""
        print(f"   {name:10s}: {data['color']}{obj}{home}")
    
    bb = setup_simulation()
    tree = build_behavior_tree()
    tree.setup_with_descendants()
    
    print(f"\nTargets da visitare: {[t['id'] for t in bb.get('targets')]}")
    print("\n" + "-" * 60)
    print(" ESECUZIONE")
    print("-" * 60 + "\n")
    
    # Simula visita stanza per stanza
    for room_id in ['room_a', 'room_b', 'room_c']:
        room_data = MAP[room_id]
        
        # 1. Tick -> seleziona target
        tree.tick_once()
        target = bb.get("target")
        if target:
            print(f"[>] Seleziono target: {target['id']}")
        
        # 2. Simula movimento (muove verso target)
        print(f"[>] Muovo verso {room_id}...")
        tree.tick_once()
        
        # 3. Arrivo nella stanza (cambia colore)
        bb.set("room_color", room_data['color'])
        print(f"[>] Arrivato a {room_id}")
        
        # 4. Simula detection YOLO
        if room_data['has'] == 'valve':
            bb.set("detections", {'valve': True})
            print(f"[!] VALVE RILEVATA in {room_id}!")
        elif room_data['has'] == 'person':
            bb.set("detections", {'person': True})
            bb.set("found", "person")
            print(f"[!] PERSONA RILEVATA in {room_id}!")
        else:
            bb.set("detections", {})
            print(f"[-] Niente di interessante in {room_id}")
        
        # 5. Tick per processare detection
        tree.tick_once()
        
        # 6. Se valve trovata -> missione completata
        if bb.get("valve_found"):
            print(f"\n[OK] VALVE ATTIVATA!")
            break
        
        # 7. Reset per prossima stanza
        bb.set("detections", {})
        bb.set("found", None)
        print()
    
    # Ritorno a casa
    if bb.get("valve_found"):
        print(f"\n[>] Torno a casa...")
        bb.set("room_color", MAP['corridor']['color'])
        tree.tick_once()
        print(f"[OK] ARRIVATO A CASA!")
    
    print("\n" + "-" * 60)
    print(" RISULTATO")
    print("-" * 60)
    print(f"Valve found: {bb.get('valve_found')}")
    print(f"Segnali: {bb.get('signals')}")
    print("=" * 60)


if __name__ == "__main__":
    simulate()