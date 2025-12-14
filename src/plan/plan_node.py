"""
    Charlie Robot - Plan Node
    Nodo ROS2 che esegue il Behavior Tree per navigazione e decisioni.

    Architettura:
        - Riceve dati da Sense (odometry, obstacles, detections)
        - Esegue Behavior Tree per logica decisionale
        - Pubblica comandi ad Act (cmd_vel)
    
    NOTA: Le coordinate (pos) sono approssimative e rappresentative,
    NON sono riferimenti assoluti per la navigazione.
    La navigazione si basa su colori stanza, ostacoli e detections YOLO.
"""

import json
import math

# ROS2 imports (opzionali per test locali)
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray, String, Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object  # Placeholder per ereditarietà

import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status, ParallelPolicy

# BLACKBOARD - memoria condivisa tra nodi BT
# I valori vengono aggiornati dai subscribers ROS2 (dati reali da Sense)
BLACKBOARD_KEYS = [
    'battery',      # float: % batteria (da /charlie/battery)
    'pos',          # dict: {x,y,theta} APPROSSIMATIVO (da /charlie/odometry)
    'obstacles',    # list: [front,left,right] distanze (da /charlie/obstacles)
    'detections',   # dict: output YOLO (da /charlie/detections)
    'room_color',   # list: [R,G,B] colore stanza rilevato
    'home_color',   # list: [R,G,B] colore home (salvato all'avvio)
    'found',        # str: oggetto trovato ("person", "obstacle", None)
    'target',       # dict: target corrente
    'targets',      # list: target rimanenti
    'signals',      # list: segnali emessi
    'valve_found',  # bool: missione completata
    'cmd_vel',      # dict: {linear,angular} comando per Act
]


def navigate_to_color(current_color, target_color, obstacles, linear_speed=0.3, angular_speed=0.5):
    """
        Helper: naviga verso una stanza con colore target.
        Ritorna: (arrived: bool, cmd: dict)
    """
    # Controlla se siamo arrivati (colore corrisponde)
    diff = sum(abs(c - t) for c, t in zip(current_color, target_color)) / 3
    if diff < 30:
        return True, {'linear': 0.0, 'angular': 0.0}  # Arrivati!
    
    # Non arrivati -> naviga evitando ostacoli
    front = obstacles[0] if obstacles else 1.0
    if front < 0.5:  # ostacolo davanti
        cmd = {'linear': 0.0, 'angular': angular_speed}
    else:
        cmd = {'linear': linear_speed, 'angular': 0.0}
    
    return False, cmd


class BatteryRequired(py_trees.behaviour.Behaviour):
    """CONDIZIONE: batteria sufficiente (>20%)?"""

    def __init__(self):
        super().__init__(name="BatteryRequired") #nome nodo albero
        self.bb = self.attach_blackboard_client(name=self.name) #collega nodo a blackboard (client che legge/scrive)
        self.bb.register_key("battery", access=py_trees.common.Access.READ) #registra chiave blackboard (lettura)
    
    def update(self): #ad ogni tik deve restirutire qualcosa
        batt = self.bb.get("battery") #legge
        if batt > 20: #condizione
            self.feedback_message = f"Battery OK: {batt:.0f}%"
            return Status.SUCCESS #se > 20: successo
        self.feedback_message = f"Battery LOW: {batt:.0f}%"
        return Status.FAILURE #altrimenti fallimento


class GoCharge(py_trees.behaviour.Behaviour):
    """
    AZIONE: vai a ricaricare.
    Fasi: 1. Vai a home  2. Ricarica  3. Torna dove eri
    """
    
    def __init__(self):
        super().__init__(name="GoCharge") #nome nodo BT
        self.bb = self.attach_blackboard_client(name=self.name) #client blackboard
        self.bb.register_key("battery", access=py_trees.common.Access.WRITE) #batteria (legge+scrive)
        self.bb.register_key("room_color", access=py_trees.common.Access.READ) #colore stanza corrente
        self.bb.register_key("home_color", access=py_trees.common.Access.READ) #colore home (stazione ricarica)
        self.bb.register_key("cmd_vel", access=py_trees.common.Access.WRITE) #comandi velocità
        self.bb.register_key("obstacles", access=py_trees.common.Access.READ) #ostacoli [front,left,right]
        self.state = "GOING_HOME" #stato corrente della macchina a stati
        self.saved_color = None #colore stanza da cui siamo partiti (per tornare)
    
    def initialise(self):
        """Chiamato quando il nodo viene attivato (prima di update)."""
        self.state = "GOING_HOME" #reset stato
        self.saved_color = self.bb.get("room_color") #SALVA dove eravamo!
    
    def update(self):
        """Chiamato ad ogni tick. Gestisce la macchina a stati."""
        current = self.bb.get("room_color") or [0, 0, 0] #colore stanza attuale
        home = self.bb.get("home_color") or [128, 128, 128] #colore home
        obstacles = self.bb.get("obstacles") or [1.0, 1.0, 1.0] #distanze ostacoli
        
        # === STATO 1: GOING_HOME ===
        if self.state == "GOING_HOME":
            #usa helper per navigare verso home
            arrived, cmd = navigate_to_color(current, home, obstacles)
            self.bb.set("cmd_vel", cmd) #pubblica comando velocità
            if arrived: #arrivato a home?
                self.state = "CHARGING" #passa a stato CHARGING
                self.feedback_message = "At charging station"
            else:
                self.feedback_message = "Going to charging station..."
            return Status.RUNNING #continua (non finito)
        
        # === STATO 2: CHARGING ===
        elif self.state == "CHARGING":
            batt = min(100, self.bb.get("battery") + 30) #ricarica +30%
            self.bb.set("battery", batt) #aggiorna batteria
            self.bb.set("cmd_vel", {'linear': 0.0, 'angular': 0.0}) #fermo
            if batt >= 80: #ricarica completata?
                self.state = "RETURNING" #passa a stato RETURNING
                self.feedback_message = f"Charged to {batt:.0f}%, returning..."
            else:
                self.feedback_message = f"Charging... {batt:.0f}%"
            return Status.RUNNING #continua
        
        # === STATO 3: RETURNING ===
        elif self.state == "RETURNING":
            if self.saved_color:
                #naviga verso posizione salvata
                arrived, cmd = navigate_to_color(current, self.saved_color, obstacles)
                self.bb.set("cmd_vel", cmd)
                if arrived: #tornati?
                    self.state = "GOING_HOME" #reset per prossima volta
                    self.feedback_message = "Returned from charging"
                    return Status.SUCCESS #FINITO! Missione ricarica completata
            self.feedback_message = "Returning..."
            return Status.RUNNING #continua
        
        return Status.FAILURE #stato sconosciuto


class CalculateTarget(py_trees.behaviour.Behaviour):
    """AZIONE: seleziona prossimo target dalla lista."""
    
    def __init__(self):
        super().__init__(name="CalculateTarget") #nome nodo albero
        self.bb = self.attach_blackboard_client(name=self.name) #collega nodo a blackboard (client che legge/scrive)
        self.bb.register_key("targets", access=py_trees.common.Access.WRITE) #registra chiavi blackboard (scrittura)
        self.bb.register_key("target", access=py_trees.common.Access.WRITE) #registra chiave blackboard (scrittura)
    
    def update(self):  #chiamato ogni tick del BT
        targets = self.bb.get("targets")  #legge lista target rimanenti
        
        if not targets:  #lista vuota?
            self.feedback_message = "No more targets"  #messaggio debug
            return Status.FAILURE  #fallisce: nessun target da visitare
        
        target = targets.pop(0)
        
        self.bb.set("target", target)    #scrive target corrente nella blackboard
        self.bb.set("targets", targets)  #aggiorna lista (senza quello appena preso)
        
        self.feedback_message = f"Target: {target.get('id', '?')}"  #feedback per debug
        return Status.SUCCESS  #successo: target selezionato


class AtTarget(py_trees.behaviour.Behaviour):
    """
        CONDIZIONE: robot arrivato al target?
        Basato su rilevamento colore stanza, NON su coordinate precise.
    """
    
    def __init__(self):
        super().__init__(name="AtTarget")  #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)  #client blackboard
        self.bb.register_key("target", access=py_trees.common.Access.READ)      #legge target corrente
        self.bb.register_key("room_color", access=py_trees.common.Access.READ)  #legge colore stanza rilevato
    
    def update(self):  #chiamato ogni tick
        target = self.bb.get("target")  #legge target dalla blackboard
        
        if not target:  #nessun target impostato?
            return Status.FAILURE  #fallisce
        
        #legge colore stanza ATTUALE (rilevato da camera/YOLO)
        current_color = self.bb.get("room_color") or [0, 0, 0]  #default nero se None
        
        #legge colore ATTESO del target 
        target_color = target.get("color", [0, 0, 0])
        
        #calcola differenza media tra colori RGB
        diff = sum(abs(c - t) for c, t in zip(current_color, target_color)) / 3
        
        if diff < 30:  #tolleranza: differenza < 30 = stesso colore
            self.feedback_message = f"At {target.get('id', '?')} (color match)"
            return Status.SUCCESS  #siamo al target!
        
        self.feedback_message = f"Not at target (color diff: {diff:.0f})"
        return Status.FAILURE  #colore diverso, non siamo al target


class MoveToTarget(py_trees.behaviour.Behaviour):
    """
        AZIONE: muove verso target.
        Navigazione basata su ostacoli e direzione generale, NON coordinate precise.
    """
    
    def __init__(self, linear_speed: float = 0.3, angular_speed: float = 0.5):
        super().__init__(name="MoveToTarget")  #nome nodo
        self.linear_speed = linear_speed    #velocità avanti (m/s)
        self.angular_speed = angular_speed  #velocità rotazione (rad/s)
        self.bb = self.attach_blackboard_client(name=self.name)  #client blackboard
        self.bb.register_key("obstacles", access=py_trees.common.Access.READ)   #legge ostacoli
        self.bb.register_key("cmd_vel", access=py_trees.common.Access.WRITE)    #scrive comandi velocità
    
    def update(self):  #chiamato ogni tick
        #legge distanze ostacoli [front, left, right] in metri
        obstacles = self.bb.get("obstacles") or [1.0, 1.0, 1.0]  #default: nessun ostacolo
        front, left, right = obstacles[0], obstacles[1], obstacles[2]
        
        cmd = {'linear': 0.0, 'angular': 0.0}  #comando velocità iniziale: fermo
        
        #LOGICA NAVIGAZIONE REATTIVA:
        if front < 0.5:  #ostacolo davanti entro 50cm?
            #ruota verso il lato più libero
            #left > right → ruota a sinistra (angular positivo)
            #right > left → ruota a destra (angular negativo)
            cmd['angular'] = self.angular_speed if left > right else -self.angular_speed
            self.feedback_message = "Avoiding obstacle"
        else:  #nessun ostacolo frontale
            cmd['linear'] = self.linear_speed  #vai avanti
            
            #correzione leggera: se un lato è più libero, sterza leggermente
            if abs(left - right) > 0.3:  #differenza significativa
                cmd['angular'] = 0.2 if left > right else -0.2
            
            self.feedback_message = "Moving forward"
        
        self.bb.set("cmd_vel", cmd)  #scrive comando nella blackboard
        return Status.RUNNING  #sempre RUNNING: continua muoversi finché AtTarget non dice SUCCESS


class SearchObj(py_trees.behaviour.Behaviour):
    """
    AZIONE: cerca oggetti durante movimento.
    Legge detections YOLO e scrive in 'found' cosa ha trovato.
    Eseguito in parallelo con MoveToTarget.
    """
    
    def __init__(self):
        super().__init__(name="SearchOnPath") #nome nodo BT
        self.bb = self.attach_blackboard_client(name=self.name) #client blackboard
        self.bb.register_key("detections", access=py_trees.common.Access.READ) #legge YOLO
        self.bb.register_key("found", access=py_trees.common.Access.WRITE) #scrive cosa trovato
    
    def update(self):
        detections = self.bb.get("detections") or {} #legge detections da YOLO
        
        if detections.get("person"): #YOLO ha rilevato persona?
            self.bb.set("found", "person") #segnala persona trovata
            self.feedback_message = "Person detected!"
            return Status.SUCCESS
        
        if detections.get("obstacle"): #YOLO ha rilevato ostacolo?
            self.bb.set("found", "obstacle") #segnala ostacolo trovato
            self.feedback_message = "Obstacle detected!"
            return Status.SUCCESS
        
        self.bb.set("found", None) #nessun oggetto
        self.feedback_message = "Path clear"
        return Status.SUCCESS #sempre SUCCESS (non blocca Parallel)


class RecognitionPerson(py_trees.behaviour.Behaviour):
    """
    CONDIZIONE: verifica se 'found' == 'person'.
    Usato nel Fallback per decidere se gestire persona.
    """
    
    def __init__(self):
        super().__init__(name="RecognitionPerson") #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ) #legge found
    
    def update(self):
        if self.bb.get("found") == "person": #trovata persona?
            self.feedback_message = "Person found"
            return Status.SUCCESS #si: SUCCESS → esegue SignalPerson + GoAroundP
        return Status.FAILURE #no: FAILURE → prova RecognitionObstacle


class SignalPerson(py_trees.behaviour.Behaviour):
    """
    AZIONE: aggiunge segnale 'PersonFound' alla lista signals.
    Usato per notificare che è stata trovata una persona.
    """
    
    def __init__(self):
        super().__init__(name="SignalPerson") #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE) #scrive signals
    
    def update(self):
        signals = self.bb.get("signals") or [] #legge lista segnali
        signals.append("PersonFound") #aggiunge segnale
        self.bb.set("signals", signals) #scrive
        self.feedback_message = "Person signaled!"
        return Status.SUCCESS


class GoAroundP(py_trees.behaviour.Behaviour):
    """
    AZIONE: evita persona ruotando a sinistra.
    Pubblica cmd_vel con angular positivo.
    """
    
    def __init__(self):
        super().__init__(name="GoAroundP") #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("cmd_vel", access=py_trees.common.Access.WRITE) #scrive velocità
    
    def update(self):
        self.bb.set("cmd_vel", {'linear': 0.0, 'angular': 0.5}) #ruota sinistra
        self.feedback_message = "Avoiding person"
        return Status.SUCCESS


class RecognitionObstacle(py_trees.behaviour.Behaviour):
    """
    CONDIZIONE: verifica se 'found' == 'obstacle'.
    Usato nel Fallback dopo RecognitionPerson.
    """
    
    def __init__(self):
        super().__init__(name="RecognitionObstacle") #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ) #legge found
    
    def update(self):
        if self.bb.get("found") == "obstacle": #trovato ostacolo?
            self.feedback_message = "Obstacle found"
            return Status.SUCCESS #si: SUCCESS → esegue GoAroundO
        return Status.FAILURE #no: FAILURE


class GoAroundO(py_trees.behaviour.Behaviour):
    """
    AZIONE: evita ostacolo ruotando a destra.
    Pubblica cmd_vel con angular negativo.
    """
    
    def __init__(self):
        super().__init__(name="GoAroundO") #nome nodo
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("cmd_vel", access=py_trees.common.Access.WRITE) #scrive velocità
    
    def update(self):
        self.bb.set("cmd_vel", {'linear': 0.0, 'angular': -0.5}) #ruota destra
        self.feedback_message = "Avoiding obstacle"
        return Status.SUCCESS


class RecognitionValve(py_trees.behaviour.Behaviour):
    """CONDIZIONE: valvola rilevata da YOLO?"""
    
    def __init__(self):
        super().__init__(name="RecognitionValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("detections", access=py_trees.common.Access.READ)
    
    def update(self):  #chiamato ogni tick
        #legge detections dalla blackboard (dati che arrivano da YOLO via Sense)
        detections = self.bb.get("detections") or {}  #default: dict vuoto
        
        #controlla se c'è chiave "valve" con valore True
        if detections.get("valve"):
            self.feedback_message = "VALVE DETECTED!"
            return Status.SUCCESS
        
        self.feedback_message = "No valve detected"
        return Status.FAILURE


class ActiveValve(py_trees.behaviour.Behaviour):
    """AZIONE: attiva valvola e segnala completamento missione."""
    
    def __init__(self):
        super().__init__(name="ActiveValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
        self.bb.register_key("valve_found", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.bb.get("signals") or []
        signals.append("ValveActivated")
        self.bb.set("signals", signals)
        self.bb.set("valve_found", True)
        self.feedback_message = "Valve activated!"
        return Status.SUCCESS


class GoHome(py_trees.behaviour.Behaviour):
    """AZIONE: torna verso home usando navigate_to_color."""
    
    def __init__(self):
        super().__init__(name="GoHome")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("obstacles", access=py_trees.common.Access.READ)
        self.bb.register_key("room_color", access=py_trees.common.Access.READ)
        self.bb.register_key("home_color", access=py_trees.common.Access.READ)
        self.bb.register_key("cmd_vel", access=py_trees.common.Access.WRITE)
        
    def update(self):
        current = self.bb.get("room_color") or [0, 0, 0]
        home = self.bb.get("home_color") or [128, 128, 128]
        obstacles = self.bb.get("obstacles") or [1.0, 1.0, 1.0]
        
        arrived, cmd = navigate_to_color(current, home, obstacles)
        self.bb.set("cmd_vel", cmd)
        
        if arrived:
            self.feedback_message = "Home reached!"
            return Status.SUCCESS
        
        self.feedback_message = "Going home..."
        return Status.RUNNING



def build_behavior_tree():
    """
    Costruisce il Behavior Tree secondo schema bTree.md.
    
    FLUSSO:
        1. Se batteria bassa → ricarica
        2. Seleziona prossimo target dalla lista
        3. Muovi verso target (in parallelo cerca persona/ostacolo)
        4. Se persona → segnala e evita
        5. Se ostacolo → evita
        6. Cerca valvola con YOLO
        7. Se no valvola → FAILURE → Retry ricomincia da (2)
        8. Se valvola → attiva → torna home → SUCCESS
    """
    
    # === FallBack0: Battery management ===
    fallback_battery = Selector("FallBack0_Battery", memory=False, children=[
        BatteryRequired(),
        GoCharge()
    ])
    
    # === Sequence2: Handle Person ===
    sequence_person = Sequence("Sequence2_Person", memory=False, children=[
        RecognitionPerson(),
        SignalPerson(),
        GoAroundP()
    ])
    
    # === Sequence3: Handle Obstacle ===
    sequence_obstacle = Sequence("Sequence3_Obstacle", memory=False, children=[
        RecognitionObstacle(),
        GoAroundO()
    ])
    
    # === FallBack2: Person / Obstacle ===
    fallback_found = Selector("FallBack2_HandleFound", memory=False, children=[
        sequence_person,
        sequence_obstacle
    ])
    
    # === Sequence1: SearchObj → FallBack2 ===
    sequence_search = Sequence("Sequence1_Search", memory=False, children=[
        SearchObj(),
        fallback_found
    ])
    
    # === Decorator: FailureIsSuccess per non bloccare il Parallel ===
    decorator_search = decorators.FailureIsSuccess(
        name="Decorator1_SearchLoop",
        child=sequence_search
    )
    
    # === Parallel0: MoveToTarget + Search ===
    # SuccessOnAll: aspetta che MoveToTarget arrivi al target
    parallel_move = Parallel(
        "Parallel0_MoveAndSearch",
        policy=ParallelPolicy.SuccessOnAll(),
        children=[MoveToTarget(), decorator_search]
    )
    
    # === FallBack1: AtTarget? / Parallel0 ===
    fallback_goto = Selector("FallBack1_GoToTarget", memory=False, children=[
        AtTarget(),
        parallel_move
    ])
    
    # === Sequence0: Main sequence ===
    sequence_main = Sequence("Sequence0_Main", memory=True, children=[
        fallback_battery,    # Battery check
        CalculateTarget(),   # Select next target
        fallback_goto,       # Go to target (with search)
        RecognitionValve(),  # Check valve (FAILURE → loop retry)
        ActiveValve(),       # Activate valve
        GoHome()             # Return home
    ])
    
    # === Decorator0: LoopUntilSuccess ===
    root = decorators.Retry(
        name="Decorator0_LoopUntilSuccess",
        child=sequence_main,
        num_failures=-1
    )
    
    return root



class PlanNode(Node):
    """
    Nodo ROS2 Plan.
    Riceve dati reali da Sense, esegue BT, pubblica comandi per Act.
    
    SUBSCRIBERS (riceve da Sense):
        /charlie/odometry   → posizione (approssimativa!)
        /charlie/obstacles  → distanze ostacoli [front, left, right]
        /charlie/detections → JSON da YOLO {valve, room_color, ...}
        /charlie/battery    → livello batteria %
    
    PUBLISHERS (invia ad Act):
        /charlie/cmd_vel    → comandi velocità Twist
        /charlie/status     → stato BT in JSON
    """
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Charlie Plan Node ===')
        
        # Behavior Tree
        self.tree = build_behavior_tree()
        
        # Blackboard
        self.blackboard = py_trees.blackboard.Client(name="PlanNode")
        for key in BLACKBOARD_KEYS:
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
        self._init_blackboard()
        
        self.tree.setup_with_descendants()
        
        # === SUBSCRIBERS (dati da Sense) ===
        self.create_subscription(Odometry, '/charlie/odometry', self._odom_cb, 10)
        self.create_subscription(Float32MultiArray, '/charlie/obstacles', self._obstacles_cb, 10)
        self.create_subscription(String, '/charlie/detections', self._detections_cb, 10)
        self.create_subscription(Float32, '/charlie/battery', self._battery_cb, 10)
        
        # === PUBLISHERS (comandi per Act) ===
        self.cmd_pub = self.create_publisher(Twist, '/charlie/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/charlie/status', 10)
        
        #timer che esegue _tick ogni 100ms (10 Hz = 10 volte al secondo)
        self.create_timer(0.1, self._tick)
        
        self.get_logger().info('Plan Node ready')
    
    def _init_blackboard(self):
        """Valori iniziali blackboard."""
        self.blackboard.set("battery", 100.0)
        self.blackboard.set("pos", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.blackboard.set("obstacles", [1.0, 1.0, 1.0])
        self.blackboard.set("detections", {})
        self.blackboard.set("room_color", [128, 128, 128])
        self.blackboard.set("home_color", [128, 128, 128])  # Salva colore home
        self.blackboard.set("found", None)                  # Oggetto trovato
        self.blackboard.set("target", None)
        self.blackboard.set("targets", [
            {'id': 'Room1', 'color': [255, 0, 0]},
            {'id': 'Room2', 'color': [0, 255, 0]},
            {'id': 'Room3', 'color': [0, 0, 255]},
        ])
        self.blackboard.set("signals", [])
        self.blackboard.set("valve_found", False)
        self.blackboard.set("cmd_vel", {'linear': 0.0, 'angular': 0.0})
    
    # === CALLBACKS: chiamate automaticamente quando arrivano messaggi ===
    
    def _odom_cb(self, msg: Odometry):
        """Callback odometria: aggiorna posizione (APPROSSIMATIVA, solo per log)."""
        pos = msg.pose.pose.position  #posizione (x, y, z)
        orient = msg.pose.pose.orientation  #orientamento (quaternion)
        #conversione semplificata quaternion → yaw (angolo z)
        theta = 2 * math.atan2(orient.z, orient.w)
        self.blackboard.set("pos", {'x': pos.x, 'y': pos.y, 'theta': theta})
    
    def _obstacles_cb(self, msg: Float32MultiArray):
        """Callback ostacoli: aggiorna distanze [front, left, right] in metri."""
        if len(msg.data) >= 3:  #verifica che ci siano almeno 3 valori
            self.blackboard.set("obstacles", list(msg.data[:3]))
    
    def _detections_cb(self, msg: String):
        """Callback detections: aggiorna dati YOLO {valve, room_color, ...}."""
        try:
            data = json.loads(msg.data)  #parse JSON
            self.blackboard.set("detections", data)  #salva tutto
            if 'room_color' in data:  #estrae colore stanza se presente
                self.blackboard.set("room_color", data['room_color'])
        except json.JSONDecodeError:
            pass  #ignora JSON malformato
    
    def _battery_cb(self, msg: Float32):
        """Callback batteria: aggiorna livello in percentuale."""
        self.blackboard.set("battery", float(msg.data))
    
    
    def _tick(self):
        """
        Loop principale: esegue tick BT e pubblica comandi.
        Chiamato dal timer ogni 100ms (10 Hz).
        """
        #esegue UN tick del Behavior Tree
        self.tree.tick_once()
        
        #legge comando velocità dalla blackboard (scritto dai nodi BT)
        cmd = self.blackboard.get("cmd_vel")
        
        #crea messaggio Twist per Act
        twist = Twist()
        twist.linear.x = cmd.get('linear', 0.0)   #velocità avanti/indietro
        twist.angular.z = cmd.get('angular', 0.0)  #velocità rotazione
        self.cmd_pub.publish(twist)  #invia a /charlie/cmd_vel
        
        # Pubblica status
        status = String()
        status.data = json.dumps({
            'status': str(self.tree.status),
            'valve_found': self.blackboard.get("valve_found"),
            'target': self.blackboard.get("target"),
        })
        self.status_pub.publish(status)



def main(args=None):
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available.")
        print("Run inside Docker container or use: python3 tests/test_plan_node.py")
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
