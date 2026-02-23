import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from enum import Enum




class ActState(Enum):
    STOP = 0
    LEFT = 1
    FRONT_LEFT = 2
    FRONT = 3
    FRONT_RIGHT = 4
    RIGHT = 5
    BACK = 6
    
    #SEARCH_VALVE = 3
    #ACTIVATE_VALVE = 4
    

class ActNode(Node):

    def __init__(self):
        super().__init__('act_node')

        # Publisher verso il robot (Twist per iCreate3 fisico)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber dal PLAN che miinvia Sam
        self.plan_sub = self.create_subscription(
            String,
            '/plan/command',
            self.plan_callback,
            10
        )

        # Stato di default è stop eprche parte fermo
        self.state = ActState.STOP

        # Gestione SearchValve 
        self.search_start_time = None
        self.search_angular_speed = 0.4  # rad/s velocita con cui ruota durante la ricerca

        # Gestione ActivateValve (simulazione gira intorno a se stesso per 10 secondi)
        self.activate_start_time = None
        self.activate_duration = 10.0     # secondi
        self.activate_speed = 1.0         # rad/s

        # Timer di controllo (10 Hz) 
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("ACT NODE AVVIATO (Twist - Robot Fisico)")

    #funzione generale per inviare comandi di velocità al robot in base allo stato
    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    #definisce il comportamento in base al comando ricevuto dal PLAN
    def plan_callback(self, msg):
        command = msg.data #prende il comando come stringa
        prev_state = self.state

        if command == "Front":
            self.state = ActState.FRONT
        elif command == "Back":
            self.state = ActState.BACK
        elif command == "Right":
            self.state = ActState.RIGHT
        elif command == "Left":
            self.state = ActState.LEFT
        elif command == "FrontLeft":
            self.state = ActState.FRONT_LEFT
        elif command == "FrontRight":
            self.state = ActState.FRONT_RIGHT
        elif command == "Stop":
            self.state = ActState.STOP
        
        # DEBUG: Log cambio di stato
        if self.state != prev_state:
            state_emoji = {
                ActState.STOP: "🛑",
                ActState.FRONT: "⬆️",
                ActState.BACK: "⬇️",
                ActState.LEFT: "⬅️",
                ActState.RIGHT: "➡️",
                ActState.FRONT_LEFT: "↖️",
                ActState.FRONT_RIGHT: "↗️",
            }
            emoji = state_emoji.get(self.state, "❓")
            self.get_logger().info(f"[DEBUG][Act] {emoji} Comando: {command} | Stato: {prev_state.name} → {self.state.name}")
        
        '''
        elif command == "SearchValve":
            if self.state != ActState.SEARCH_VALVE:
                self.search_start_time = self.get_clock().now() #se alla fine lo devo fare io la decisione di quanto dura a cercare
                # self.get_logger().info("PLAN → SEARCH_VALVE (inizio ricerca)") 
            self.state = ActState.SEARCH_VALVE
            # qui pero la gestione di quando deve smettere di cercare la valvola lo decide il plan, lo invia a me, control loop e il robot fa altro

        elif command == "ActivateValve":
            if self.state != ActState.ACTIVATE_VALVE:
                self.activate_start_time = self.get_clock().now()
                # self.get_logger().info("PLAN → ACTIVATE_VALVE (inizio attivazione)")
            self.state = ActState.ACTIVATE_VALVE
        '''
        
    
    def control_loop(self):
        # STOP
        if self.state == ActState.STOP:
            self.send_cmd(0.0, 0.0)

        # FRONT: avanti continuo
        elif self.state == ActState.FRONT:
            self.send_cmd(0.2, 0.0)
            
        # BACK: indietro continuo
        elif self.state == ActState.BACK:
            self.send_cmd(-0.2, 0.0)

        #LEFT: rotazione continua finché PLAN non cambia
        elif self.state == ActState.LEFT:
            self.send_cmd(0.0, +0.3)
        
        #RIGHT: rotazione continua finché PLAN non cambia
        elif self.state == ActState.RIGHT:
            self.send_cmd(0.0, -0.3)
        
        # FRONT_LEFT: leggermente a sinistra (avanzando)
        elif self.state == ActState.FRONT_LEFT:
            self.send_cmd(0.10, 0.30)
        
        # FRONT_RIGHT: leggermente a destra (avanzando)
        elif self.state == ActState.FRONT_RIGHT:
            self.send_cmd(0.10, -0.30)

    '''
        # SEARCH_VALVE: rotazione continua con memoria 
        elif self.state == ActState.SEARCH_VALVE:
            self.send_cmd(0.0, self.search_angular_speed)

        # ACTIVATE_VALVE: bisogna attiva la valcola quindi la rotazione è su se stesso per 10 secondi 
        elif self.state == ActState.ACTIVATE_VALVE:
            if not self.activate_finished():
                self.send_cmd(0.0, self.activate_speed)
            else:
                self.send_cmd(0.0, 0.0)
                self.state = ActState.STOP
                self.get_logger().info("Attivazione valvola completata")
    
    
    
    
    #funzione per verificare se l'attivazione della valvola è completata praticamente se so passati 10 secondi
    def activate_finished(self):
        if self.activate_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self.activate_start_time).nanoseconds / 1e9
        return elapsed >= self.activate_duration

    def reset_search(self):
        self.search_start_time = None

    def reset_activate(self):
        self.activate_start_time = None
    '''


def main(args=None):
    rclpy.init(args=args)
    node = ActNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
