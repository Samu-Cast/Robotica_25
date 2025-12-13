#!/usr/bin/env python3
"""
Action Node - Execute Robot Actions
Receives commands from controller and executes movements/actions
"""


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ActNode(Node):
    """
    ACT node for a differential drive robot (iRobot-like).
    Executes symbolic commands by publishing cmd_vel.
    """

    def __init__(self):
        super().__init__("act_node")

        # ---- Parameters (tunable) ----
        self.linear_speed = 0.2     # m/s è la velocità lineare del robot che determina la velocità di avanzamento
        self.angular_speed = 0.6    # rad/s è la velocità angolare del robot che determina la velocità di rotazione
        self.control_rate = 10.0    # Hz frequenza del ciclo di controllo che permette di aggiornare i comandi di movimento

        # ---- Internal state ----
        self.current_state = "IDLE" # Stato iniziale del robot dal Planner

        # ---- ROS interfaces ---- 
        self.cmd_sub = self.create_subscription(
            String,
            "/act/command",
            self.command_callback,
            10
        )

        self.cmd_pub = self.create_publisher( 
            Twist,
            "/cmd_vel",
            10
        )

        # ---- Control loop timer ----
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info("ACT node started")

    # ==============================
    # Callbacks
    # ==============================
    def command_callback(self, msg: String):
        command = msg.data.upper()
        self.get_logger().info(f"Received command: {command}")
        self.current_state = command

    # ==============================
    # Control loop
    # ==============================
    def control_loop(self):
        twist = Twist()

        if self.current_state == "MOVE_FORWARD": # il robot avanza in avanti
            twist.linear.x = self.linear_speed

        elif self.current_state == "TURN_LEFT": # il robot ruota a sinistra
            twist.angular.z = self.angular_speed

        elif self.current_state == "TURN_RIGHT": # il robot ruota a destra
            twist.angular.z = -self.angular_speed

        elif self.current_state == "AVOID":     # il robot evita un ostacolo
            # simple avoidance: rotate in place
            twist.angular.z = self.angular_speed

        elif self.current_state == "STOP": #si ferms
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:  # IDLE or unknown
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist) # Pubblica il comando di velocità calcolato verso il topic /cmd_vel


#funzione main per eseguire il nodo ACT
def main(args=None): 
    rclpy.init(args=args) # Inizializza il client ROS2, rclpy è la libreria client per ROS2 in Python
    node = ActNode() # Crea l'istanza del nodo ACT 
    rclpy.spin(node) # Mantiene il nodo in esecuzione fino a quando non viene interrotto
    node.destroy_node() # Distrugge il nodo alla fine perche non è più necessario dato che il programma sta per terminare
    rclpy.shutdown()    # Chiude il client ROS2


if __name__ == "__main__":
    main()
