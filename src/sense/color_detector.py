#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # Sottoscrizione alla camera
        self.subscription = self.create_subscription(
            Image,
            '/camera_front/image',
            self.listener_callback,
            10)
        
        # Publisher per il debug (vedrai i rettangoli disegnati qui)
        self.debug_pub = self.create_publisher(Image, '/camera/color', 10)
        
        self.bridge = CvBridge()

        # --- CONFIGURAZIONE COLORI ---
        # Definisci qui i colori che vuoi cercare in formato HSV.
        # Format: 'nome': {'lower': (H, S, V), 'upper': (H, S, V), 'text_color': (B, G, R)}
        self.target_colors = {
            'Verde': {
                'lower': np.array([40, 50, 50]),   # Range HSV Verde
                'upper': np.array([80, 255, 255]),
                'draw_color': (0, 255, 0)          # Colore rettangolo (BGR)
            },
            'Blu': {
                'lower': np.array([100, 150, 0]),  # Range HSV Blu
                'upper': np.array([140, 255, 255]),
                'draw_color': (255, 0, 0)
            },
            'Rosso': {
                'lower': np.array([0, 150, 50]),  # Range HSV Rosso
                'upper': np.array([10, 255, 255]),
                'draw_color': (0, 0, 255)
            },
            # Puoi aggiungere 'Giallo', etc. qui
        }

        self.get_logger().info("Nodo Shape Detector avviato!")

    def listener_callback(self, msg):
        try:
            # 1. Converti ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Pre-processing (Fondamentale per le forme)
            # Sfocare leggermente l'immagine rimuove il rumore e rende i lati dei rettangoli più dritti
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 3. Cicla per ogni colore che vogliamo cercare
            for color_name, params in self.target_colors.items():
                
                # A. Crea la maschera per il colore specifico
                mask = cv2.inRange(hsv, params['lower'], params['upper'])
                
                # Pulizia maschera (Erosione + Dilatazione) per togliere puntini bianchi
                kernel = np.ones((5,5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

                # B. Trova i contorni nella maschera
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    
                    # Filtro area minima (ignora oggetti troppo piccoli/lontani)
                    if area > 1000:
                        
                        # C. Approssimazione Poligonale (Il trucco per le forme)
                        # Calcola il perimetro
                        perimeter = cv2.arcLength(cnt, True)
                        # Approssima il contorno a una figura geometrica semplice
                        # 0.02 * perimeter è la precisione (epsilon)
                        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

                        # D. Logica Rettangolo: Se ha 4 vertici è un quadrilatero
                        if len(approx) == 4:
                            # Ottieni le coordinate del rettangolo per disegnarlo
                            x, y, w, h = cv2.boundingRect(approx)
                            
                            # Disegna il rettangolo
                            cv2.rectangle(frame, (x, y), (x + w, y + h), params['draw_color'], 3)
                            
                            # Scrivi il nome del colore
                            cv2.putText(frame, f"{color_name} Rec", (x, y - 10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, params['draw_color'], 2)

                            # Logga l'evento (utile per far decidere al robot cosa fare)
                            self.get_logger().info(f"Trovato rettangolo {color_name} a dist stimata (area): {area}")

            # 4. Pubblica l'immagine elaborata
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Errore processing: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()