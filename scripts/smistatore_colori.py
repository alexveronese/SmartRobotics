#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import subprocess 

class SmistatoreColori(Node):
    def __init__(self):
        super().__init__('smistatore_colori')

        # Publisher per Braccio e Pinza
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        
        self.camera_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            qos_profile
        )
        self.bridge = CvBridge()

        self.stato_attuale = "ATTESA_IMMAGINE"
        self.cubo_target = None
        
        self.get_logger().info("🤖 Cervello di Smistamento Avviato. In attesa del video...")

        # Dizionario degli angoli precalcolati per le posizioni chiave [joint_1, joint_2, joint_3, joint_4]
        self.posizioni = {
            "HOME": [0.0, 0.0, 0.0, 0.0],
            "PRE_PRESA_ROSSO": [0.25, 0.5, 0.5, -1.0],
            "PRESA_ROSSO": [0.25, 0.8, 0.8, -1.57],
            "SCARICO_ROSSO": [1.57, 0.2, 0.5, -0.5],
            
            "PRE_PRESA_VERDE": [0.0, 0.5, 0.5, -1.0],
            "PRESA_VERDE": [0.0, 0.8, 0.8, -1.57],
            "SCARICO_VERDE": [-1.57, 0.2, 0.5, -0.5],
            
            "PRE_PRESA_BLU": [-0.25, 0.5, 0.5, -1.0],
            "PRESA_BLU": [-0.25, 0.8, 0.8, -1.57],
            "SCARICO_BLU": [3.14, 0.4, 0.2, -0.5]
        }
        
        subprocess.Popen(['ros2', 'run', 'rqt_image_view', 'rqt_image_view'])
        
        self.get_logger().info("🖥️ Finestra di visualizzazione aperta.")

    def muovi_braccio(self, target_name, duration_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        point = JointTrajectoryPoint()
        point.positions = self.posizioni[target_name]
        point.time_from_start.sec = int(duration_sec)
        msg.points.append(point)
        self.arm_pub.publish(msg)
        time.sleep(duration_sec + 0.5)

    def muovi_pinza(self, apri=True):
        msg = JointTrajectory()
        msg.joint_names = ['joint_left_finger', 'joint_right_finger']
        point = JointTrajectoryPoint()
        if apri:
            point.positions = [0.02, 0.02]  # Aperta
        else:
            point.positions = [0.005, 0.005]  # Chiusa
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        time.sleep(1.5)

    def image_callback(self, msg):
        # Aggiungiamo un contatore per elaborare solo 1 frame su 10 (riduce il carico del 90%)
        if not hasattr(self, 'frame_count'): self.frame_count = 0
        self.frame_count += 1
        if self.frame_count % 10 != 0:
            return 

        if self.stato_attuale != "ATTESA_IMMAGINE":
            return

        # 1. Converti l'immagine da ROS a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Definisci i limiti dei colori (HSV)
        rosso_basso = np.array([0, 150, 100])
        rosso_alto = np.array([10, 255, 255])
        verde_basso = np.array([40, 150, 100])
        verde_alto = np.array([80, 255, 255])
        blu_basso = np.array([100, 150, 100])
        blu_alto = np.array([140, 255, 255])

        # 3. Trova i pixel per ogni colore
        mask_rosso = cv2.inRange(hsv_image, rosso_basso, rosso_alto)
        mask_verde = cv2.inRange(hsv_image, verde_basso, verde_alto)
        mask_blu = cv2.inRange(hsv_image, blu_basso, blu_alto)

        # 4. Decidi quale cubo prendere (ordine di priorità: Rosso, Verde, Blu)
        if cv2.countNonZero(mask_rosso) > 500:
            self.cubo_target = "ROSSO"
        elif cv2.countNonZero(mask_verde) > 500:
            self.cubo_target = "VERDE"
        elif cv2.countNonZero(mask_blu) > 500:
            self.cubo_target = "BLU"
        else:
            self.get_logger().info("Tavolo pulito! Lavoro terminato.")
            self.stato_attuale = "FINE"
            return

        self.get_logger().info(f"👁️ Rilevato cubo {self.cubo_target}! Inizio smistamento...")
        self.stato_attuale = "SMISTAMENTO"
        
        # Mostra cosa vede la telecamera
        #cv2.imshow("Vista Robot", cv_image)
        #cv2.waitKey(1)

        # 5. Esegui la Macchina a Stati per lo smistamento
        self.esegui_ciclo_pick_and_place()

    def esegui_ciclo_pick_and_place(self):
        # A. Prepara la pinza e vai sopra il cubo
        self.muovi_pinza(apri=True)
        self.get_logger().info("Avvicinamento...")
        self.muovi_braccio(f"PRE_PRESA_{self.cubo_target}", 2.0)
        
        # B. Scendi e afferra
        self.get_logger().info("Presa!")
        self.muovi_braccio(f"PRESA_{self.cubo_target}", 1.5)
        self.muovi_pinza(apri=False)
        
        # C. Solleva
        self.muovi_braccio(f"PRE_PRESA_{self.cubo_target}", 1.5)
        
        # D. Vai al cesto corrispondente e rilascia
        self.get_logger().info("In transito verso il cesto...")
        self.muovi_braccio(f"SCARICO_{self.cubo_target}", 2.5)
        self.get_logger().info("Rilascio!")
        self.muovi_pinza(apri=True)
        
        # E. Torna a casa e preparati per il prossimo
        self.muovi_braccio("HOME", 2.0)
        self.stato_attuale = "ATTESA_IMMAGINE"

def main(args=None):
    rclpy.init(args=args)
    nodo = SmistatoreColori()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()