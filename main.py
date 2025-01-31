import sys
import wiringpi #Permettra d'interagir avec les GPIOs du RPI
import subprocess #Premettra de faire des "system call"
import rclpy
from rclpy.node import Node #Importation de la librairie ROS
from std_msgs.msg import Int32 #Importation des types de message "Int32"

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
    
        # Subscribers pour les valeurs des encodeurs
        self .sub_left_encoder = self.create_subscription(Int32, 'encodeur_A',
            self .left_encoder_callback, 10)
        self .sub_right_encoder = self.create_subscription(Int32, 'encodeur_B',
            self .right_encoder_callback, 10)

        # Variables de vitesse des moteurs
        self .vitesse_mg = 512 # Vitesse initiale pour le moteur gauche
        self .vitesse_md = 512 # Vitesse initiale pour le moteur droit
        self.current_speed = 512
        self.speed_increment = 100
        self.increasing = True

        # Configuration des entres- sorties (GPIOs) vis la librairie wiringpi
        wiringpi .wiringPiSetup() # Initialisation de la librairie wiringpi pour le controle des GPIOs
        wiringpi .pinMode(25, 1) # Configurer la GPIO 25 en sortie
        wiringpi .pinMode(3, 1) # Configurer la GPIO 3 en sortie
        wiringpi .pinMode(27, 1) # Configurer la GPIO 27 en sortie
        wiringpi . digitalWrite (25, 1) # Ecrire des 1 logiques ...
        wiringpi . digitalWrite (3, 1)
        wiringpi . digitalWrite (27, 1)

        # Temporisateur 1Hz ROS2 pour le controle des moteurs
        self .timer = self .create_timer(1.0, self .control_motors)
        self .get_logger(). info ("Le node de controle des moteurs est en execution.")
    
    def left_encoder_callback(self , msg):
        """Callback pour l 'encodeur gauche."""
        self .get_logger(). info ( f"Valeur de l 'encodeur gauche: {msg.data}")
        self .vitesse_mg = msg.data # Exemple: Assigner la valeur de l'encodeur au moteur (a changer)
    
    def right_encoder_callback(self, msg):
        """Callback pour l 'encodeur droit. """
        self .get_logger(). info ( f"Valeur de l 'encodeur droit: {msg.data}")
        self .vitesse_md = msg.data # Exemple: Assigner la valeur de l'encodeur au moteur (a changer)
    
    def control_motors(self ):
        """Envoyer les commandes PWM aux moteurs."""
        if self.increasing:
            self.current_speed += self.speed_increment
            if self.current_speed >= 1012:
                self.increasing = False
        else:
            self.current_speed -= self.speed_increment
            if self.current_speed <= 12:
                self.increasing = True

        commande = f"echo \"raspberry\" | sudo -S gpio pwm 26 {self.current_speed}; sudo -S gpio pwm 23 {self.current_speed}"
        self .get_logger(). info ( f"Commande envoyee: {commande}")
        subprocess. call (commande, shell=True)
    
    def cleanup( self ):
        """Reconfigurer les pins GPIO avant la fermeture"""
        self .get_logger(). info ("Desactivation des moteurs and reconfiguration des GPIOs...")
        wiringpi . digitalWrite (25, 0)
        wiringpi . digitalWrite (3, 0)
        wiringpi . digitalWrite (27, 0)



def main(args=None):
    rclpy . init (args=args)
    node = MotorControlNode()
    try :
        rclpy .spin(node)
    except KeyboardInterrupt:
        node.get_logger().info ("Fermeture du node de controle des moteurs...")
    finally :
        node.cleanup()
        rclpy .shutdown()


if __name__ == '__main__':
    main()
