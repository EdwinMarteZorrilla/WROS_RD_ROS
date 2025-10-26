#!/usr/bin/env python3
import time
import subprocess
import json
import paho.mqtt.client as mqtt
import os

BROKERS = ["192.168.149.171", "192.168.149.1"]
BROKER_PORT = 1883
TOPIC_FIRE = "alerta/fuego"
NAVIGATION_SCRIPT = "/programs/navigation.py"
POSE_FILE = "/tmp/robot_pose.json"

class FireFighterRobot:
    def __init__(self):
        self.state = "IDLE"
        self.fire_detected = False
        self.last_rpi = None
        print("Robot iniciado. Estado: IDLE")
        self.mqtt_clients = []
        for ip in BROKERS:
            client = mqtt.Client()
            client.on_connect = self.on_connect
            client.on_message = self.on_message
            try:
                client.connect(ip, BROKER_PORT, 60)
                print(f"[MQTT] Conectado al broker {ip}:{BROKER_PORT}")
            except Exception as e:
                print(f"[MQTT] Error al conectar al broker {ip}: {e}")
            client.loop_start()
            self.mqtt_clients.append(client)

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_FIRE)
        print(f"[MQTT] Subscrito a {TOPIC_FIRE}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            label = data.get("label")
            rpi_id = data.get("rpi_id")
            if label in ["fire","cigar","fireball"]:
                print(f"[ALERTA] Fuego detectado por {rpi_id}")
                self.fire_detected = True
                self.last_rpi = rpi_id
        except Exception as e:
            print("[MQTT] Error procesando mensaje:", e)

    def run(self):
        while True:
            if self.state == "IDLE":
                self.idle_state()
            elif self.state == "NAVIGATION":
                self.navigation_state()
            elif self.state == "FIRE_FIGHTING":
                self.fire_fighting_state()
            elif self.state == "DONE":
                self.done_state()
                break
            time.sleep(0.1)

    def idle_state(self):
        print("[STATE] IDLE esperando alerta...")
        if self.fire_detected:
            self.state = "NAVIGATION"

    def navigation_state(self):
        print(f"[STATE] NAVIGATION ejecutando navegación hacia {self.last_rpi}")

        # Load last known robot position (if available)
        if os.path.exists(POSE_FILE):
            with open(POSE_FILE, "r") as f:
                pose = json.load(f)
            print(f"[NAVIGATION] Usando última posición: {pose}")
            # Convert odom position (meters) to grid indices
            start = (int(round(pose["y"] / 0.2)), int(round(pose["x"] / 0.2)))
        else:
            print("[NAVIGATION] Sin posición previa -> usando (0,0) como inicio")
            start = (0, 0)

        # Define goal coordinates based on RPi that triggered alert
        if self.last_rpi == "RPI_1":
            goal = (4, 0)
        elif self.last_rpi == "RPI_2":
            goal = (2, 2)
        else:
            goal = (5, 5)

        print(f"[NAVIGATION] Navegando de {start} hacia {goal}")

        try:
            subprocess.run([
                "python3", NAVIGATION_SCRIPT,
                "--start_row", str(start[0]),
                "--start_col", str(start[1]),
                "--goal_row", str(goal[0]),
                "--goal_col", str(goal[1])
            ], check=True)

            # After navigation, read pose
            if os.path.exists(POSE_FILE):
                with open(POSE_FILE, "r") as f:
                    pose = json.load(f)
                print(f"[NAVIGATION] Posición final: {pose}")

            # Go back home (origin)
            subprocess.run([
                "python3", NAVIGATION_SCRIPT,
                "--start_row", str(goal[0]),
                "--start_col", str(goal[1]),
                "--goal_row", "0",
                "--goal_col", "0"
            ], check=True)

            self.state = "FIRE_FIGHTING"

        except Exception as e:
            print(f"[STATE] Error ejecutando navegación: {e}")
            self.state = "DONE"


    def fire_fighting_state(self):
         print("[STATE] FIRE_FIGHTING: Activando bomba de agua y servos simult  neamente...")

        try:
            # Lanzar la bomba y el servo en paralelo
            pump_process = subprocess.Popen(["python3", FIRE_FIGHT_SCRIPT])
            servo_process = subprocess.Popen(["python3", SERVO_SCRIPT])

            print("[ACTION] Bomba y servo iniciados al mismo tiempo.")

            # Esperar un tiempo mientras trabajan (puedes ajustar este valor)
            time.sleep(5)

            # Finalizar ambos procesos
            pump_process.terminate()
            servo_process.terminate()

            print("[ACTION] Operaci  n de extinci  n completada.")

        except Exception as e:
            print(f"[ERROR] Error ejecutando scripts simult  neos: {e}")

    def done_state(self):
        print("[STATE] DONE: Misión completa. Regresando a IDLE.")
        self.fire_detected = False
        self.last_rpi = None
        self.state = "IDLE"

if __name__ == "__main__":
    robot = FireFighterRobot()
    robot.run()
