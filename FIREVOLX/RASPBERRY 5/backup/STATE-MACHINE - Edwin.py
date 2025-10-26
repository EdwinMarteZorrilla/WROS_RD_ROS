#!/usr/bin/env python3
import time
import subprocess
import json
import threading
import paho.mqtt.client as mqtt

# -----------------------------
# CONFIGURACI ^sN MQTT
# -----------------------------
BROKERS = ["192.168.149.171", "192.168.149.1"]  # Lista de brokers
BROKER_PORT = 1883
TOPIC_FIRE = "alerta/fuego"

# Ruta al script de navegaci  n
NAVIGATION_SCRIPT = "/programs"

# -----------------------------
# FireFighterRobot: M  quina de estado
# -----------------------------
class FireFighterRobot:
    def __init__(self):
        self.state = "IDLE"
        self.fire_detected = False
        self.last_rpi = None
        print(" ^= ^v Robot iniciado. Estado actual: IDLE")

        # Conectar a todos los brokers
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

    # -----------------------------
    # MQTT Callbacks
    # -----------------------------
    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Conectado. Subscrito al t  pico: {TOPIC_FIRE}")
        client.subscribe(TOPIC_FIRE)

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            label = data.get("label")
            rpi_id = data.get("rpi_id")

            if label in ["fire","cigar", "fireball"]:
                print(f"[MQTT]  ^=^z  Fuego detectado por {rpi_id}: {label}")
                self.fire_detected = True
                self.last_rpi = rpi_id
                
            else:
                print(f"[MQTT] Mxensaje ignorado de {rpi_id}: {label}")
        except Exception as e:
            print("[MQTT] Error procesando mensaje:", e)

    # -----------------------------
    # L  gica de estados
    # -----------------------------
    def run(self):
        while True:
            if self.state == "IDLE":
                self.idle_state()
            elif self.state == "NAVIGATION":
                self.navigation_state()
            elif self.state == "SEARCH":
                self.search_state()
            elif self.state == "FIRE_FIGHTING":
                self.fire_fighting_state()
            elif self.state == "DONE":
                self.done_state()
                break
            else:
                print("Unknown state! Stopping.")
                break
            time.sleep(0.1)
            
    

    def idle_state(self):
        print("[STATE] IDLE: Running mosquito control program...")
        if self.fire_detected:
            self.state = "NAVIGATION"
        

    def navigation_state(self):
        print(f"[STATE] NAVIGATION: Ejecutando navegaci  n por {self.last_rpi}...")
        try:
            subprocess.run(["python3", NAVIGATION_SCRIPT], check=True)    #after navegation
            self.state = "FIRE_FIGHTING"   # Go to SEARCH state after navigation
            print("State changed to FIRE_FIGHTING.")
        except Exception as e:
            print(f"Error ejecutando navegaci  n: {e}")

    def search_state(self):
        print("[STATE] SEARCH: Escaneando   rea...")
        time.sleep(2)
        print("Fuego confirmado visualmente.")

    def fire_fighting_state(self):
        
        #Aqui activar bomba
        #activar Servo
        
        print("[STATE] FIRE_FIGHTING: Activando bomba de agua...")
        start_time = time.time()
        while time.time() - start_time < 10:
            print("Bomba ON")
            time.sleep(10)
        print("Fuego apagado.")
        print("Bomba OFF")
        self.state = "DONE"

    def done_state(self):
        print("[STATE] DONE: Operaci  n completada. Regresando a IDLE.")
        self.fire_detected = False
        self.last_rpi = None

# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    robot = FireFighterRobot()
    robot.run()



