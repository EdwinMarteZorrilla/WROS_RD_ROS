#!/usr/bin/env python3
import time, subprocess, json, paho.mqtt.client as mqtt

# ---------------- CONFIGURATION ----------------
BROKERS = ["192.168.0.135", "192.168.149.148"]
BROKER_PORT = 1883
TOPIC_FIRE = "alerta/fuego"
RPI_ID = "FIREVOLX_ROBOT"

# Script paths
NAVIGATION_SCRIPT = "NAVIGATION-VIDEO-rotations-orig.py"
FIRE_FIGHT_SCRIPT = "MQTT-TRANSMITTER-PUMP"
SERVO_SCRIPT = "PUMP-SERVO"
SERVO_CAMERA_SCRIPT = "SERVO-CAMERA"

# New scripts for the two added states
LOCATE_FIRE_SCRIPT = "LOCATE-FIRE"
TURN_FIRE_SCRIPT = "TURN-FIRE"


class FireFighterRobot:
    def __init__(self):
        self.state = "IDLE"
        self.fire_detected = True
        self.last_rpi = None

        # MQTT setup
        self.mqtt_clients = []
        for ip in BROKERS:
            client = mqtt.Client()
            client.on_connect = self.on_connect
            client.on_message = self.on_message
            try:
                client.connect(ip, BROKER_PORT, 60)
                client.loop_start()
                print(f"[MQTT] ✅ Connected to broker {ip}")
            except Exception as e:
                print(f"[MQTT] ⚠️ Could not connect to {ip}: {e}")
            self.mqtt_clients.append(client)

    # ---------------- MQTT ----------------
    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_FIRE)
        print(f"[MQTT] Subscribed to {TOPIC_FIRE}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            label = payload.get("label", "").lower()
            rpi_id = payload.get("rpi_id", "UNKNOWN")

            if msg.topic == TOPIC_FIRE and label in ["fire", "cigar", "fireball"]:
                print(f"[MQTT] 🔥 Fire signal detected from {rpi_id}: {label}")
                self.fire_detected = True
                self.last_rpi = rpi_id

        except Exception as e:
            print(f"[MQTT] ❌ Error processing message: {e}")

    # ---------------- STATE MACHINE ----------------
    def run(self):
        while True:
            if self.state == "IDLE":
                print("[STATE] 💤 IDLE → waiting for fire alert...")
                if self.fire_detected:
                    print("[STATE] 🚀 Switching to NAVIGATION")
                    self.state = "NAVIGATION"

            elif self.state == "NAVIGATION":
                self.navigation_state()
                self.state = "LOCATE_FIRE"

            elif self.state == "LOCATE_FIRE":
                self.locate_fire_state()
                self.state = "TURN_FIRE"

            elif self.state == "TURN_FIRE":
                self.turn_fire_state()
                self.state = "FIRE_FIGHTING"

            elif self.state == "FIRE_FIGHTING":
                self.fire_fighting_state()
                self.state = "SEARCH"

            elif self.state == "SEARCH":
                self.search_state()
                self.state = "RETURN_NAVIGATION"

            elif self.state == "RETURN_NAVIGATION":
                self.return_navigation_state()
                self.state = "DONE"

            elif self.state == "DONE":
                self.done_state()
                self.state = "IDLE"

            time.sleep(0.2)

    # ---------------- STATE DEFINITIONS ----------------

    def navigation_state(self):
        try:
            print("[STATE] 🧭 NAVIGATION: Moving toward fire zone...")
            subprocess.run(
                ["python3", NAVIGATION_SCRIPT, str(self.last_rpi)],
                check=True
            )
        except Exception as e:
            print(f"[ERROR] ❌ NAVIGATION: {e}")

    def locate_fire_state(self):
        print("[STATE] 🔍 LOCATE_FIRE: Scanning area for flame position...")
        try:
            subprocess.run(["python3", LOCATE_FIRE_SCRIPT], check=True)
            print("[ACTION] ✅ Fire located.")
        except Exception as e:
            print(f"[ERROR] ❌ LOCATE_FIRE: {e}")

    def turn_fire_state(self):
        print("[STATE] 🔄 TURN_FIRE: Orienting robot toward fire.")
        try:
            subprocess.run(["python3", TURN_FIRE_SCRIPT], check=True)
            print("[ACTION] ✅ Robot turned toward fire.")
        except Exception as e:
            print(f"[ERROR] ❌ TURN_FIRE: {e}")

    def fire_fighting_state(self):
        print("[STATE] 💦 FIRE_FIGHTING: Activating pump and servo.")
        try:
            pump = subprocess.Popen(["python3", FIRE_FIGHT_SCRIPT])
            servo = subprocess.Popen(["python3", SERVO_SCRIPT])
            time.sleep(6)
            pump.terminate()
            servo.terminate()
            print("[ACTION] ✅ Fire extinguishing completed.")
        except Exception as e:
            print(f"[ERROR] ❌ FIRE_FIGHTING: {e}")

    def search_state(self):
        try:
            print("[STATE] 🎥 SEARCH: Moving servo camera.")
            servo_process = subprocess.Popen(["python3", SERVO_CAMERA_SCRIPT])
            time.sleep(10)
            servo_process.terminate()
            print("[SEARCH] ✅ Servo camera movement finished.")
        except Exception as e:
            print(f"[ERROR] ❌ SEARCH: {e}")

    def return_navigation_state(self):
        print("[STATE] 🏁 RETURN_NAVIGATION: Returning to origin.")
        try:
            subprocess.run(
                ["python3", NAVIGATION_SCRIPT, str(self.last_rpi)],
                check=True
            )
        except Exception as e:
            print(f"[ERROR] ❌ RETURN_NAVIGATION: {e}")

    def done_state(self):
        print("[STATE] 🔄 DONE: Resetting to IDLE.")
        self.fire_detected = False
        self.last_rpi = None


if __name__ == "__main__":
    FireFighterRobot().run()
