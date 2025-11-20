#!/usr/bin/env python3
import time
import paho.mqtt.client as mqtt
from gpiozero import DigitalOutputDevice

# -----------------------------
# MQTT CONFIG
# -----------------------------
BROKER_IP = "127.0.0.1"  # Cambia si tu broker estÃ¡ en otra IP
BROKER_PORT = 1883
TOPIC = "PUMP-TESTX"

# -----------------------------
# GPIO CONFIG
# -----------------------------
PUMP_PIN = 23  # Pin de control de la bomba
PUMP_DURATION = 3.0  # DuraciÃ³n (en segundos) que la bomba estarÃ¡ encendida

# -----------------------------
# InicializaciÃ³n de hardware
# -----------------------------
pump = DigitalOutputDevice(PUMP_PIN)

# -----------------------------
# FUNCIONES MQTT
# -----------------------------
def on_connect(client, userdata, flags, rc):
    print(f"âœ… Connected to MQTT Broker with result code {rc}")
    client.subscribe(TOPIC)

def run_pump_cycle():
    """Activa la bomba durante el tiempo definido en PUMP_DURATION."""
    print(f"ðŸš° Activating pump for {PUMP_DURATION} seconds...")
    pump.on()
    time.sleep(PUMP_DURATION)
    pump.off()
    print("âœ… Pump cycle completed.")

def on_message(client, userdata, msg):
    command = msg.payload.decode().strip().lower()
    print(f"ðŸ“© Received MQTT message: {command}")

    if command == "on":
        run_pump_cycle()
    elif command == "off":
        print("ðŸ›‘ Pump stopped manually.")
        pump.off()
    else:
        print(f"âš ï¸ Unknown command: {command}")

# -----------------------------
# MAIN MQTT CLIENT
# -----------------------------
client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_IP, BROKER_PORT, 60)
client.loop_forever()

