#!/usr/bin/env python3
# edgeimpulse_mqtt.py
# Raspberry Pi 4: Publica detecci車n de fuego filtrada + heartbeat cada 5s
# Despu谷s de 5 detecciones consecutivas, ignora nuevas detecciones por 60 segundos

import subprocess
import threading
import json
import time
import paho.mqtt.client as mqtt
import sys
import os

# ---------- CONFIGURACI車N ----------
BROKER_IP = "192.168.149.171"
BROKER_PORT = 1883
MQTT_TOPIC = "alerta/fuego"
RPI_ID = "RPI_1"   # <-- CHANGE TO "RPI_2" ON THE SECOND PI
runner_path = '/usr/bin/edge-impulse-linux-runner'
model_path = 'model.eim'
DESIRED_LABEL = "fire"
THRESHOLD = 0.90
HEARTBEAT_INTERVAL = 5     # seconds between heartbeats
REQUIRED_CONSECUTIVE = 5
IGNORE_DURATION = 60       # seconds ignoring new detections
# -----------------------------------

# Verify model exists
if not os.path.exists(model_path):
    print(f"[{RPI_ID}] ? ERROR: El archivo de modelo '{model_path}' no existe.")
    sys.exit(1)

# ---------------- MQTT CONFIGURATION ----------------
client = mqtt.Client(client_id=f"edge_client_{RPI_ID}")  # ? UNIQUE CLIENT ID

def on_disconnect(client, userdata, rc):
    print(f"[{RPI_ID}] ?? Disconnected from broker (code {rc}). Trying to reconnect...")
    while True:
        try:
            client.reconnect()
            print(f"[{RPI_ID}] ? Reconnected to broker!")
            break
        except Exception as e:
            print(f"[{RPI_ID}] Reconnect failed: {e}")
            time.sleep(5)

client.on_disconnect = on_disconnect

try:
    client.connect(BROKER_IP, BROKER_PORT, 60)
    print(f"[{RPI_ID}] ? Connected to MQTT broker {BROKER_IP}:{BROKER_PORT}")
except Exception as e:
    print(f"[{RPI_ID}] ? ERROR: Could not connect to broker:", e)
    sys.exit(1)

client.loop_start()
# ----------------------------------------------------

# State variables
consecutive_fire = 0
fire_active = False
ignore_further_detections = False
last_box_published = None

def reset_ignore_flag():
    global ignore_further_detections, fire_active, consecutive_fire, last_box_published
    print(f"[{RPI_ID}] ?? Ignorar detecciones finalizado. Listo para detectar de nuevo.")
    ignore_further_detections = False
    fire_active = False
    consecutive_fire = 0
    last_box_published = None

def publish_fire_with_coords(box):
    global last_box_published
    
    if box == last_box_published:
        print(f"[{RPI_ID}] ?? Ignorando publicaci車n duplicada.")
        return

    x = box.get('x', 0)
    y = box.get('y', 0)
    width = box.get('width', 0)
    height = box.get('height', 0)
    center_x = x + width / 2
    center_y = y + height / 2

    payload = {
        "rpi_id": RPI_ID,
        "label": box.get('label', 'fire'),
        "confidence": box.get('value', 0.0),
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "center_x": center_x,
        "center_y": center_y
    }

    try:
        client.publish(MQTT_TOPIC, json.dumps(payload))
        print(f"[{RPI_ID}] ?? FIRE enviado: {payload}")
        last_box_published = box
    except Exception as e:
        print(f"[{RPI_ID}] Error publicando MQTT:", e)

def publish_heartbeat():
    while True:
        if not fire_active and not ignore_further_detections:
            payload = {
                "rpi_id": RPI_ID,
                "label": "none",
                "status": "OK"
            }
            try:
                client.publish(MQTT_TOPIC, json.dumps(payload))
            except Exception as e:
                print(f"[{RPI_ID}] Error publicando heartbeat:", e)

        client.loop(0.1)  # ? keeps MQTT responsive under CPU load
        time.sleep(HEARTBEAT_INTERVAL)

# Launch Edge Impulse runner
try:
    proc = subprocess.Popen(
        [runner_path, '--model-file', model_path, '--clean', '--camera', '/dev/video0', '--fps', '5', '--crop', '320x240'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    print(f"[{RPI_ID}] ?? Runner lanzado con modelo '{model_path}'.")
except Exception as e:
    print(f"[{RPI_ID}] ? No se pudo lanzar edge-impulse-linux-runner:", e)
    sys.exit(1)

def read_loop():
    global consecutive_fire, fire_active, ignore_further_detections
    for line in proc.stdout:
        line = line.strip()
        if ignore_further_detections or not line:
            continue
        
        if 'boundingBoxes' in line or 'anomaly' in line:
            print(f"[{RPI_ID}][runner]", line)

        if 'boundingBoxes' in line:
            try:
                json_start = line.find('[')
                if json_start == -1:
                    continue
                    
                boxes = json.loads(line[json_start:])
                fire_in_this_frame = False
                best_fire_box = None
                max_confidence = 0

                for box in boxes:
                    label = box.get('label', '')
                    value = box.get('value', 0.0)
                    if label == DESIRED_LABEL and value >= THRESHOLD:
                        fire_in_this_frame = True
                        if value > max_confidence:
                            max_confidence = value
                            best_fire_box = box
                
                if fire_in_this_frame:
                    consecutive_fire += 1
                    print(f"[{RPI_ID}] ?? Detectado {best_fire_box.get('label')} conf {max_confidence:.2f}")
                    print(f"[{RPI_ID}] Fuegos consecutivos: {consecutive_fire}")

                    if consecutive_fire >= REQUIRED_CONSECUTIVE and not fire_active:
                        fire_active = True
                        ignore_further_detections = True
                        publish_fire_with_coords(best_fire_box)
                        print(f"[{RPI_ID}] ?? Fuego confirmado. Ignorando por {IGNORE_DURATION}s.")
                        threading.Timer(IGNORE_DURATION, reset_ignore_flag).start()
                else:
                    if consecutive_fire > 0:
                        print(f"[{RPI_ID}] ?? Racha de fuego rota. Reset {consecutive_fire} ↙ 0.")
                    consecutive_fire = 0

            except Exception as e:
                print(f"[{RPI_ID}] Error analizando boundingBoxes: {e} | L赤nea: {line}")

# Start threads
reader = threading.Thread(target=read_loop, daemon=True)
reader.start()

heartbeat_thread = threading.Thread(target=publish_heartbeat, daemon=True)
heartbeat_thread.start()

# Monitor main loop
try:
    while True:
        time.sleep(1)
        if proc.poll() is not None:
            print(f"[{RPI_ID}] ?? El runner se cerr車.")
            break
except KeyboardInterrupt:
    print(f"[{RPI_ID}] ?? Interrumpido por usuario.")
finally:
    print(f"[{RPI_ID}] Cerrando...")
    try:
        proc.kill()
    except:
        pass
    client.loop_stop()
    client.disconnect()
    print(f"[{RPI_ID}] Desconectado y finalizado.")
