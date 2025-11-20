#!/usr/bin/env python3
# edgeimpulse_mqtt.py
# Raspberry Pi 4: publica detección de fuego filtrada + heartbeat cada 1s
# Después de 5 detecciones consecutivas, ignora nuevas detecciones por 60 segundos

import subprocess
import threading
import json
import time
import paho.mqtt.client as mqtt
import sys
import os

# ---------- CONFIGURACIÓN ----------
BROKER_IP = "192.168.149.171"
BROKER_PORT = 1883
MQTT_TOPIC = "alerta/fuego"
RPI_ID = "RPI_1"
runner_path = '/usr/bin/edge-impulse-linux-runner'
model_path = 'model.eim'
DESIRED_LABEL = {"fire","cigar"}
THRESHOLD = 0.90
HEARTBEAT_INTERVAL = 1     # manda OK cada 1 segundo
REQUIRED_CONSECUTIVE = 5
IGNORE_DURATION = 60        # segundos que se ignoran nuevas detecciones
# -----------------------------------

# Verificar existencia del modelo
if not os.path.exists(model_path):
    print(f"[emisor] ERROR: El archivo de modelo '{model_path}' no existe.")
    sys.exit(1)

# Configurar MQTT
client = mqtt.Client()
try:
    client.connect(BROKER_IP, BROKER_PORT, 60)
except Exception as e:
    print("[emisor] ERROR: No pude conectar al broker MQTT:", e)
    sys.exit(1)

client.loop_start()

# Variables de estado
consecutive_fire = 0
fire_active = False
ignore_further_detections = False  # ignora detecciones por un tiempo
last_box_published = None ## <-- NUEVO: Para evitar publicar la misma caja varias veces

def reset_ignore_flag():
    global ignore_further_detections, fire_active, consecutive_fire, last_box_published
    print("[emisor] 🔵 Ignorar detecciones finalizado. Listo para detectar de nuevo.")
    ignore_further_detections = False
    fire_active = False
    consecutive_fire = 0
    last_box_published = None # Limpiamos la última caja

def publish_fire_with_coords(box):
    global last_box_published
    
    ## <-- MODIFICADO: Comprobación para no duplicar
    if box == last_box_published:
        print("[emisor] Ignorando publicación duplicada de la misma detección.")
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
        print(f"[emisor] 🚨 FIRE enviado: {payload}")
        last_box_published = box # Guardamos la caja que acabamos de publicar
    except Exception as e:
        print("[emisor] Error publicando MQTT:", e)

def publish_heartbeat():
    while True:
        # Solo manda heartbeat si no hay una alarma activa
        if not fire_active and not ignore_further_detections: ## <-- MODIFICADO
            payload = {
                "rpi_id": RPI_ID,
                "label": "none",
                "status": "OK"
            }
            try:
                client.publish(MQTT_TOPIC, json.dumps(payload))
                # Descomenta la siguiente línea si quieres ver el heartbeat en la consola
                # print(f"[emisor] Mensaje HEARTBEAT enviado: {payload}")
            except Exception as e:
                print("[emisor] Error publicando heartbeat:", e)
        time.sleep(HEARTBEAT_INTERVAL)

# Lanza el runner de Edge Impulse con modelo específico
try:
    proc = subprocess.Popen(
        [runner_path, '--model-file', model_path, '--clean', '--camera', '/dev/video0'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    print("[emisor] Runner lanzado con modelo model.eim.")
except Exception as e:
    print("[emisor] No se pudo lanzar edge-impulse-linux-runner:", e)
    sys.exit(1)

def read_loop():
    global consecutive_fire, fire_active, ignore_further_detections
    for line in proc.stdout:
        line = line.strip()
        
        # Si estamos en modo "ignorar", simplemente saltamos todo
        if ignore_further_detections:
            continue
            
        if not line:
            continue
        
        # Imprime la salida del runner solo si no es un heartbeat
        if 'boundingBoxes' in line or 'anomaly' in line:
            print("[runner]", line)

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
                            best_fire_box = box # Guardamos la caja con mayor confianza
                

                if fire_in_this_frame:
                    consecutive_fire += 1
                    print(f"[emisor] Detectado {best_fire_box.get('label')} con confianza {max_confidence:.2f}")
                    print(f"[emisor] Fuegos consecutivos: {consecutive_fire}")

                    if consecutive_fire >= REQUIRED_CONSECUTIVE and not fire_active:
                        fire_active = True
                        ignore_further_detections = True # Inicia el modo "ignorar"
                        
                        # Publicamos la mejor detección de este frame
                        publish_fire_with_coords(best_fire_box)
                        
                        print(f"[emisor] 🚀 Fuego confirmado. Ignorando detecciones por {IGNORE_DURATION} segundos.")

                        # Lanzar temporizador para reiniciar después de IGNORE_DURATION segundos
                        t = threading.Timer(IGNORE_DURATION, reset_ignore_flag)
                        t.start()
                
                ## -----------------------------------------------------------------
                ## --- ESTA ES LA MODIFICACIÓN CLAVE ---
                else:
                    # Si no hubo fuego en este frame, reseteamos el contador
                    if consecutive_fire > 0:
                        print(f"[emisor] Racha de fuego rota. Reseteando contador de {consecutive_fire} a 0.")
                    consecutive_fire = 0
                ## -----------------------------------------------------------------

            except Exception as e:
                print(f"[emisor] Error analizando boundingBoxes: {e} | Línea: {line}")

# Iniciar hilos
reader = threading.Thread(target=read_loop, daemon=True)
reader.start()

heartbeat_thread = threading.Thread(target=publish_heartbeat, daemon=True)
heartbeat_thread.start()

# Monitor básico
try:
    while True:
        time.sleep(1)
        if proc.poll() is not None:
            print("[emisor] El runner se cerró.")
            break
except KeyboardInterrupt:
    print("[emisor] Interrumpido por usuario.")
finally:
    print("[emisor] Cerrando...")
    try:
        proc.kill()
    except:
        pass
    client.loop_stop()
    client.disconnect()
    print("[emisor] Desconectado y finalizado.")