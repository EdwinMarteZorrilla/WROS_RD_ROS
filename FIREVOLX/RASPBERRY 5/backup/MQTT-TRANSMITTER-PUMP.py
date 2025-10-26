import time
import paho.mqtt.client as mqtt

# -----------------------------
# CONXFIGURACI ^sN MQTT
# -----------------------------
BROKER_IP = "127.0.0.1"  # Cambia a la IP de tu Pi si es necesario
BROKER_PORT = 1883
TOPIC = "PUMP-TESTX"

# Duraci  n opcional para mandar "off" despu  s de cierto tiempo (en segundos)
OFF_DELAY = 10  # Cambia este valor seg  n lo que quieras esperar

# -----------------------------
# CONEXI ^sN MQTT
# -----------------------------
client = mqtt.Client()
client.connect(BROKER_IP, BROKER_PORT, 60)
client.loop_start()

try:
    # Enviar "on" una sola vez
    client.publish(TOPIC, "on")
    print("Published: on")

    # Esperar el tiempo configurado antes de mandar "off"
    time.sleep(OFF_DELAY)

    # Enviar "off" una sola vez
    client.publish(TOPIC, "off")
    print("Published: off")

except KeyboardInterrupt:
    print("Stopping program...")

finally:
    client.loop_stop()
    client.disconnect()



