import paho.mqtt.client as mqtt
from time import sleep, time
from peewee import Model, IntegerField, FloatField, DateTimeField, SqliteDatabase
from playhouse.sqlite_ext import JSONField
import os
import datetime
import logging
import json

ip = "192.168.1.112"
port = 1883
topic = "home/PMStation/raw"

directory = os.path.join(os.path.expanduser("~"), "PMS_data")
os.makedirs(directory, exist_ok=True)
db_path = os.path.join(directory, "pms.db")
log_path = os.path.join(directory, "pms.log")

# db_path = "pms_log.db"
db = SqliteDatabase(db_path)


class PMSReading(Model):

    pm1 = IntegerField()
    pm2_5 = IntegerField()
    pm10 = IntegerField()
    temperature = FloatField()
    humidity = FloatField()
    pressure = FloatField()
    raw = JSONField()
    time = DateTimeField(default=datetime.datetime.utcnow)

    class Meta:
        database = db


if __name__ == '__main__':
    logger = logging.getLogger(__name__)

    console = logging.StreamHandler()
    file = logging.FileHandler(log_path)
    formatter = logging.Formatter(fmt='%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    file.setFormatter(formatter)
    logger.addHandler(console)
    logger.addHandler(file)
    logger.setLevel(logging.INFO)

    db.connect()
    db.create_tables([PMSReading], safe=True)

    def on_connect(client, userdata, flags, rc):
        logger.info("MQTT client connected")
        client.subscribe(topic, qos=2)

    def on_msg(client, userdata, message):
        logger.debug("Received message on topic '{}'".format(message.topic))
        try:
            data = json.loads(message.payload)
        except Exception as e:
            logger.error("JSON decode error: {}".format(e))
            return

        logger.debug("Received data: {}".format(data))

        try:
            reading = PMSReading(
                                pm1=int(data["PM1.0_avg"]),
                                pm2_5=int(data["PM2.5_avg"]),
                                pm10=int(data["PM10.0_avg"]),
                                temperature=float(data["temperature_avg"]),
                                humidity=float(data["humidity_avg"]),
                                pressure=float(data["pressure"]),
                                raw=data)
        except Exception as e:
            logger.error("JSON key access error: {}".format(e))
            return
        reading.save()

    client = mqtt.Client("PMStationLogger1")
    client.on_message = on_msg
    client.on_connect = on_connect

    client.enable_logger(logger)
    client.connect_async(ip, port)
    client.loop_forever(retry_first_connection=True)
