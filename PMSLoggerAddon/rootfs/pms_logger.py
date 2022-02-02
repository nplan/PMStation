import datetime
import logging
import json
import argparse
from secrets import token_hex
import sys

from peewee import Model, IntegerField, FloatField, DateTimeField, SqliteDatabase
from playhouse.sqlite_ext import JSONField
import paho.mqtt.client as mqtt

HOST = "localhost"
PORT = 1883
USER = None
PASSWORD = None
TOPIC = "home/PMStation/raw"
DB_PATH = "pms.db"
LOG_PATH = "pms.log"

def main():
    parser = argparse.ArgumentParser(description="Logs PMS MQTT data to sqlite database.")
    parser.add_argument("--host", help="MQTT broker host", default=HOST)
    parser.add_argument("--port", help="MQTT broker port", type=int, default=PORT)
    parser.add_argument("--user", help="MQTT broker user", default=USER)
    parser.add_argument("--password", help="MQTT broker password", default=PASSWORD)
    parser.add_argument("--topic", help="MQTT topic with raw json data", default=TOPIC)
    parser.add_argument("--db", help="Database path", default=DB_PATH)
    parser.add_argument("--log", help="Log path", default=LOG_PATH)
    parser.add_argument("--debug", help="verbose debug mode", action="store_true")
    parser.add_argument("--test", help="print params and quit", action="store_true")
    args = parser.parse_args()

    if args.test:
        print("Input Parameters:")
        print(f"Host: {args.host}")
        print(f"Port: {args.port}")
        print(f"User: {args.user}")
        print(f"Password: {args.password}")
        print(f"Topic: {args.topic}")
        print(f"DB: {args.db}")
        print(f"Log: {args.log}")
        print("Exiting.")
        sys.exit(0)

    db = SqliteDatabase(args.db)


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


    logger = logging.getLogger(__name__)

    console = logging.StreamHandler()
    file = logging.FileHandler(args.log)
    formatter = logging.Formatter(fmt='%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    file.setFormatter(formatter)
    logger.addHandler(console)
    logger.addHandler(file)
    logger.setLevel(logging.DEBUG if args.debug else logging.INFO)

    db.connect()
    db.create_tables([PMSReading], safe=True)

    def on_connect(client, userdata, flags, rc):
        logger.info("MQTT client connected")
        client.subscribe(args.topic, qos=2)

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

    client = mqtt.Client(f"PMSLogger_{token_hex(3)}")
    if args.user and args.password:
        client.username_pw_set(args.user, args.password)
    client.on_message = on_msg
    client.on_connect = on_connect

    client.enable_logger(logger)
    client.connect_async(args.host, args.port)
    client.loop_forever(retry_first_connection=True)


if __name__ == '__main__':
    main()
