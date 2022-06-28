#! /bin/sh

mavproxy.py --master=/dev/ttyAMA0 --out=udpin:127.0.0.1:14551 --baudrate 115200

sleep 10

python3 ARRC_Power_sampling.py
