#! /bin/sh

python3 mavproxy.py --master=/dev/serial0 --out=udpin:127.0.0.1:14551 --baudrate 921600

sleep 10

python3 ARRC_Power_sampling.py