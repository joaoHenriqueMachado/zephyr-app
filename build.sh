#!/bin/bash

cd C:/Zephyr/zephyrproject/
source .venv/bin/activate
west build -p always C:/Zephyr/zephyrproject/zephyr/zephyr_app -d C:/Zephyr/zephyrproject/zephyr/zephyr_app/build

