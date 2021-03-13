#!/bin/bash
service dbus start
sleep 5
hciconfig hci0 reset
hciconfig hci0
/nrf52_sht30_collector