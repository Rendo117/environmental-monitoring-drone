# Raspberry Pi bridge

```bash
sudo raspi-config            # Interface → Serial: login shell NO, hardware YES
sudo apt install pigpio python3-pigpio python3-serial python3-requests
sudo systemctl enable --now pigpiod
cp config.example.ini config.ini        # Firebase URL, API key, drone account
python3 bridge.py --simulate            # test without hardware
python3 bridge.py                       # real run
sudo cp drone-bridge.service /etc/systemd/system/ && sudo systemctl enable --now drone-bridge
python3 test_sensors.py                 # unit tests
```

* `bridge.py`: main loop (commands → pulses, telemetry → Firebase, watchdog)
* `sensors.py`: MQ-2 ppm, Geiger dose rate with uncertainty, frame parser, pulse mapping
* `firebase_client.py`: minimal authenticated REST client (no Firebase SDK)
