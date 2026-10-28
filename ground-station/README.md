# Ground station (Java 11+, no dependencies)

```bash
./build.sh          # Windows: build.bat
java -jar build/ground-station.jar
```
Copy `config.example.properties` to `config.properties` next to where you run the jar to pre-fill the
connection tab. The password is typed in and never stored. Tick **Offline demo** to explore the interface
without Firebase.

Keyboard (after ticking *Keyboard control*): W/S pitch · A/D yaw · ←/→ roll · ↑/↓ throttle · Space centre · Esc motor stop.
