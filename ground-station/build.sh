#!/bin/sh
# Build the ground station (needs JDK 11+). Output: build/ground-station.jar
set -e
cd "$(dirname "$0")"
rm -rf build/classes && mkdir -p build/classes
javac -d build/classes $(find src -name '*.java')
printf 'Main-Class: ro.chirea.drone.GroundStation\n' > build/manifest.txt
jar cfm build/ground-station.jar build/manifest.txt -C build/classes .
echo "Run:  java -jar build/ground-station.jar"
