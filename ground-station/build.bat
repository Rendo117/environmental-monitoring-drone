@echo off
rem Build the ground station on Windows (needs JDK 11+).
cd /d "%~dp0"
if exist build\classes rmdir /s /q build\classes
mkdir build\classes
dir /s /b src\*.java > build\sources.txt
javac -d build\classes @build\sources.txt || exit /b 1
echo Main-Class: ro.chirea.drone.GroundStation> build\manifest.txt
jar cfm build\ground-station.jar build\manifest.txt -C build\classes .
echo Run:  java -jar build\ground-station.jar
