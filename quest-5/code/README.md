# Code Readme

The code folder consists of four folders (running the C code for the ESPs) and two files (running the server and web client) on the outer level of the directory:
- ./ fob_vs.c
- ./ fob_vk.c
- ./ fob_jn.c
- ./ hub.c
- demo.js
- demo.html

The c files primarily use UART (IR Tx/Rx), 38KHz Signal from RMT, GPIO (LED control) for IR communication to send id and codes. The c files also use UDP sockets to communicate with the node server over port 8082. The hub sends information via 'payload' that it has received from each fob. Each fob receives a unique, respective message 'action' as to whether the fob has been authorized or not (LED on or not).

'demo.js' implements the two Level databases that are used for checking authorization and authentication, communicates with the fobs with the UDP server, receives messages from the hub with the UDP server, and hosts and sends database entries to the web client via socket.io to 'demo.html'.

How to run locally:

1. Connect to Group_7 WiFi
2. Power RPi and SSH or use Raspian desktop
3. idf.py menuconfig on all 4 C folders. Configure the WiFi, port as 8082, and RPi IP address.
4. Build ESP .c code using espressif build and make files.
5. Install all necessary node modules: npm i && npm i express socket.io level --save
6. Start node server: node demo.js
7. Flash the code to 4 the ESP32 (3 fobs and 1 hub) and run.
8. Open (ip):3000 for the demo.html view
9. Bring the fob IR LEDs close to the hub IR receiver. Click the button to send the signal and begin

The pins used for communication on the fob are:
- RMT Pulse - GPIO 26
- UART Transmitter - GPIO 25
- Button - GPIO 4
- Green LED - GPIO 32
- BLUE LED - GPIO 14

The pins used for communication on the hub are:
- UART Receiver - GPIO 34
- Red LED - GPIO 15
- Green LED - GPIO 32
- BLUE LED - GPIO 14
