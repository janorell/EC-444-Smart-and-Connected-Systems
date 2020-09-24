# Security in Connected Systems

Author: Jennifer Norell, 2019-11-19

## Summary

Below are the responses to the questions asked for this skill

Identify weaknesses in your node.js to ESP32 system for quest 3 (wearable)?
* Host IP address  is hardcoded in, if someone accessed the code, could easily be changed to connect to a different host. 
* Once the server is up and running, any device that accesses the server can click the buttons (show/hide/etc) and it will affect all other people using the server at the same time (will show for all computers, hide for all computers)
* Using console log to output errors, someone can use this to see what can / will break our server
* data is being outputted to a page on the server (/data) so the format and all the old data is being taken in. this isn't something we would want anyone to see, but they can if they find the page (easily found with a site mapper)


What ways can a bad guy attack your specific system? List at least three
* using #define is actually considered bad practice (according to my cybersecurity professor). Define is literally just inserting what is typed after it into wherever the define is called, so if it is typed incorrectly (or something malicious is purposely put there, there is no warning for it)
* checking the console log to see what is being outputted/ happening behind the scenes from certain events (can use to break the system)
* when clicking the find my device button too often there is too much traffic for the server to handle and it crashes. (DoS attack)

Describe ways that you can overcome these attacks. What ways can you mitigate these issues?
* instead of using define you can simply change it to a static constant variable so the compiler can tell if there are characters that should not be there.
* instead output to a specific debugging file or simply comment/delete the console.log info once done debugging
* could put a limit on how many times a button could be click in a minute in order to combat this (if pressed more than 5 times, ignore other presses for x amount of time)

“IoT” provisioning is a specific challenge for connected devices. Describe how the ESP32 code base provides an IoT provisioning solution. This is the process of associating an ESP32 with a particular user and authentication in a local network.
* Espressif provides a private key and a device certificate for each device. This private key remains encrypted during the flash aswell to ensure security. We can see this particularly on windows computer as each device is assigned a specific port (for example on my group mates computer my port is COM4 and her's is COM3). Additionally, each ESP32 has its own IP address so when connecting to a network, you have to know what your ESP32's IP and use it in the code in order to make any sort of connection. 

## Sketches and Photos
N/A


## Modules, Tools, Source Used in Solution
Quest 3 Code


## Supporting Artifacts
https://www.rapid7.com/fundamentals/man-in-the-middle-attacks/
http://www.skullbox.net/tcpudp.php
https://us.norton.com/internetsecurity-emerging-threats-dos-attacks-explained.html
https://www.espressif.com/en/news/Espressif_preprovisioning
https://medium.com/the-esp-journal/understanding-esp32s-security-features-14483e465724
EC521 (Cybersecurity)

-----

## Reminders
- Repo is private
