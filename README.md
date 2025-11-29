# Activity 1 Observation
<img width="720" height="173" alt="image" src="https://github.com/user-attachments/assets/678dab73-9f9d-44f5-abbe-258ebbe1764e" />

On one Pico, a transmit task is running and transmits "hello." Another pico, running a receiving task, sends the received message to
a queue from the ISR and prints it to the serial monitor. The "???" chars in the image are an artifact of the msg.dlc being 8 bytes, while
"hello" is only 5 bytes.

The transmit and receive code exists in one c file. The transmit or receive task is commented out depending on which pico is being flashed,
and then compiled. There are two build directories: txbuild and rxbuild. The builds are flashed to the respective picos.
