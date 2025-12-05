# Activity 1 Observation
<img width="720" height="173" alt="image" src="https://github.com/user-attachments/assets/678dab73-9f9d-44f5-abbe-258ebbe1764e" />

On one Pico, a transmit task is running and transmits "hello." Another pico, running a receiving task, sends the received message to
a queue from the ISR and prints it to the serial monitor. The "???" chars in the image are an artifact of the msg.dlc being 8 bytes, while
"hello" is only 5 bytes.

The transmit and receive code exists in one c file. The transmit or receive task is commented out depending on which pico is being flashed,
and then compiled. There are two build directories: txbuild and rxbuild. The builds are flashed to the respective picos.


# Activity 2 Observation
<img width="1004" height="754" alt="image" src="https://github.com/user-attachments/assets/0a6acbd4-9147-40b5-bdd3-547f03e73105" />

The pico that broadcasts slowly sends a message with a low priority of 2 every 2 seconds. The babbling pico sends messages nearly continuously using a busy loop that doesn't yeild with
message IDs of 1. Without the busy loop, the pico broadcasting the low priority messages was starved. With the added busy loop, the low priority messages are able to squeeze in fairly consistently
every 2 seconds. The transmission of the message takes about 200 micro seconds, so there is little less than 10,000 babbling transmissions for every low priority transmission. 

<img width="938" height="304" alt="image" src="https://github.com/user-attachments/assets/66873783-fefc-415a-a2f2-0eb94c06da42" />

