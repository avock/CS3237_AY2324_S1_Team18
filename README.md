# Wall-ESP Code

This cpp file is to be uploaded to the ESP32 that is connected to the respective
sensors and servo through the indicated pins.

The ESP32 will read inputs from the surroundings and publish the readings as an
MQTT message on the topic home/input.

The ESP32 also subscribes to the MQTT topic home/input, where actuation commands
from the MQTT broker will be processed in order to move the servo to turn on/off
the light according to the user's wishes.

# Bed-ESP

Contains code used for :

-   Handling ESP32cam functions (taking pictures, sending to localserver)
-   Triggering ESP32cam awake via an ultrasonic sensor
-   Collecting pressure data and send data to the remote server

# Gesture Recognition CNN

Contains the jupyternotebook used to train the CNN for gesture recognition and
the dataset used to train said model.
