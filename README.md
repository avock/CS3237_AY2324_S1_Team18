# Wall-ESP

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

-   gestureRecognition_modelTraining.ipynb file contains the training code for
    CNN model.
-   gestureRecognition_modelPrediction.ipynb file contains the code to load and
    deploy the trained CNN model
-   trained CNN model is saved as handclassifier6.pt
-   10113-bg4.jpg and 10113-hand5.jpg are sample background and gesture images
    used for prediction.
-   gesture_dataset.zip is the hand gesture training dataset found online.
