# Motor-Monitoring-System

A motor management app that uses an Adafruits MQTT cloud dashboard to display the tempature taken off the STM, the status of the motor, if it is balanced on its axis. The user can use serial or the buttons on the MQTT dashboard to control the motor. 
The motor will automatically shut off if the board is off balance that is set on startup or changed. or if the tempature is too high. 
