# Arduino: Sentry Turret

Sentry Turret es un sistema embebido hecho Arduino durante la cursada de Sistemas Operativos Avanzados, 2020 2C. 

El sistema monitorea el área circundante en busca de movimientos producidos por seres vivos. En caso de detectarlos, 
escaneará el área tratando de dar con lo que causó dicho movimiento. 

Estas son algunas notables características relaciondas con la programación del sistema:

* Implementación de máquina de estados con eventos.
* Uso personalizado de los Timers de Arduino.
* Generación de interrupciones internas a través de los Timers.
* Generación de interrupciones externas.
* Temporizador híbrido (software basado en interrupciones por hardware).

## Sensores y actuadores

El sistema utiliza los siguientes sensores y actuadores:

* Sensor PIR
* Sensor de luz (fotoresistor)
* Sensor de distancia (ultrasonido - HC-SR04)
* Actuador: Pulsador
* Actuador: Servomotor
* Actuador: LEDs

## ¿Cómo funciona?

La torreta utiliza el fotoresistor para detectar el nivel de luminosidad del ambiente. Si el nivel de luminosidad es lo suficientemente bajo, se activará y permanecerá en stand-by hasta
detectar movimiento.

El sensor PIR se encarga de detectar movimiento producido por seres vivos. Una vez que se detecta movimiento, el servomotor comenzará a girar la torreta, haciendo barridos de 180º.

Mientras la torreta se mueve, el sensor de distancia realiza mediciones constantemente, hasta dar con un objeto que se encuentre dentro de su rango de detección. La torreta quedará fija apuntando al objeto que se encuentra dentro del rango.




