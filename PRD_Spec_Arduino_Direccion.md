# Product Requirements Document & Spec - Arduino Dirección

**Subsistema:** Módulo de Dirección (Sensórica)

**Hardware:** Arduino + Ethernet Shield (PoE)

**Lenguaje:** C++ / Arduino Framework

## 1. Visión y Alcance

Este nodo microcontrolador actuará exclusivamente como el sistema nervioso (lectura) del módulo de dirección de la tuneladora. Su única responsabilidad es adquirir datos físicos del entorno a través de sensores (IMUs, encoders y fines de carrera) y transmitirlos ininterrumpidamente al Servidor Central (Controlador Supervisor) mediante la red Ethernet. 

Por regla arquitectónica inquebrantable de separación de roles, este dispositivo tiene **estrictamente prohibido** ejecutar lógica de control, evaluar umbrales de seguridad, o enviar comandos eléctricos a los actuadores físicos (motores de las cuñas).

## 2. Mapa de Hardware y Sensórica (IO List)

El Arduino se alimentará del bus DC de bajo voltaje aislado (PoE) para garantizar que la lectura de telemetría sobreviva a un corte de energía de los motores de 480V.

Debe censar los siguientes componentes:

*   **IMU (Unidad de Medición Inercial):** Comunicación I2C. Provee la base para calcular Roll, Pitch y Heading (Orientación del cabezal).

*   **Encoders Absolutos (x2):** Miden los grados de rotación exactos de las cuñas direccionales (Motor A y Motor B).

*   **Fines de Carrera (Limit Switches x2):** Entradas digitales físicas que indican si los anillos han alcanzado su tope mecánico máximo de ±10°.

## 3. Capacidades del Sistema (Capabilities)

- **CAP-1 - Adquisición de Alta Frecuencia (Oversampling)**

  - intent: El microcontrolador lee todos los sensores físicos conectados a la mayor velocidad que permita el hardware, garantizando que el servidor Python reciba datos frescos con sobremuestreo.

  - success: El bucle `loop()` de C++ se ejecuta sin funciones bloqueantes `delay()`) y actualiza las variables en memoria a una tasa mínima sostenida de 50 Hz.

- **CAP-2 - Transmisión de Telemetría Binaria (Push)**

  - intent: El Arduino empuja su estado actual hacia la IP del servidor central mediante un socket de red ligero, evitando el procesamiento de texto o formatos pesados.

  - success: Los datos se empaquetan en un `struct` estricto de C++ y se transmiten vía **UDP** a una tasa configurable (mínimo 10 Hz). No se utiliza JSON en la capa del microcontrolador.

- **CAP-3 - Nodo Sensorial Puro (Dumb Node)**

  - intent: El microcontrolador ignora por completo la toma de decisiones lógicas o de seguridad (Interlocks).

  - success: Si un fin de carrera se activa, el Arduino simplemente envía un `1` (true) en el bit correspondiente de su *struct*. No apaga nada ni altera su propio flujo; el servidor Python es el encargado exclusivo de leer ese `1` y disparar el E-STOP hacia el PLC.

- **CAP-4 - Resiliencia y Auto-Recuperación**

  - intent: El Arduino debe sobrevivir a caídas de red físicas y recuperar la transmisión sin intervención humana.

  - success: Si se desconecta y reconecta el cable Ethernet físico, la placa reinicializa su socket y reanuda el envío de paquetes UDP hacia el servidor en menos de 2 segundos sin requerir un reinicio eléctrico (reboot).

## 4. Restricciones Técnicas (Constraints)

- **Cero Lógica Web:** El Arduino NO expondrá un servidor HTTP ni endpoints web. Todo el tráfico saliente es un *stream* UDP hacia una IP receptora (el servidor Python) y un puerto predefinidos.

- **Sin Credenciales Hardcodeadas:** La configuración de red local (MAC address, IP estática en el rango `192.168.1.10 - 192.168.1.19`, IP del servidor destino) debe estar declarada en un bloque de constantes al inicio del código para su rápida modificación en terreno.

- **Tipado Fijo:** El tamaño en bytes del `struct` de envío (payload) debe ser inmutable para que el servidor Python pueda desempaquetarlo sistemáticamente usando la librería `struct`.
