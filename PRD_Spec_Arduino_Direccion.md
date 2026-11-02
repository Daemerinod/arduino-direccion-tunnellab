# PRD — Nodo sensorial Arduino de dirección

**Proyecto:** Tunnel Lab — Dirección  
**Documento:** `PRD_Spec_Arduino_Direccion.md`  
**Rol:** Especificación de restricciones de hardware industrial  
**Estado:** Borrador inicial  
**Alcance de este documento:** Restricciones absolutas del nodo. No incluye implementación en C++.

---

## 1. Propósito

Definir el contrato del Arduino de dirección como **nodo estrictamente sensorial**. El nodo adquiere medidas de orientación, posición angular de los anillos de dirección y estado de los fines de carrera, las empaqueta en un formato binario estructurado y las entrega al servidor central.

El nodo **no participa en el lazo de mando** de los motores de dirección.

---

## 2. Restricciones absolutas

Estas reglas no se relajan por conveniencia de firmware, de red ni de puesta en marcha.

| ID | Restricción | Criterio de cumplimiento |
| --- | --- | --- |
| R-01 | El Arduino opera como nodo estrictamente sensorial. | El firmware solo lee sensores y transmite telemetría. No calcula consignas, no cierra lazo y no arbitra movimiento. |
| R-02 | Queda prohibido enviar comandos físicos a los motores de dirección. | No existen salidas de mando hacia drivers, contactores, variadores ni cualquier actuador de dirección. Ningún mensaje saliente constituye una orden de movimiento. |
| R-03 | La conectividad es Ethernet con alimentación PoE e IP fija. | La dirección IP pertenece al rango `192.168.1.10`–`192.168.1.19` y no se obtiene por DHCP. |
| R-04 | La telemetría se publica en un bucle continuo. | El ciclo principal no se detiene entre lecturas ni entre envíos hacia el servidor central. |
| R-05 | El payload es binario estructurado. | Cada trama corresponde a un `struct` con las lecturas del ciclo. No se usa texto libre como formato de intercambio. |
| R-06 | El transporte es un socket TCP o UDP hacia el servidor central. | El nodo es cliente de publicación. El servidor central es el único destinatario de la telemetría. |

---

## 3. Hardware

### 3.1 Plataforma

- Placa Arduino.
- Shield Ethernet con PoE.

El shield es la única interfaz de red del nodo y la vía de alimentación prevista (PoE). No se define en este documento otro bus de campo ni otra interfaz de mando.

### 3.2 Direccionamiento

| Parámetro | Valor |
| --- | --- |
| Modo | IP fija |
| Rango autorizado | `192.168.1.10` a `192.168.1.19` |
| Fuera de rango | No permitido |

Cada nodo desplegado ocupa una única dirección de ese rango. La elección concreta dentro del rango es de asignación de planta; el firmware no puede salir de él.

### 3.3 Frontera eléctrica de actuación

El nodo no dispone de canal de mando hacia los motores de dirección. Cualquier pin, relé, PWM, bus o mensaje que pudiera mover un motor queda fuera de esta especificación y está prohibido.

---

## 4. Sensores a monitorear

El nodo solo monitorea los tres grupos siguientes.

### 4.1 IMU (I2C)

- Bus: I2C.
- Magnitudes: Roll, Pitch y Heading.
- Función: orientación del conjunto. La IMU no se usa para generar órdenes de dirección.

### 4.2 Encoders absolutos

- Función: posición angular de los anillos de dirección.
- Naturaleza: encoders absolutos. La posición de cada anillo debe poder leerse sin depender de un cero relativo reconstruido por el nodo.

### 4.3 Fines de carrera mecánicos

- Función: estado de los límites mecánicos de carrera.
- Naturaleza: contactos mecánicos. El nodo informa su estado; no los usa para accionar motores.

---

## 5. Comportamiento del firmware

El código C++ —fuera del alcance de este documento— deberá cumplir este comportamiento y ningún otro de mando:

1. Ejecutar un **bucle infinito**.
2. En cada iteración, leer los sensores del apartado 4.
3. Empaquetar esas lecturas en un **`struct` binario**.
4. Transmitir el `struct` por un **socket TCP o UDP** al servidor central.
5. Repetir el ciclo **sin interrupciones**: el bucle no se suspende a la espera de un comando de movimiento ni inserta una pausa que corte la publicación continua.

El nodo no acepta, no interpreta y no ejecuta consignas de dirección. Si el enlace con el servidor se degrada, el nodo sigue siendo sensorial: no pasa a un modo de actuación local.

---

## 6. Contrato de datos

La trama es un bloque binario con diseño de `struct`. Como mínimo transporta, por cada ciclo:

- Roll, Pitch y Heading de las IMU.
- Posición absoluta de los anillos de dirección.
- Estado de los fines de carrera mecánicos.

No se define en este paso el layout de campos, el endianness ni el puerto. Esos detalles pertenecen a la especificación de protocolo, posterior a este PRD. Lo que queda cerrado aquí es el formato: binario estructurado, no un comando.

La trama es de **observación**. No contiene campos de consigna, habilitación de drive, sentido de giro ni cualquier otro actuador.

---

## 7. Fuera de alcance

- Código fuente C++ y cualquier sketch de Arduino.
- Definición del layout binario, puertos y elección final entre TCP y UDP.
- Servidor central, visualización e historial.
- Drivers, variadores y motores de dirección.
- Estrategia de control, homing o recuperación de movimiento.

---

## 8. Criterios de aceptación del nodo

1. Con alimentación PoE y shield Ethernet, el nodo anuncia una IP fija dentro de `192.168.1.10`–`192.168.1.19`.
2. Publica de forma continua lecturas de IMU (Roll, Pitch, Heading), encoders absolutos de los anillos y fines de carrera mecánicos.
3. Cada publicación es un `struct` binario enviado por socket TCP o UDP al servidor central.
4. No existe salida ni mensaje capaz de comandar los motores de dirección.
5. El bucle de adquisición y envío no se interrumpe para ejecutar una acción física.
