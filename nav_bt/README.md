# Behaviour Tree Using Nav2

Este módulo forma parte del sistema de patrullaje multi-robot implementado con robots Kobuki, utilizando ROS 2 y Behavior Trees para coordinar la detección y rescate de víctimas simuladas en un mapa compartido.

## Descripción

nav_bt implementa la lógica de comportamiento para un Kobuki patrullero que:

    Patrulla una zona designada según un fichero YAML de configuración.

    Detecta víctimas mediante un escaneo en puntos predefinidos.

    Comparte descubrimientos con otros Kobukis vía tópicos ROS.

    Asiste en rescates cuando otro robot detecta una víctima en su zona.

La navegación se gestiona usando Nav2 y la lógica de decisión con árboles de comportamiento (BT), diseñados y visualizados con Groot.

## Estructura del Árbol de Comportamiento

El comportamiento del robot sigue una estructura basada en un nodo raíz con fallback que gestiona dos ramas principales:

🔹 Patrullaje autónomo (izquierda)

    Comprueba si ya ha sido detectada una víctima (CheckVictim invertido).

    Si no, obtiene un nuevo waypoint dentro de su zona (GetWaypoint).

    Se desplaza al punto (Move) y escanea (Scan4Report).

    Repite hasta detectar algo o recibir notificación.

🔹 Asistencia a víctima detectada (derecha)

    Al recibir aviso de otra detección, ejecuta RescueWaypoint para obtener posición.

    Se mueve a la zona de rescate (Move).

    Espera a que todos lleguen (IsEveryoneHere), verificado mediante RetryUntilSuccessful.

    Ejecuta ResetVictim para reiniciar el estado de la víctima y volver al inicio.

## Contenido Principal

    bt_tree.xml: Estructura del árbol de comportamiento usada por Groot.

    nodes/: Nodos personalizados para el BT como CheckVictim, RescueWaypoint, Scan4Report, etc.

    config/params.yaml: Parámetros que definen zonas asignadas por robot.

    launch/: Archivos para lanzar el comportamiento con ROS 2.

## Requisitos

    ROS 2 Foxy o superior

    Nav2 Stack

    Groot (para edición visual del BT)

    Paquetes ROS personalizados para detección, publicación y sincronización de rescates.

## Ejecución

Lanzamiento del comportamiento:

ros2 launch nav_bt nav_bt.launch.py

    Asegúrate de que la navegación esté inicializada y el mapa cargado previamente.

## Comunicación entre Kobukis

Los robots se conectan a través de una VPN centralizada, más detallado en la carpeta dd, lo que permite que compartan información como:

    victim_detected: Topic publicado al detectar una víctima.

    assist_ready: Señal de preparación para el rescate conjunto.

    zones.yaml: Asigna áreas específicas a cada robot.

## Imágenes e Información Relevante

A continuación muestro la imagen de todo el BT: ![BT](https://github.com/user-attachments/assets/66b5d535-0db6-4810-a914-173a91530d89)


Breve explicación del funcionamkiento de cada nodo:

    CheckVictim: Verfica que no se haya recibido ningún topic indicando una nueva zona a la que desplazarse para asistir.

    GetWaypoint: Obtiene un nuevo punto de patrullaje dentro del área asignada al robot. Utiliza parámetros definidos en un archivo YAML.

    Move: Desplaza al robot hasta el waypoint seleccionado utilizando el planificador de Nav2.

    Scan4Report: Realiza un escaneo en busca de una posible víctima en la posición actual. Si se detecta una víctima, publica su localización (incluyendo frame_id del mapa y zona) en un tópico compartido.

    zones.yaml: Asigna áreas específicas a cada robot.

    RescueWaypoint: Calcula y proporciona la posición a la que debe desplazarse el robot para asistir en el rescate. Se basa en la información publicada por el robot que detectó la víctima.

    Move: Navega hacia el waypoint de asistencia proporcionado.

    RetryUntilSuccessful(IsEveryoneHere): Reintenta indefinidamente la verificación de que todos los robots asignados a la tarea de rescate han llegado. Una vez que todos están presentes, permite continuar el proceso.

    ResetVictim: Restablece el estado del sistema, limpiando las banderas de víctima detectada y permitiendo reiniciar el ciclo de patrullaje.

Puedes simular víctimas y patrullas en un entorno de Gazebo con los nodos de prueba incluidos en el repositorio principal.

## Licencia
## 📄 Licencia

Este proyecto está licenciado bajo los términos de la [Licencia Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0).

Copyright (c) 2025 [Asignatura Arquitectura Software Curso 2024/25]

Licencia Apache, Versión 2.0 (la "Licencia"); no se puede usar este archivo salvo en cumplimiento con la Licencia.
Puede obtener una copia en:

> http://www.apache.org/licenses/LICENSE-2.0

Salvo cuando lo requiera la ley aplicable o se acuerde por escrito, el software distribuido bajo la Licencia se distribuye "TAL CUAL", SIN GARANTÍAS NI CONDICIONES DE NINGÚN TIPO, ni expresas ni implícitas.
Consulta la Licencia para conocer el lenguaje específico que regula los permisos y limitaciones.
