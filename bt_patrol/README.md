# BT Patrol

Behaviour Tree para el comportamiento de un sistema de patrullaje.

# Overview

Sistema de patrullaje avanzado para el control de zona e identificacion de victimas mediante robots. Partiendo de un mapa, previamente dividido en "sectores"(mapas de coste), se asigna a cada robot un sector para "patrullar".
Todos los robots comparten el mismo codigo / behaviour-tree, lo que lo hace ampliable a mapas mas grandes con mas sectores y que necesiten mas robots.
La comunicacion entre robots se realiza mediante topicos en una red compartida (fastdds discovery server), con un tipo de mensaje personalizado para compartir unicamente la informacion necesaria.

![alt text][logo]

[logo]: https://github.com/maarinaa2023/proyecto-patrullaje/blob/main/bt_patrol/doc/patrolBT.jpg "mapaBT"
