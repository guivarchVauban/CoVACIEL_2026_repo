# CoVaciel_2026

## Repartitions des etudiants
* Etudiant 3 = Eliot
    * Agir

Fonctions
1. Mise en place de Docker / Docker Compose pour ROS 2 sur la RPi5.
2. Intégration des containers des étudiants 1 et 2 (perception + commande).
3. Développement firmware Arduino (PWM servo, moteur, ultrasons).
4. Node passerelle ROS2 ↔ série (communication bas-niveau).
5. Mise en place de la simulation Webots et mapping des topics.
6. Supervision ROS 2 : /tf, logs, RViz, monitoring des topics.
Partie “Code”
· Firmware Arduino (C++) : servo PWM, capteurs ultrasons.
· Node ROS 2 passerelle (Python ou C++).
· Dockerfile + docker-compose rootless ROS 2.
· Scripts de lancement ROS (launch files).
Physique appliquée
· Temps de réponse Arduino → servo (latence).
· Mesures précises de plages PWM et angle réel de braquage.
· Analyse des consommations électriques (RPI, servo, moteur)
