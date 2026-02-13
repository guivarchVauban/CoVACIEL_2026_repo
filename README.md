# CoVaciel_2026

## Repartitions des etudiants

* Etudiant 1 = Baptiste
    * Percevoir
Fonctions
1. Acquisition et prétraitement de la caméra (détection de bord de piste
rouge/vert).
2. Lecture et filtrage des données LiDAR Slamtech A1 dans ROS 2.
3. Analyse des obstacles arrière via les ultrasons (topic ROS2).
4. Extraction des informations utiles :
o décalage latéral /lane_offset
o orientation de piste
o distance aux obstacles
5. Publication des données sur des topics ROS 2 dédiés.
6. Validation en simulation dans Webots et sur la voiture réelle.
Partie “Code”
· Node ROS 2 pour traitement image (OpenCV).
· Node ROS 2 pour traitement LiDAR (/scan → /obstacle_distance).
· Node ROS 2 pour fusion de données capteurs.
Physique appliquée
· Mesures de distance réelle vs distance LiDAR (erreurs, précision).
· Courbe de réponse des capteurs ultrasons selon angle et matériau.
· Étude des conditions lumineuses et impact sur la vision
