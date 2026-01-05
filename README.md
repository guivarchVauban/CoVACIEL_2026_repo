# CoVaciel_2026

## Repartitions des etudiants
* Etudiant 2 = Tom
    * Decider

Fonctions
1. Conception du contrôleur latéral (PID) pour suivre la piste.
2. Gestion de la vitesse selon la courbure ou obstacles (loi d’adaptation).
3. Production des commandes de mouvement : /cmd_vel ou /servo_cmd.
4. Tests en simulation Webots (comportement, trajectoire).
5. Ajustements après premiers essais sur piste réelle.
Partie “Code”
· Node ROS 2 de décision (perception → /cmd_vel).
· Implémentation du PID (C++ ou Python).
· Scripts de test (trajectoire, courbure, réaction obstacle).
· Analyse des logs (rqt_plot ou scripts matplotlib).
Physique appliquée
· Mesure de la courbe de braquage du servo vs angle réel.
· Analyse de la réponse dynamique : sur/sous-virage, temps de réaction.
· Étude de l’influence du frottement/accélération sur la trajectoire
