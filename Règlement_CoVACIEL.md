# Règlement CoVACIEL + données importantes

## La voiture

- [ ] Le moteur et le chassis doivent être ceux d'origine, **on ne peut pas les remplacer**
- [ ] La batterie doit être une NiMH 7.2V avec capacité de 5000mAh maximum
- [ ] La carrosserie doit recouvrir la voiture à **minimum 80%**

### Dimensions (capteurs compris)

- [ ] Hauteur : 115mm < Hauteur < 190mm
- [ ] Arrière : Surface minimum de 150mm de large & 110mm de hauteur avec maximum 25mm de vide au dessus du sol pour la garde au sol
- [ ] Longueur : 400mm < Longueur < 480mm
- [ ] Largeur : 160mm < Largeur < 190mm

- [ ] Vérifier si enlever la transmission avant (roues avant uniquement motrice) pour augmenter le rayon de braquage est favorable ou non
- [ ] La voiture doit posséder un module sans fil pour recevoir le signal de début de course + envoi de données pratiques si besoin (pour débuggage)
- [ ] Marche avant + arrière fonctionnels
- [ ] 2ème batterie uniquement pour l'électronique autorisée. Il est interdit d'alimenter le moteur avec si elle ne rentre pas dans les contraintes

## La piste 

- Le tracé n'est pas connu avant le jour de la course. Il est interdit de donner des infos sur la forme de la course a la voiture

- Les murs de la piste font **200mm** de hauteur, de couleur **verte a droite** RAL 6037 et **rouge a gauche** RAL 3020, les virages feront minimum R=400mm de rayon de courbure

- La piste est en tout point d'une largeur supérieure à 800mm, mais peut contenir des obstacles à l'intérieur (?)

## Homologation 

### Vérifications

- [ ] Dimensions de la voiture
- [ ] Detection du signal de départ (ASCII **$GO** & **STOP**)
- [ ] Détectabilité de la voiture par un lidar et capteur
- [ ] Capacité à repartir en marche arrière en cas de bloquage
- [ ] La voiture est détectable par un lidar

## Départ

Le top départ est transmis par communication sans fil, les 2 messages envoyés seront "**$GO**" et **STOP** pour le début et l'arrêt des voitures.

Le signal est envoyé par un appareil XBEE de configuration : 

    firmware: 802.15.4;
    coordinateur;
    mode transparent;
    identifiant du réseau PAN (PAN-ID) = 1234;
    Canal (CH) = C.

## Qualifications

2 manches de qualifications par voiture, seule sur le circuit

2 tours par manche de qualification

1er tour sans obstacles sur une piste A
2eme tour avec obstacles sur une piste B (taille obstacle > voiture)

Noté selon le pourcentage de tours effectués, ou par temps de complétion

Si la voiture se bloque, un arbitre la débloque

Les résultats (=classement) détermine la position sur la grille de départ

## Course

3 minutes pour installer la voiture sur la piste

Dès que toutes les équipes sont prêtes, plus le droit de toucher la voiture.

Top départ donné par le module XBEE

 ### Disqualifications

- Comportement aggressif
- Empêchement de dépasser
- Voiture immobile plus de 10s sans voiture la bloquant
- Contre-sens sur plus de 2m
- Voiture en marche arrière avec une autre voiture derrière elle.

Seuls les arbitre peuvent toucher les voitures lors de la course

## Points

*1er* **25 pts** 
*2nd* **18 pts** 
*3ème* **15 pts** 
*4ème* **12 pts** 
*5ème* **10 pts** 
*6ème* **8 pts** 
*7ème* **6 pts** 
*8ème* **4 pts** 
*9ème* **2 pts** 
*10ème* **1 pts**

En cas d'égalité, les temps des qualifications sont pris en compte.
