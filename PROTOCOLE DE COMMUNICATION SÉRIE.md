    FORMAT DE LA TRAME La commande envoyée par la Raspberry Pi à l'Arduino est une trame fixe de 4 octets : [START] [VITESSE] [DIRECTION] [CHECKSUM]

    DÉTAIL DES OCTETS

    OCTET 1 : START (Valeur : 0xFF) C’est l’octet de synchronisation. On utilise 0xFF (255) car cette valeur est impossible à trouver dans les données (vitesse et direction sont limitées à 100). Cela garantit que l'Arduino ne se décale jamais dans la lecture.

    OCTET 2 : VITESSE (Valeur : 0 à 100) Définit le mouvement du robot : De 0 à 49 : Marche arrière (0 = max, 49 = mini). 50 : Arrêt (Point mort). De 51 à 100 : Marche avant (51 = mini, 100 = max).

    OCTET 3 : DIRECTION (Valeur : 0 à 100) Définit l'angle des roues : De 0 à 49 : Braquage à gauche (0 = max). 50 : Neutre (Roues droites). De 51 à 100 : Braquage à droite (100 = max).

    OCTET 4 : CHECKSUM (Valeur : Vitesse + Direction) Sert à vérifier que la donnée n'a pas été modifiée par un parasite. L'Arduino additionne la vitesse et la direction reçues : si le résultat est différent du Checksum, il ignore la commande.

    EXEMPLE DE COMMANDE Pour avancer à 20% (valeur 70) et tourner à droite à 10% (valeur 60) : Octet 1 (Start) : 0xFF Octet 2 (Vitesse) : 0x46 (70 en décimal) Octet 3 (Direction) : 0x3C (60 en décimal) Octet 4 (Checksum) : 0x82 (70 + 60 = 130 en décimal)

Trame envoyée en hexadécimal : FF 46 3C 82
