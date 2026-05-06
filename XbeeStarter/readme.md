# XBee Starter

## Extrait du règlement

*Le top départ*

*Le top départ doit être transmis aux voitures par une communication sans fil. Cette communication sans fil permet uniquement de transmettre le top départ et ne doit pas permettre de commander le véhicule à distance. Chaque équipe est libre d'utiliser la solution technologique de son choix pour donner le top départ à sa voiture. Le jour de CoVACIEL, lors des phases d'homologation, de qualification et de course, un top départ est accessible à tous les véhicules qui le souhaitent. Ce top départ est donné par un module XBEE. La configuration du module XBEE donnant le top départ est la suivante:*

    * firmware: 802.15.4;
    * coordinateur;
    * mode transparent;
    * identifiant du réseau PAN (PAN-ID) = 1234;
    * Canal (CH) = C.

*Le message envoyé en broadcast par le XBEE lors du top départ est composé des 4 caractères ASCII suivants: $GO; . Un autre message envoyé également en broadcast par le XBEE permet d'arrêter les voitures. Ce message d'arrêt des voitures est composé des 4 caractères ASCII suivants: STOP .*

## Ce dossier contient :

### Le programme Arduino du simulateur de Top départ Xbee
A modifier pour que le Top départ et le Stop se lancent en fonction d'appui sur des boutons lumineux

### Les configurations XCTU pour les modules XBee
Configuration conforme au réglement du concours Covaciel
