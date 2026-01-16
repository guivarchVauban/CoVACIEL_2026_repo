# A faire

Ce code est tout de même assez complexe et peu commenté, il y a beaucoup de fonctions et **il ne sera pas vraiment approprié au contrôleur final**
J'ai demandé à chatgpt de me faire du code non bloquant pour la fonction démo, il y a pour gérer ça une programmation de type automate à état avec un switch case, c'est très intéressant et pratique mais pas forcément simple à comprendre.
C'est la même chose pour la génération de rampe d'accéleration pour le moteur, c'est non bloquant avec la fonction updateRamp qui utilise les interruptions millis(),pas évident non plus.

Pourtant il va falloir que tu t'assures que tu comprends l'intégralité de ce code, pour cela je veux que tu :

- Commente chacune des fonctions en mode documentation Doxygen (@brief, @param...)
- Rajoute des commentaires à la main là ou ça te semble nécessaire
