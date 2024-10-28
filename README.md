# the_code
tout le code pour un robot

helloworld2 est un projet STM32cubeide

Il y a 3 modules dans le code :
- IOT01A : qui contient tout ce qui est spécifique à la carte életronique
- Robotic : qui contient des outils réutilisable ailleurs
- Eaglesteward : qui contient le comportement et la configuration du robot principal de 2025

Ce qui veut dire :
- qu'en simulation sur PC, le code de IOT01A ne peut pas et ne dois pas être compiler avec
- que si on réutilise le carte électronique pour une PAMI, robotic et IOT01A doivent être réutilisé
- que si on recommence la robotique en 2026, robotic est réutilisable quelque soit le hardware, IOT01A est utilisable avec la carte

Le but est surtout de séparer ce qui est simulable de ce qui ne l'est pas sur PC.

Le module robotic permet de garder des codes sous le coude. 