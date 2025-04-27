# the_code
[![Build all targets](https://github.com/Eagletech-robotic/the_code/actions/workflows/build.yml/badge.svg)](https://github.com/Eagletech-robotic/the_code/actions/workflows/build.yml)
[![CLang Formatting](https://github.com/Eagletech-robotic/the_code/actions/workflows/check-format.yml/badge.svg)](https://github.com/Eagletech-robotic/the_code/actions/workflows/check-format.yml)

Tout le code pour un robot.

helloworld2 est un projet STM32cubeide

Il y a 3 modules dans le code :

- IOT01A : qui contient tout ce qui est spécifique à la carte életronique
- Robotic : qui contient des outils réutilisable ailleurs
- Eaglesteward : qui contient le comportement et la configuration du robot principal de 2025

Ce qui veut dire :

- qu'en simulation sur PC, le code de IOT01A ne peut pas et ne dois pas être compiler avec
- que si on réutilise le carte électronique pour une PAMI, robotic et IOT01A doivent être réutilisé
- que si on recommence la robotique en 2026, robotic est réutilisable quelque soit le hardware, IOT01A est utilisable
  avec la carte

Le but est surtout de séparer ce qui est simulable de ce qui ne l'est pas sur PC.

Le module robotic permet de garder des codes sous le coude.

## Script de Build (`build.sh`)

`build.sh` permet d'exécuter des tâches courantes : compilation des différentes cibles (native, WASM, STM32), nettoyage des répertoires de build, ou formatage du code.

Pour voir toutes les options disponibles, exécutez :

```bash
./build.sh
```

## `host-tools`

Ce répertoire contient les programmes qui s'exécutent sur la machine de développement (l'hôte), et non sur le robot.

### `thibault_debug.cpp`

#### Compilation et exécution

```bash
./build.sh build --native --clean && \
  ./build/host-tools/thibault_debug
```

### `simulator_connector.cpp`

Ce connecteur intègre le code de guidage du robot dans un binaire WASM utilisé par notre [simulateur Web](https://github.com/Eagletech-robotic/simulator).

#### Prérequis

Installez Emscripten en suivant les instructions sur le [site officiel](https://emscripten.org/docs/getting_started/downloads.html) :

- Clonez le repository git
- Suivez la documentation, notamment l'ajout de `source (...)emsdk/emsdk_env.sh` (adaptez le chemin) dans votre fichier `.bashrc` ou `.zshrc`.

#### Compilation

```bash
./build.sh build --wasm --clean && \
  cp build-wasm/host-tools/simulator-connector.wasm ../simulator/public/
```

#### Utilisation

Le fichier WASM est utilisé par le simulateur, disponible sur http://localhost:3000 (après avoir démarré le simulateur avec `yarn start` dans le répertoire `simulator`).
