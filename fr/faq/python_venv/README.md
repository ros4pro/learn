## 🔨 Créer et utiliser un Environnement Virtuel Python (EVP)

### Intérêt

Un Environnement Virtuel Python (EVP) procure un environnement informatique étanche contenant une installation de Python :

* indépendante des autres environnements Python susceptibles de coexister sur la même machine,
* indépendante des mises à jour de l’ordinateur.

Un EVP repose sur la création d’une arborescence disque dédiée qui héberge la version de Python et des modules dont tu as besoin pour ton projet.

On peut effacer et re-créer un EVP très facilement, sans que cela n'impacte la distribution Python système installée avec Ubuntu.

Quand tu actives un EVP sous Linux, la variable d’environnement PATH est modifiée de sorte que l’interpréteur Python et tous les modules soient recherchés dans l’arborescence dédiée à cet EVP et nulle part ailleurs.

### Créer un EVP avec `conda`

Plusieurs outils permettent de créer EVP, en particulier  :

* la commande `conda` , disponible si tu as installé Python avec la distribution [miniconda](https://docs.conda.io/en/latest/miniconda.html).
* le module Python `venv` qui permet de créer un EVP (cf [venv](https://docs.python.org/3/library/venv.html)).

L'intérêt de `miniconda` pour le calcul numérique est que cette distribution installe de façon transparente la bibliothèque [MKL](https://software.intel.com/content/www/us/en/develop/tools/oneapi/components/onemkl.html), qui fournit l'optimisation pour les processeurs Intel des bibliothèques d'algèbre linéaire (BLAS, Lapack...) à la base des performances du module numpy.

### Étapes de création d'un EVP avec `conda`

1. Télécharge et installe la distribution [miniconda](https://docs.conda.io/en/latest/miniconda.html) ...

    * La seule précaution est d'installer le répertoire d'installation `miniconda3` dans un chemin qui ne comporte ni espace ni caractère accentué.
    * En général, le chemin ressemble à `/home/<logname>/miniconda3/`.
    * À la fin de l'installation réponds `yes` à la question `Do you wish the installer to initialize Miniconda3 by running conda init? [yes|no]`
    * Lance un nouveau terminal ou tape la commande `source ~/.bashrc` pour hériter des modifications du fichier `.bashrc`.
    * Une fois `conda` installé, désactive le lancement automatique de l'EVP `(base)` en tapant la commande : `conda config --set auto_activate_base false`.

1. Créé l'EVP avec la commande `conda create -n <nom_evp> python=<version>`

    * `<nom_environnement>` : nom (libre) de ton EVP : par exemple `pyml`(pour Python mechine learning)
    * `<version>` :  version de Python que tu veux installer dans ton EVP : par exmple `3.6` ou `3.6.8` ou `3.8`...

1. Active ton EVP avec la commande `conda activate <nom_evp>` :

    * L'activation de l'EVP se traduit par le préfixage du *prompt* avec la chaîne : `(<nom_environnement>)`.
    Par exemple si le *prompt* courant est `user@host $`, l'activation de l'EVP nommé `pyml` modifie le prompt qui devient : `(pyml) user@host $`

1. Charge les modules Python dans ton EVP **activé**

    Avec ton **EVP activé** utilise `conda install <module>`  ou `pip install <module>` pour installer le module Python  `<module>`.

    :question: `conda install...` ou `pip install...` :arrow_right: la règle est simple :

    * commence de préférence par `conda install...`, qui va installer une version optimisée du module Python si elle est connue de `conda`
    * si `conda install...` échoue, utilise alors `pip install...`"

## 🔨 Comment fonctionne un EVP

Quand l'EVP `<evp>` est activé :

* la variable d'environnement `PATH` est modifiée pour mentionner en premier :
    * le répertoire permettant d'accéder à la commande `conda` : par exemple `/home/<logname>/miniconda3/condabin/`
    * le répertoire associé à l'environnement `<evp>` : par exemple `/home/<logname>>/miniconda3/envs/<evp>`
* toutes les commandes liées à Python (`python`, `conda`, `pip`...) sont recherchées en premier dans ces deux répertoires.
* toute installation d'un module Python par `conda` ou `pip` installe le module dans le l'arboresence `/hoem/<logname>/envs/<evp>/...`

## 🔨 Quels modules Python  installer dans mon EVP pour l'entraînement des réseaux de neurones?

La journée **Perception** du workshop ROS nécessite de travailler avec les modules **tensorflow** et **keras** :
l'idéal est de les installer dans ton **EVP activé** avec les commandes suivantes :

* conda install numpy==1.19.4 scipy==1.5.2
* pip install tensorflow==2.4.0
* pip install opencv-python==4.5.1.48
* pip install matplotlib pandas jupyter testresources sklearn scikit-image seaborn pydot

## 🔨 EVP : commandes utiles

* Afficher des informations sur la distribution **conda** : `conda info`

* Lister les EVP connus par **conda** : `conda env list`

* Désactiver l'EVP courant : `conda deactivate`

* Activer l'EVP nommé `<evp>` : `conda activate <evp>`

* Avec ton **EVP activé** :

    * Lister les paquets installés : `conda list` ou `pip freeze`

    * Rechercher les versions d'un module Python pour l'EVP courant activé:

        * `conda search <module>` : cherche les versions de `<module>` connues par conda
        * `pip search <module>` : cherche les versions de `<module>` disponibles pour ton EVP sur le site `pypi`.

* Mettre à jour la commande **conda** : `conda update -n base -c defaults conda`
