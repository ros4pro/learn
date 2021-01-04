# IV. Perception avec Keras

tensorflow et keras sont deux modules Python qui permettent de construire des réseaux de neurones apprenants. Nous allons les utiliser ici sur des imagettes sur lesquelles sont inscrits des chiffres écrits manuellement au feutre avec différentes calligraphies. Le réseau de neurones que vous allez créer devra apprendre à déterminer quel chiffre est marqué, ce que l'on appelle **classifier**. 

## Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy

## Diapositives

{% pdf src="https://files.ros4.pro/perception.pdf", width="100%", height="565px" %}{% endpdf %}


## 0. Installation

Le code source à télécharger se trouve à [cet emplacement](https://github.com/cjlux/ros4pro_perception).

Une fois téléchargé, vous devez installer les packages Python requis en tapant la commande suivante (depuis le dossier `ros4pro_perception`) :
```
pip install -r requirements.txt
```

## 1. Documentation

Pour anlyser et compléter les codes fournis, vous aurez besoin de consulter plusieurs documentations.

De manière générale, vous aurez besoin de numpy. Si vous n'êtes pas familier avec ce module, vous pouvez consulter:
+ [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
+ [Numpy Documentation](https://numpy.org/devdocs/user/quickstart.html)

Pour la partie détection des faces des cubes et pré-processing, nous utiliserons `scikit-image`: 
+ [Scikit-Image Documentation](https://scikit-image.org/docs)

Enfin, pour la partie reconnaissance, nous utilsons le `keras` inclus dans le module `tensorflow` depuis sa version 2. Un point d'entrée sur l'API Séquentielle peut être consultée sur cette page :
* [keras API Sequential](https://www.tensorflow.org/guide/keras/sequential_model?hl=fr)

## 2. Partie apprentissage

Deux  étapes sont proposées pour une prise en main progressive de la partie *Machine Learning* (*ML*):

### a/ Travail de prise en main des **réseaux denses**, puis **convolutifs**

Deux notebooks *à trous* sont proposés pour la prise en main du *machine learning* avec `tensorflow-keras` :

* `notebook/TP1_MNIST_dense.ipynb` : pour acquérir les bases sur le *machine learning*, la banque d'images MNIST utilisée pour l'entraînement des réseaux, et la construction d'un réseau de neurones dense, son entraînement et son exploition, conduisant à un taux de reconnaissance des images MNIST voisin de 98 %.

* `notebook/TP2_MNIST_convol.ipynb` : pour la construction d'un réseau convolutif, son entraînement avc les images MNIST et son exploition, conduisant à un taux de reconnaissance voisin de 98 %.

### b/ Mise en oeuvre avec des programmes Pythons

Une fois familiarisé avec les principes de construction des réseaux denses et convolutifs abordés avec les notebooks des TP1 et TP2, vous pourrez aborder l'exploitation de vos acquis en ouvrant le fichier `src/learning.py`.

Prendre connaissance du code, puis le lancer :

### 2.1 Chargement des données

Avant d'appuyer sur entrée, répondre aux questions suivantes :

+ Que contiennent les variables `x_train` et `y_train`?

+ Pourquoi la fonction `load_data` renvoie-t-elle également les variables `x_test` et `y_test`?

+ Quelles sont les formes respectives de `x_train` et `y_train`? 

### 2.2 Prévisualisation des données brutes

Appuyer sur entrée pour continuer, et observer les images :

+ Quelles sont les valeurs des pixels blancs et des pixels noirs ? 

+ Observer les données et leurs labels. Toutes les images sont elles simples à classifier correctement? Ferriez vous des erreurs en les classifiants ?

### 2.3 Préparation des données

Fermer la fenêtre et appuyer à nouveau sur entré :

+ Quelles sont les formes respectives de `x_train` et `y_train` maintenant?

+ Pourquoi ces changements ?

### 2.4 Prévisualation des données préparées

Appuyer à nouveau sur entrée et observer les images:

+ Quelles sont les valeurs des pixels blanc et des pixels noirs maintenant?

+ Regarder la fonction `prepare_input` : quelle transformation des images est effectuée? 

### 2.5 Le modèle

Arrêter le script. Dans le fichier source `learning.py` modifier la fonction `build_model` pour implémenter le réseau *LeNet* vu dans le notebook `TP2_MNIST_convol.ipynb`. Une fois fait, relancer le script et faire défiler jusqu'à la partie 2.5 (vous pouvez modifier `SHOW_SAMPLES` pour ne pas afficher toutes les fenêtres...). Observer le résumé du modèle :

+ Observer le nombre de paramêtres par couche. Quelles sont les couches qui ont le plus grand nombre de paramètres ?

+ Qu'en concluez vous sur l'utilité des couches par convolution ?

### 2.6 La fonction de coût et l'optimiseur

Arrêter le script.

Dans le notebook `TP1_MNIST_dense.ipynb` nous avons vu que la fonction de coût **entropie croisée** (*cross entropy*) était la plus adaptée au calcul d'erreur avec des sorties au format *hot-one*.

Vérifier que c'est bien la fonction de coût utilisée dans l'appel à `modele.compile(...)`.

L'optimiseur est utilisé pour se déplacer vers un minimum sur la surface déssinée par la fonction de coût dans l'espace des paramêtres. 

Un des algorithmes les plus simples est la __descente de gradient__ (GD) et consiste à se déplacer dans le sens de la pente la plus forte à chaque pas de temps :

+ Dans quelle hypothèse cet algorithme permet-il de trouver le minimum global de la fonction de coût selon vous ?

+ Pensez vous que cette hypothèse soit vérifiée pour les réseaux de neurones ?

+ Que se passe-t-il si cette hypothèse n'est pas vérifiée ? 

__Adam__ est un optimiseur plus complexe que GD. Sur l'image suivante, on voit plusieurs optimiseurs se déplacer sur une fonction de coût.
![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie11.gif)
(source : github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie11.gif)

Concentrez-vous sur Adam et GD:

+ Quelle semble être la caractéristique de Adam comparée a GD ?

Une autre caracteristique de l'algorithme GD, est que la taille du pas qui est effectué à chaque itération est fixe. L'image suivante montre Adam et GD dans un cas ou la pente devient trés forte. 
![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie9.gif)
(source : github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie9.gif)

Répondre aux questions suivantes: 

+ GD arrive-t-il à converger ? Comprenez vous pourquoi ?

+ Adam ne semble pas soumis au même problême que GD ? Quelle autre caractéristique de Adam cela montre t il? 

pour finir :

+ Quelle conclusion pouvez vous tirer sur l'utilité de GD pour entrainer des réseaux de neurones ?

+ Quel est l'optimiseur utilisé dans le code ?

### 2.7 Entraînement

* Observer la fonction `train_model` pour vérifier la mise en place du mécanisme **keras** permettant de gérer le sur-entraînement (*over-fit*).

* Relancer le code et appuyer sur entrée jusqu'au déclenchement de la partie 2.7...
Vous devriez voir les itérations d'entraînement se succéder et s'arrêter automatiquement par **early stopping**.


### 2.8 Poids appris

Appuyer sur entrée pour visualiser les noyaux convolutifs appris par le réseau de neurones :

+ En observant les noyaux de la première couche, arrivez vous à distinguer le genre de *features* qui seront extraites par chacun ?

+ Pouvez vous en faire de même pour la deuxième couche ?

### 2.9 Activations

Appuyer sur entrée, puis entrer un indice (un entier de n'importe quelle valeur inferieure a 12000) :

+ Après la première couche de convolution, les *features* extraites correspondent elles à celles que vous imaginiez ?

+ Après la première couche de *pooling*, les *features* présentes auparavant sont-elles conservées ?

+ Après la deuxième couche de *pooling*, diriez vous que de l'information spatiale est toujours présente ? Autrement dit, les activations ressemblent elles toujours à des images ?

### 2.10 Entraînement final

Arrêter le script. Jusqu'à présent, nous avons travaillé sur l'ensemble des données, mais pour la suite nous n'aurons besoin que des images de 1 et de 2 :

* Changez la valeur de la variable `CLASSES` pour ne garder que les classes qui nous intéressent, 

* Changer `SHOW_SAMPLES`, `SHOW_WEIGHTS` et `SHOW_ACTIV` si vous désirez moins d'affichage graphique,

* Entraîner le réseau, puis le sauvegarder en donnant le nom d'un répertoire où sauvegarder les fichiers du réseau entraîné.

Vous pouvez passer maintenant à la **Partie Vision** qui permettra, une fois achevée, d'observer les prédictions du réseau avec les images des cubes correctement traitées... 

## 3. Partie Vision

Ouvrez le fichier `src/detection.py`.

### 3.1 Présentation des données

Démarrer le script. Une des images d'exemple issue du robot devrait vous être présentée :

+ Observer les valeurs de pixels ? Quelles sont les valeurs de pixels blancs et noirs ?
+ De manière générale, la face des cubes est elle semblable aux images MNIST ? 

### 3.2 Binarisation de l'image

Appuyer sur entrée, et vous devriez voir s'afficher une image binarisée :

+ Pouvez vous penser à un algorithme permettant d'arriver à un résultat similaire ?

Dans le code, observer la fonction `binarize`:

+ À quoi sert la fonction `threshold_otsu` ? Aidez vous au besoin de la documentation de `scikit-image`.
+ Ajouter une ligne pour afficher la valeur de `thresh` à chaque appel de la fonction. Cette valeur est elle la même pour toutes les images ?

En commentant successivement les lignes les utilisant, observer l'impact de chacune des fonctions suivantes :

+ À quoi sert la fonction `closing` ?
+ À quoi sert la fonction `clear_border` ?
+ À quoi sert la fonction `convex_hull_object` ?
+ Conclure en résumant l'enchaÎnement des opérations effectuées dans la fonction `binarize`.

Ne pas hésiter à vous aider de la documentation de `scikit-image`.

### 3.3 Recherche de contours

Appuyer sur entrée pour faire défiler quelques images dont les contours ont été détectés.

Observer la fonction `get_box_contours`:

+ À quoi sert la fonction `label` ?
+ À quoi sert la paramètre `area` ?
+ À quoi sert la fonction numpy `argsort` utilisée à la fin pour le ré-arragement des contours ? Pourquoi cette opération est elle importante ?
+ Conclure en résumant l'enchaînement des opérations effectuées dans la fonction `get_box_contours`.

### 3.4 Extraction des vignettes

Appuyer sur entrée pour faire défiler quelques images dont les vignettes ont été extraites.

Observer la fonction `get_sprites`:

+ Qu'est ce qu'une "transformation projective" ?

### 3.5 Préparation des images

Pendant la phase d'apprentissage, nous avons étudié la préparation qui était faite des images. 

Les vignettes que nous allons présenter au réseau de neurones doivent aussi subir une préparation pour avoir les mêmes caractéristiques que les images d'entrainement MNIST :

* Remplir la fonction `preprocess_sprites` pour effectuer cette préparation.

Une fois que cela est fait, executez le script jusqu'à la fin.

Vous pouvez maintenat ouvrir le fichier `main.py` pour tester l'intégration de la détection et de la reconnaissance...


## 4. Intégration

Il est maintenant temps d'intégrer les deux parties du pipeline pour l'utilisation finale. Ouvrez le fichier `main.py` à la racine du projet. 

Pour que les deux parties du pipeline s'adaptent correctement, vous avez complété la fonction `preprocess_sprites` qui permet de mettre les vignettes renvoyées par la partie détection dans un format compatible avec celui des images MNIST.

Exécuter le programme `main.py` : donner le nom d'un dossier qui contient les fichiers des poids du réseau entraîné et vous devriez commencer à obtenir la reconnaissance des chiffres '1' et '2' des images fournies. Il faudra certainement refaire plusieurs fois l'entraînement du réseau en jouant sur plusieurs paramètres avant d'obtenir un réseau entraîné qui fonctionne correctement.
