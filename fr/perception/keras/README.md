# IV. Perception avec Keras

Keras est un système d'apprentissage automatique utilisant des réseaux de neurones. Nous allons l'utiliser ici sur des imagettes sur lesquelles sont inscrites des chiffres marqués manuellement au feutre avec différentes calligraphies. Le réseau de neurones que vous allez créer devra apprendre lui-même à déterminer quel chiffre est marqué, ce que l'on appelle **classifier**. 

## Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy

## Diapositives

{% pdf src="https://files.ros4.pro/perception.pdf", width="100%", height="565px" %}{% endpdf %}


## 0. Installation

Le code source se trouve à [cet emplacement](https://gitlab.inria.fr/apere/ros_workshop/-/tree/master).

Pour effectuer cet atelier, vous devez installer quelques packages:
```
pip install tensorflow keras imageio matplotlib scikit-image numpy
git clone https://gitlab.inria.fr/apere/ros_workshop.git
```

## 1. Documentation

Pour manipuler le code, vous aurez besoin de consulter plusieurs documentations. De manière générale, vous aurez besoin de numpy. Si vous n'êtes pas familier avec, vous pouvez consulter:
+ [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
+ [Numpy Documentation](https://numpy.org/devdocs/user/quickstart.html)
Pour la partie détection et pré-processing, nous utiliserons `scikit-image`: 
+ [Scikit-Image Documentation](https://scikit-image.org/docs)
Enfin, pour la partie reconnaissance, nous utiliserons `keras`. Vous devriez trouver ce qu'il vous faut dans le tutoriel suivant:
+ [Keras Documentation](https://keras.io/)

## 2. Partie apprentissage

Commencez par ouvrir le fichier `learning.py`.

### 2.1 Chargement des données

Prenez connaissance du code, puis lancez le en exécutant à la ligne de commande:
```
python learning.py
```
Avant d'appuyer sur entrée, répondez aux questions suivantes: 
+ Que contiennent les variables `x_train` et `y_train`?
+ Pourquoi la fonction `load_data` renvoie-t-elle également les variables `x_test` et `y_test`?
+ Quelles sont les formes respectives de `x_train` et `y_train`? 

### 2.2 Prévisualisation des données brutes

Appuyez sur entrée pour continuer, et observez les images. Répondez aux questions suivantes:
+ Quelle sont les valeurs des pixels blancs (représentés en jaune) et des pixels noirs ? 
+ Observez bien les données et leurs labels. Toutes les images sont elles simples à classifier correctement? Ferriez vous des erreurs en les classifiants ?

### 2.3 Préparation des données

Fermez la fenêtre et appuyez à nouveau sur entrée. Répondez aux questions suivantes:
+ Quelle sont les formes respectives de `x_train` et `y_train` maintenant?
+ Pourquoi ces changements ?

### 2.4 Prévisulation des données préparées

Appuyez à nouveau sur entrée et observez les images:
+ Quelles sont les valeurs des pixels blanc et des pixels noirs maintenant?
+ Regardez la fonction `prepare_input`. Quelle transformation des images est effectuée? 

### 2.5 Le modêle

Arrêtez le script en appuyant sur `ctrl+c`. Dans le fichier, modifiez la fonction `build_model` pour implémenter le réseau _LeNet_ vu pendant la présentation. Une fois cela fait, relancez le script et faites défiler jusqu'à la partie 2.5. Observez le résumé du modêle: 
+ Observez le nombre de paramêtres par couche. Quelles sont les couches qui ont le plus grand nombre de paramêtre? 
+ Qu'en concluez vous sur l'utilité des couches par convolution ?

### 2.6 La fonction de cout et l'optimiseur

Arrêtez le script en appuyant sur `ctrl+c`. Durant la présentation, nous avons vu que deux fonctions de coût sont régulièrement utilisées dans l'entrainement des réseaux de neurones:
+ L'érreur quadratique moyenne (Mean squared error)
+ L'entropie croisée (Cross entropy)
Dans le fichier, modifiez la fonction `get_loss` pour implémenter la fonction de coût adaptée à notre problême.

Pendant la présentation, nous avons vu que l'optimiseur est l'algorithme qui permet de se déplacer sur la surface déssinée par la fonction de cout dans l'espace des paramêtres. Cet algorithme permet de chercher l'endroit ou la fonction de cout est minimale. 

Un des algorithmes les plus simples s'appelle la __descente de gradient__ (GD) et consiste à se déplacer dans le sens de la pente la plus forte à chaque pas de temps:
+ Dans quelle hypothèse cet algorithme permet il de trouver le minimum global de la fonction de coût selon vous ?
+ Pensez vous que cette hypothèse soit vérifiée pour les réseaux de neurones ?
+ Que se passe-t-il si cette hypothèse n'est pas vérifiée ? 

__Adam__ est un optimiseur plus complexe que GD. Sur l'image suivante, on voit plusieurs optimiseurs se déplacer sur une fonction de cout.
![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie11.gif)

Concentrez-vous sur Adam et GD:
+ Quelle semble être la caractéristique de Adam comparée a GD? 

Une autre caracteristique de l'algorithme GD, est que la taille du pas qui est effectué à chaque itération est fixe. L'image suivante montre Adam et GD dans un cas ou la pente devient trés forte. 
![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie9.gif)

Repondez aux questions suivantes: 
+ GD arrive-t-il à converger ? Comprenez vous pourquoi ?
+ Adam ne semble pas soumis au même problême que GD ? Quelle autre caractéristique de Adam cela montre t il? 

Enfin: 
+ Quelle conclusion pouvez vous tirer sur l'utilité de GD pour entrainer des réseaux de neurones ?
+ Quel est l'algorithme utilisé dans le code ?

### 2.7 Entrainement

Relancez le code et appuyez sur entrée jusqu'au déclenchement de la partie 2.7. Vous devriez voir les itérations d'entrainement se succéder:
+ Observez l'évolution de la précision sur l'ensemble d'entrainement et l'ensemble de test. Les valeurs sont elles identiques ?
+ À partir de combien d'époques le réseau est il entrainé selon vous ?
+ En réglant le nombre d'itération d'apprentissage dans le code (argument `epochs` de la fonction fit), arrivez vous à observer une phase de sur-apprentissage ?

### 2.8 Poids appris

Appuyez sur entrée pour visualiser les noyaux appris par le réseau de neurones:
+ En observant les noyaux de la première couche, arrivez vous à distinguer le genre de features qui seront extraites par chacun?
+ Pouvez vous en faire de même pour la deuxième couche ?

### 2.9 Activations

Appuyez sur entrée, puis rentrez un indice (un entier de n'importe quelle valeur inferieure a 12000):
+ Après la première couche de convolution, les features extraites correspondent elles à celles que vous imaginiez ?
+ Après la première couche de pooling, les features présentes auparavant sont elles conservées ?
+ Après la deuxième couche de pooling, diriez vous que de l'information spatiale est toujours présente ? Autrement dit, les activations ressemblent elles toujours à des images ?

### 2.10 Entrainement final

Arrêtez le script en appuyant sur `ctrl+c`. Jusqu'à présent, nous avons travaillé sur l'ensemble des données, mais pour la suite nous n'aurons besoin que des images de 1 et de 2. Changez la valeur de la variable `CLASSES` pour ne garder que les classes qui nous intéressent, entrainez en réseau, puis sauvegardez le dans un fichier.

## 3. Partie vision

Ouvrez le fichier `detection.py`.

### 3.1 Présentation des données

Démarrez le script. Une des images d'exemple issue du robot devrait vous être présentée:
+ Observez les valeurs de pixels ? Quelles sont les valeurs de pixels blancs et noirs ?
+ De manière génerale, la face des cubes est elle semblable aux images de mnist ? 

### 3.2 Binarisation de l'image

Appuyez sur entrée, et vous devriez voir s'afficher une image binarisee:
+ Pouvez vous penser à un algorithme permettant d'arriver à un résultat à peu prés similaire ?

Dans le code observez la fonction `binarize`:
+ A quoi sert la fonction `threshold_otsu` ? Aidez vous de la documentationde `scikit-image`.
+ Ajoutez une ligne pour afficher la valeur de `thresh` à chaque appel de la fonction. Cette valeur est elle la même pour toutes les images ?

En commentant successivement les lignes les utilisant, décrivez l'impact de chacune des fonctions suivantes:
+ A quoi sert la fonction `closing` ?
+ A quoi sert la fonction `clear_border` ?
+ A quoi sert la fonction `convex_hull_object` ?
+ Concluez en résumant l'enchainement des opérations effectuées dans la fonction `binarize`.

N'hesitez pas à vous aider de la documentation de `scikit-image`.

### 3.3 Recherche de contours

Appuyez sur entrée pour faire défiler quelques images dont les contours ont été détectés.

Observez la fonction `get_box_contours`:
+ A quoi sert la fonction `find_contour` ?
+ A quoi sert la fonction `approximate_square` ? Sur quelle fonction de `scikit-image` repose-t-elle ?
+ A quoi sert la fonction `reorder_contour` ? Pourquoi cette opération est elle importante ?
+ Concluez en résumant l'enchainement des opérations effectuées dans la fonction `get_box_contours`

### 3.4 Extraction des vignettes

Appuyez sur entrée pour faire défiler quelques images dont les vignettes ont été extraites.

Observez la fonction `get_sprites`:
+ Qu'est ce qu'une transformation projective ? 
+ Regardez l'ordre des points source et repensez à la fonction `reorder_contour`. Son importance est elle plus claire maintenant?
+ Dans quelle limites d'orientation, le cube sera-t-il réorienté correctement ?

### 3.5 Préparation des images

Pendant la phase d'apprentissage, nous avons étudié la préparation qui était faite des images. Les vignettes que nous allons présenter au réseau de neurones doivent aussi subir une préparation pour avoir les mêmes caractéristiques que les images d'entrainement. Remplissez la fonction `preprocess_sprites` pour effectuer cette préparation.

Une fois que cela est fait, executez le script jusqu'à la fin.

## 4. Intégration

Une fois terminé, vous pouvez vous rendre dans `main.py` pour tester l'intégration de la détection et de la reconnaissance. Bravo !
