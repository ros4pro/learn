# ROS4PRO : Journée Perception
## Installation

💻📀  Pour effectuer cet atelier, vous devez installer quelques paquets. Pour cela,
positionnez vous à la racine du dépôt et exécutez la ligne suivante:
```
pip install -r requirements.txt
```

## Paquets et documentation

📚 Pour effectuer ce travail, vous aurez besoin de manipuler majoritairement 4
bibliothèques python de calcul scientifique:
+ _numpy_: Cette bibliothèque propose un tableau n-dimensionnel, ainsi que des
  opérations pour les manipuler. Ces opérations sont codées en C pour une 
  meilleure performance (python lui-même est trés lent). Ce tableau est __la__ 
  structure de donnée la plus importante pour le machine learning en python. 
  Dans notre cas, les images seront toujours encodées sous forme de tableau 
  numpy. Attention: Les tableaux numpy et les listes python sont des objets
  différents. Il arrive que dans le code, on doive passer de l'un à l'autre.
  Faites bien attention à garder en tête le type de vos objets quand vous
  réfléchissez à votre code.
+ _scipy_: Cette bibliothèque propose un ensemble d'algorithme de calcul
  scientifique de base, rapides car souvent développés en C également. Scipy 
  utilise également les tableaux numpy comme structure de donnée principale.
+ _scikit-image_: Cette bibliothèque propose un ensemble d'algorithmes de
  traitement d'images développés en python (semblable à `opencv`). Cette
  librairie contient beaucoup d'algorithmes, et nous en utiliserons certains.
  Notez que dans certains cas, les algorithmes peuvent être assez lent à
  exécuter. Dans ce cas, il peut être intéressant de se tourner vers scipy si
  les mêmes algo sont disponibles.
+ _tensorflow-keras_: Cette bibliothèque propose un ensemble d'outils permettant de
  construire des réseaux de neurones. En outre, elle fait essentiellement deux
  choses: Elle permet de dériver automatiquement le code de la passe backward
  à partir du code de la passe forward, et elle permet d'exécuter ce code sur
  des ressources hétérogènes comme des gpus, ou des grappes de calcul. Dans notre
  cas, nous n'utiliserons que les cpu. Depuis la version 2.0 (que nous
  utiliserons), tensorflow utilise également des tableaux numpy comme structure
  de donnée.

📚 Il existe de nombreuses ressources pour chacune de ces bibliothèques, mais je vous
conseille de regarder celles ci si vous êtes coincés:
+ [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
+ [Numpy Documentation](https://numpy.org/devdocs/user/quickstart.html)
+ [Scikit-Image Documentation](https://scikit-image.org/docs)
+ [Keras Documentation](https://keras.io/)
+ [Scipy ndimage documetation](https://docs.scipy.org/doc/scipy/reference/ndimage.html)

## 1. Partie Détection

Dans cette partie, vous allez développer et mettre au point un système de
détection de cubes dans une image. A partir d'une image capturée par votre Poppy Ergo Jr,
telle que celle-ci :

![img](../img/12.png)

Vous devrez écrire un programme permettant d'extraire une (ou plusieurs si
besoin) imagettes comme celle-ci :

![img](../img/sprite.png)

🐍 Ouvrez le fichier `src/detection.py`. Durant toute la durée du tp, vous 
devrez lancer le script en exécutant la ligne de commande:
```shell
$ python detection.py
```

❌ Notez qu'à tout moment, vous pouvez interrompre le script en appuyant sur 
`Ctrl-C` deux fois d'affilée. Il se peut également qu'il faille fermer les
fenêtres encore ouverte pour que le programme se termine correctement.

🐍 En python, il n'existe pas de fonction `main` comme dans d'autre language. Dans
notre cas, le script démarre à partir de la ligne suivante:
```python
#---------------------------- MAIN

if __name__ == "__main__":

    # Le script démarre son exécution içi.
    # ...
```

♯ N'hésitez pas à commenter les sections que vous avez déjà effectué pour que le
script s'exécute plus vite à chaque fois.

### 1.1 Présentation des données

Le script commence par charger des données de test préalablement capturées, dans
un environnement contrôlé. Cependant, la fonction `show_image` qui permet
d'afficher ces images nécessite que vous remplissiez quelques fonctions, qui
manipulent des tableaux numpy.

🐍 Remplissez les fonctions:
+ `get_image_min`
+ `get_image_max`
+ `get_image_shape`
+ `get_image_dtype`

Une fois ces fonctions remplies, vous pouvez démarrer le script. Les images
d'exemple devraient alors vous être présentées.

✍ Répondez aux questions suivantes:
+ Quelle est la forme du tableau ? À quoi corresponde ces dimensions ?
+ Observez les valeurs minimales et maximales du tableau. Sont elles les mêmes pour toutes les images ? 
+ Quel est le type des données enregistrées dans le tableau numpy? Pourquoi est-ce le cas? Ce type de donnée est il adapté aux réseaux de neurones ? 
+ De manière générale, trouvez vous que la face des cubes est semblable aux images du dataset mnist que nous allons devoir utiliser pour l'apprentissage ? 

### 1.2 Segmentation de l'image

Maintenant, nous allons passer à la segmentation de l'image. L'objectif de cette
étape est de détecter, pour chacun des pixel, si il appartient à un cube, ou si
il appartient au fond de la scène. Pour ce faire, nous allons utiliser une
approche classique nommée le __thresholding__. Les grandes étapes de
l'algorithme sont les suivantes:
+ On transforme l'image en niveaux de gris
+ On crée une image binaire grâce à une opération de seuillage
+ On nettoie l'image binaire des artefacts du seuillage
+ On sépare les différentes zones blanches de l'image binaire
+ On nettoie chacune des zones blanches pour qu'elle se rapproche d'une face de
  cube blanc pleins.
  
Cet algorithme est contenu dans la fonction `segment_thresholding`, qui doit
renvoyer une liste d'images binaires contenant chacune le masque d'un des cubes
présent sur l'image.

👀 Observez la fonction `segment_thresholding`. La première étape de la
segmentation consiste à transformer l'image, jusqu'alors en couleurs, en une
image en niveaux de gris.

🐍 Remplissez le code de la fonction `turn_to_grey`.

✍ Répondez aux questions suivantes : 
+ Pouvez vous penser à plusieurs manière de transformer une image en RGB en une image en niveaux de gris ? 
+ Y a t il une différence à utiliser une méthode en particulier ?

Une fois la fonction remplie, l'exécution du script devrait montrer vos images
en noir et blanc.

✍  Répondez aux questions suivantes :
+ Quelle est la taille de l'image maintenant ? 
+ Quels sont les valeurs maximales et minimales ?
+ Quel est le type de donnée? Est-il le même ou a-t-il changé ?

Nous allons à présent chercher à séparer les pixels de l'image en deux
catégories, les pixels clairs (à priori appartenant aux cubes) et les pixels
foncés (à priori appartenant au fond). Pour cela, nous allons simplement
chercher une valeur _seuil_ (threshold en anglais) et séparer en deux les pixels
selon ce seuil. Il existe plusieurs manières de trouver le seuil optimal. Vous
pouvez le chercher à la main, ou utiliser des algorithmes automatisés pour le
trouver. La librairie `scikit-image` contient plusieurs algorithmes de
seuillage. 

🐍  Remplissez la fonction `compute_image_threshold` en essayant plusieurs stratégies de thresholding.
 
✍ Répondez aux questions suivantes:
+ Quelle stratégie avez vous essayé? 
+ Voyez vous une différence entre les différentes stratégies? Détaillez.
+ Quelle stratégie avez vous finalement choisie?

Lancez à nouveau le programme. Vous devriez voir les images binarisées se 
présenter. 

✍ Répondez aux questions suivantes :
+ Quel est maintenant le type des données de l'image ?
+ Observez ces différentes images. Les cubes sont ils entièrement classifiés comme tels ? Qu'en est il des écritures se trouvant sur les faces ?
+ La segmentation vous semble elle satisfaisante par rapport au problême ?

A la fin de l'algorithme chaque partie blanche contiguë est séparée pour créer
un masque censé couvrir un cube. 

✍ Observez attentivement les images issues de l'opération de seuillage. Voyez-vous des zones qui pourraient être séparées alors qu'elles ne couvrent pas un cube entier ?

Pour améliorer cette ébauche de segmentation, nous allons utiliser deux
opérations _morphologiques_ appelées _closing_ et _opening_.

✍ Répondez aux questions suivantes :
+ Quelle est la signification de ces opérations ? Détaillez.
+ En quoi pensez vous que ces opérations puissent améliorer notre segmentation ?
+ Quelle est l'utilité du paramètre de voisinage ? Comment pensez vous pouvoir le régler ?

Les librairies `scikit-image` et `scipy` contiennent toutes les deux des
implémentations de ces opérations. L'une est beaucoup plus rapide que l'autre.

🐍 Remplissez les fonctions `perform_closing` et `perform_opening`. Essayez les fonctions des deux librairies susmentionnées.

✍ Répondez aux questions suivantes :
+ Quelle implémentations vous semble la plus rapide entre les deux ?
+ Comment avez vous réglé le paramètre de voisinage? Quelle valeur avez vous choisi ? 

Exécutez le script a nouveau. Vous devriez voir les images binaires après
nettoyées apparaître. 

✍ Répondez aux questions suivantes :
+ Les défauts de l'image binaire ont ils disparu ?
+ La segmentation vous semble t elle satisfaisante ?

Suite à cela, l'algorithme utilise la fonction `ndimage.label` de `scipy` qui
sépare les différentes composantes contiguës d'une image binaire. Chaque
composante contiguë est ensuite nettoyé pour la phase suivante de l'algorithme
grâce a la fonction `clean_shape`. A la sortie de cette fonction, l'image doit
être parfaitement nettoyée, pour que la zone soit ne contienne aucune tache
sombre, et soit relativement lisse, comme sur cette image:

![img](img/../clean_shape.png)

🐍 Remplissez la fonction `clean_shape`. Quelles méthodes avez vous utilisé pour nettoyer la forme ? 

### 1.3 Recherche des contours

Pour continuer, nous devons transformer ces masques binaires en information
géométriques sur la position des coins du cube dans l'image. Pour faire cela,
nous allons utiliser un algorithme de détection de contours.

La fonction `get_box_contours` contient la logique d'extraction des coins.
L'algorithme passe par les étapes suivantes, pour chacun des masques:
+ Un contour initial, contenant de nombreux points est généré
+ Ce contour est simplifié pour ne garder que quatre éléments représentant les
  coins du cube.
  
La librairie `scikit-image` contient un algorithme permettant de chercher des
contours dans une image binaire.

🐍 Remplissez la fonction `extract_raw_contour`.

Le contour récupéré par la fonction `extract_raw_contour` contient un grand
nombre d'éléments. Cependant, pour la suite de notre algorithme, nous devons
pouvoir extraire les quatre coins de cet ensemble de points, pour ne garder
qu'eux.

🐍  Remplissez la fonction `simplify_raw_contour`.

✍ Répondez aux questions suivantes : 
+ Quelles stratégie avez vous adopté pour extraire les coins ? 
+ Relancez le script pour faire défiler quelques images dont les coins ont été détectés. Comment marche votre algorithme ?

### 1.4 Extraction des vignettes

Maintenant que nous avons la position des coins, nous allons extraire de l'image
originale, des vignettes de la même taille que les images du dataset MNIST. Cela
permettra, dans la suite du tp, d'envoyer directement les imagettes au réseau de 
neurones pour que le label soit reconnu. Pour cela, nous allons effectuer une
transformation projective de la zone délimitée par les coin, en une image plate 
de 28 par 28 pixels.

✍ Repondez à la question suivante :
+ Qu'est ce qu'une transformation projective ?

La bibliothèque contient une fonction permettant de calculer une transformation
projective à partir des positions des points de départ et d'arrivée, et de
transporter l'image d'un espace à l'autre. Cherchez dans la documentation cet
ensemble de fonctions.

🐍 Remplissez les fonctions `compute_transform` et `transform_image`.

Exécutez le script, vous devriez voir défiler les images extraites. 

🖼️ Enregistrez quelques exemples d'imagettes extraites par votre algorithme.

👀 Vous avez maintenant une vision complète sur l'algorithme d'extraction des
imagettes. Dans le cours de ce matin, nous avons expliqué que la pour créer un
tel programme, sans utiliser de données (ou trés peu), nous avons besoin de
faire des hypothèses sur les données.

✍  Repondez aux questions suivantes :
+ Sur quelles hypothèses est basé l'algorithme d'extraction de vignettes ? 
+ Dans les données d'essais dont nous disposons, arrive-t-il que ces hypothèses soient brisées ?
+ Pouvez vous penser à des situations dans laquelle notre robot pourrait se trouver, qui briseraient également ces hypothèses ?

Vous pouvez enregistrer votre fichier et le laisser tel quel pour le moment.

## 2. Challenge détection

Pour l'instant notre algorithme est capable d'extraire les imagettes lorsque les
cubes sont sur un fond sombre. Ce n'est probablement pas le genre de scène dans
laquelle nous voulons faire évoluer notre robot.

🐍 Toujours dans `detection.py` remplacez la ligne suivante :
```python
test_data = glob.glob('../data/ergo_cubes/dark/*.png')
```
par cette ligne:
```python
test_data = glob.glob('../data/ergo_cubes/challenge/*.png')
```

Exécutez le programme tel quel. Vous devriez voir s'afficher de nouvelles images
plus réalistes.

✍ Répondez aux questions suivantes :
+ Comment se débrouille l'algorithme sur ces nouvelles données ? 
+ Quelles hypothèses précédentes sont brisées par ces nouvelles données ?

Pendant ce challenge, vous allez développer un nouvel algorithme de segmentation
qui devrait avoir de meilleures performances sur ces images plus réalistes. À la
différence du précédent algorithme qui n'exploitait que les niveaux de gris pour
segmenter l'image, nous allons maintenant utiliser un algorithme qui utilise la
couleur. 

L'algorithme va suivre les étapes suivantes :
+ Tout d'abord, l'image est segmentée en utilisant les images rgb
+ Cette segmentation est nettoyée pour faire disparaître les artefacts de
  segmentation
+ Chaque objet de la segmentation est analysé pour déterminer si oui ou non il
  correspond à un cube.

### 2.1 Implémentation

🐍 Commencez par remplacer les lignes suivantes de la partie _main_:
```python
for im in images:
    im = segment_thresholding(im, debug=True) 
...
for i, im in enumerate(images):
    ctrs.append(get_box_contours(im, segment_thresholding, debug=True))
...
```

Par les lignes suivantes:
```python
for im in images:
    im = segment_colors(im, debug=True) 
...
for i, im in enumerate(images):
    ctrs.append(get_box_contours(im, segment_colors, debug=True))
...
```

Maintenant, à votre tour.

👀 Observez la fonction `segment_colors` et retrouvez la structure de l'algorithme présenté ci dessus. 

🐍 Implémentez les fonctions `build_segmented_image`, `clean_shape` et `is_cube`. 

✍ Dans un cours paragraphe, détaillez vos solutions, et les hypothèses qu'elle exploite sur les données.

🎞️ Une fois le programme aboutit, faite une video montrant votre algorithme en fonctionnement sur les anciennes, ainsi que les nouvelles images, que vous integrerez à votre rendu.

## 3. Partie apprentissage

Nous allons maintenant nous intéresser à la partie réseaux de neurones
permettant d'extraire les labels à partir des imagettes. Comme expliqué ce matin
dans la présentation, nous utiliserons le jeu de donnée _mnist_ pour entrainer
notre modèle. 

Commencez par ouvrir le fichier `learning.py`.

### 3.1 Chargement des données

Prenez connaissance du code, puis lancez le en exécutant à la ligne de commande :
```
python learning.py
```

✍ Observez la fonction `load_data`, répondez aux questions suivantes : 
+ Que contiennent les variables `x_train` et `y_train` ?
+ Pourquoi la fonction `load_data` renvoie-t-elle également les variables `x_test` et `y_test` ?
+ Quelles sont les formes respectives de `x_train` et `y_train` ? 
+ Pouvez vous expliquer à quoi correspondent chacune des dimensions de ces deux tableaux ?
+ Quelles sont les valeurs minimales et maximales de ces deux tableaux ? De quel type sont les données ? Expliquez.

### 3.2 Pré-visualisation des données brutes

👀 Appuyez sur entrée pour continuer, et observez les images. 

✍ Répondez aux questions suivantes :
+ Quelle sont les valeurs des pixels blancs (représentés en jaune) et des pixels noirs ? 
+ Observez bien les données et leurs labels. Toutes les images sont-elles simples à classifier correctement ? 
+ Ferriez vous des erreurs en les classifiant ?

### 3.3 Préparation des données

Au début de son entraînement, un réseau de neurones fonctionne bien lorsque les
données d'entrée sont centrées autour de 0 et ont un écart type de 1. Si ce
n'est pas le cas, les vecteurs de biais des différentes couches vont lentement
se modifier pour s'adapter aux données, mais cette étape est longue et gaspille
du temps de calcul inutilement. Il faut donc transformer nos données pour
qu'elles respectent ces pré-requis.

De plus, la première couche étant une couche de convolution, elle travaille
normalement avec des images ayant plusieurs canaux (RGB) encodés sur la dernière
dimension du tableau, ce qui n'est pas le cas de notre jeu de donnée. Il faut
donc modifier la forme de votre tableau pour la rendre compatible avec
l'opération de convolution.

🐍 Remplissez la fonction `prepare_input`.

Exécutez le script. Vous devriez voir s'afficher le informations sur le tableau
préparé.

✍ Répondez aux questions suivantes :
+ Quelle est la forme de `x_train` maintenant ?
+ Quelles sont les valeurs min et max ? Est ce que cela convient bien à notre pré-requis?
+ Le type de donnée devrait avoir changé? Quel est il? Sur quel type de donnée pensez vous qu'un réseau de neurone travaille?

Il reste à préparer les données de sortie. Pour l'instant, votre vecteur de
sortie `y_train` contient des entiers représentant le label de l'image.
Cependant pour fonctionner, notre réseau de neurones nécessite des données sous
la forme _one hot encoding_.

✍ Répondez aux questions suivantes :
+ Qu'est ce qu'un one-hot-encoding ? En quoi est ce différent des données dont nous disposons actuellement ?
+ Observez la fonction `load_data`. A quoi sert la variable globale `CLASSES` ?

🐍 Remplissez la fonction `prepare_output`.

✍ Répondez aux questions suivantes :
+ Quelle est la forme de `y_train` maintenant ?
+ Quelles sont les valeurs min et max ? Est ce que cela convient bien à notre pré-requis ?

### 3.4 Pré-visualisation des données préparées

👀 Exécutez le script et observez les données préparées.

✍  Répondez aux questions suivantes :
+ Quelles sont les valeurs des pixels blanc et des pixels noirs maintenant ?
+ Quelles sont les valeurs des labels maintenant ? Comprenez-vous leur signification sur cet exemple ?

### 3.5 Le modèle

Maintenant, vous allez devoir définir le modèle _LeNet5_ que nous avons vu
pendant le cours.

🐍 Remplissez la fonction `build_model`.

👀 Exécutez le script à nouveau et observez le résumé du modèle. 

✍ Répondez aux questions suivantes:
+ Observez le nombre de paramètres par couche. Quelles sont les couches qui ont le plus grand nombre de paramètres ?
+ Cela vous semble t il normal ?
+ Qu'en concluez vous sur l'utilité des couches par convolution ? 

### 3.6 La fonction de coût et l'optimiseur

Durant la présentation, nous avons vu que deux fonctions de coût sont
régulièrement utilisées dans l'entraînement des réseaux de neurones:
+ L'erreur quadratique moyenne (Mean squared error)
+ L'entropie croisée (Cross entropy)

✍ Repondez à la question suivante :
+ Quelle fonction de coût vous semble adaptée à notre problème ?

🐍 Remplissez la fonction `get_loss` pour implémenter la fonction de coût adaptée. 

Pendant la présentation, nous avons vu que l'optimiseur est l'algorithme qui
permet de se déplacer sur la surface dessinée par la fonction de coût dans
l'espace des paramètres. Cet algorithme permet de chercher l'endroit où la
fonction de coût est minimale. 

Un des algorithmes les plus simples s'appelle la __descente de gradient__ (GD)
et consiste à se déplacer dans le sens de la pente la plus forte à chaque pas de
temps.

✍ Répondez aux questions suivantes:
+ Dans quelle hypothèse cet algorithme permet il de trouver le minimum global de la fonction de coût selon vous ?
+ Pensez vous que cette hypothèse soit vérifiée pour les réseaux de neurones ?
+ Que se passe-t-il si cette hypothèse n'est pas vérifiée ? 

__Adam__ est un optimiseur plus complexe que GD. Sur l'image suivante, on voit
plusieurs optimiseurs se déplacer sur une fonction de coût:

![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie11.gif)

✍ Répondez à la question suivante :
+ Concentrez vous sur Adam et GD. Quelle semble être la caractéristique de Adam comparée a GD ? 

Une autre caractéristique de l'algorithme GD, est que la taille du pas qui est
effectué à chaque itération est fixe. L'image suivante montre Adam et GD dans un
cas ou la pente devient trés forte:

![cette image](https://github.com/Jaewan-Yun/optimizer-visualization/raw/master/figures/movie9.gif)

✍ Repondez aux questions suivantes: 
+ GD arrive-t-il à converger ? Comprenez vous pourquoi ?
+ Adam ne semble pas soumis au même problême que GD ? Quelle autre caractéristique de Adam cela montre-t-il ? 
+ Quelle conclusion pouvez vous tirer sur l'utilité de GD pour entraîner des réseaux de neurones ?

Maintenant à vous de jouer.

🐍 Remplissez la fonction `get_optimizer`.

### 3.7 Entrainement

Relancez le code et appuyez sur entrée jusqu'au déclenchement de la partie 3.7.
👀 Vous devriez voir les itérations d'entraînement se succéder.

✍ Répondez aux questions suivantes:
+ Observez l'évolution de la précision sur l'ensemble d'entraînement et l'ensemble de test. Les valeurs sont elles identiques ?
+ À partir de combien d'époques le réseau est il entraîné selon vous ?
+ En réglant le nombre d'itération d'apprentissage dans le code (argument `epochs` de la fonction fit), arrivez vous à observer une phase de sur-apprentissage ?

### 3.8 Poids appris

👀 Appuyez sur entrée pour visualiser les noyaux appris par le réseau de neurones. 

✍ Répondez aux questions suivantes:
+ En observant les noyaux de la première couche, arrivez vous à distinguer le genre de features qui seront extraites par chacun des noyaux?
+ Pouvez vous en faire de même pour la deuxième couche ?

### 3.9 Activations

👀 Appuyez sur entrée, puis entrez un indice (un entier de n'importe quelle valeur
inférieure a 5000).

✍ Répondez aux questions suivantes:
+ Après la première couche de convolution, les features extraites correspondent-elles à celles que vous imaginiez ?
+ Après la première couche de pooling, les features présentes auparavant sont-elles conservées ?
+ Après la deuxième couche de pooling, diriez vous que de l'information spatiale est toujours présente ? Autrement dit, les activations ressemblent-elles toujours à des images ?

### 3.10 Entrainement final

❌ Arrêtez le script en appuyant sur `ctrl+c`. Jusqu'à présent, nous avons
travaillé sur l'ensemble des données, mais pour la suite nous n'aurons besoin
que des images de 1 et de 2. Changez la valeur de la variable `CLASSES` pour ne
garder que les classes qui nous intéressent, entraînez en réseau, puis
sauvegardez le dans un fichier.

## 4. Intégration

↔️ Il est maintenant temps d'intégrer les deux parties du pipeline pour
l'utilisation finale. Ouvrez le fichier `main.py` à la racine du projet. Pour
que les deux parties du pipeline s'adaptent correctement, il faut que les
vignettes renvoyées par la partie détection ressemblent aux images du dataset
mnist (dans leur valeurs, leur type, etc).

🐍 Remplissez la fonction `preprocess`.

👀 Une fois que cela est fait, exécutez le script. Vous devriez pouvoir examiner la
performance de votre algorithme complet sur les images de test.

✍ Répondez aux questions suivantes :
+ Quelles sont les performances de l'algorithme global ?
+ Observez-vous des sources de bugs dans l'algorithme ?

🤖 Enfin, en vous connectant à Poppy Ergo Jr, récupérez quelque images des cubes dans
votre propre environnement, puis déposez les dans `data/ergo_cubes/perso`.
Lancez le programme et observez la performance de votre algorithme sur les
données de votre propre environnement.

✍ Répondez aux questions suivantes :
+ Quelles sont les performances de l'algorithme sur vos données ? 
+ Sont elles comparables aux performances sur les exemples dont nous disposons ? 

ℹ️ Nous avons vu que le sur-apprentissage est une des difficultés les plus
importantes dans l'utilisation des algorithmes d'apprentissage. Lorsque vous
avez réglé votre programme de détection, vous l'avez fait en utilisant
l'ensemble des données dont vous disposiez, sans garder de groupe test séparé.
Cette manière de faire peut également mener à du sur-apprentissage, mais cette
fois, de votre part, dans le réglage de votre algorithme. Le sur-apprentissage
n'est donc pas réservé aux algorithmes mais est plutôt une des difficultés de la
programmation basée sur les données. Que ce soit un humain ou un programme, le
réglage de paramètres en fonction de données connaît les mêmes défauts.

ℹ️ De manière générale, il est futile d'attendre de ce type de programme qu'il
extrapole à des données (des situations) qui n'ont pas été présentées pendant
l'apprentissage. Il est donc important, lors du développement de ce type de
programme, de prévoir une phase de récolte de donnée importante, dont on
s'assure qu'elle couvrira bien les usages futures du programme.
