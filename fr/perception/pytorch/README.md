# IV. Perception avec Torch (PyTorch)

Torch est une bilbiothèque opensource d'apprentissage machine et en particulier d'apprentissage profond. Depuis 2018 seule sa version Python nommée PyTorch est maintenue. Nous allons l'utiliser ici sur des imagettes sur lesquelles sont inscrites des chiffres marqués manuellement au feutre avec différentes calligraphies. Le réseau de neurones que vous allez créer devra apprendre lui-même à déterminer quel chiffre est marqué, ce que l'on appelle **classifier**. 

## 0. Installation

Pour effectuer cet atelier, vous devez installer quelques packages :
```
pip install torch==1.3.1+cpu torchvision==0.4.2+cpu -f https://download.pytorch.org/whl/torch_stable.html
pip install scikit-image imageio
git clone https://gitlab.inria.fr/apere/ros_workshop.git && git checkout e78f295b6
```

## 1. Documentation

Pour remplir ce code, vous aurez besoin de consulter plusieurs documentations. De manière générale, vous aurez besoin de numpy. Si vous n'êtes pas familier avec, vous pouvez consulter :
+ [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
+ [Numpy Documentation](https://numpy.org/devdocs/user/quickstart.html)
Pour la partie détection et pré-processing, nous utiliserons `scikit-image`. Vous pouvez vous inspirer des exemples:
+ [Scikit-Image General examples](https://scikit-image.org/docs/stable/auto_examples/)
Enfin, pour la partie reconnaissance, nous utiliserons `pytorch`. Vous devriez trouver ce qu'il vous faut dans le tutoriel suivant:
+ [Deep learning with pytorch: a 60 minute blitz](https://pytorch.org/tutorials/beginner/deep_learning_60min_blitz.html)

## 2. Partie reconnaissance (Matin)

### 2.1 Les données et le modèle

Commencez par ouvrir le fichier `src/models.py`. Pour vous familiariser avec les tenseurs, commencez par remplir les fonctions `imag_mean` et `image_std`. Quand cela est fait executez le script une première fois avec :
```
python src/models.py
```

Quelle est la taille d'un batch de données d'entrée ? De données de sortie ? 

Quelle est la moyenne du batch d'entrée ? Son écart type ? 

Dans le fichier se trouve une classe `LeNet` vide, qui contiendra votre modèle. En vous référant à la documentation de pytorch, remplissez la classe pour définir l'architecture LeNet vue pendant la présentation.

Attention, la méthode `forward` prend en entrée un tenseur de taille [128, 1, 28, 28] et doit retourner un tenseur de taille [128, 2]. En revanche, la méthode infer doit retourner le label (`1` ou `2`) trouvé par le réseau de neurones pour une image, soit un simple float.

Une fois le réseau défini, ré-exécutez le script. Si votre réseau est bien défini, il devrait être utilisé pour faire une inférence. 

Les labels trouvés par le réseau de neurones font ils sens ? 

Remplissez la fonction `compute_params_count`. Exécutez à nouveau; quel est le nombre de paramêtres du réseau de neurones ? Qu'en pensez vous ? 

Enfin, si tout le script s'exécute, vous devriez voir s'afficher les noyaux de la première couche. Qu'en pensez vous ? 

### 2.2 L'entrainement.

Dirigez vous vers le fichier `train.py`. La fonction `perform_train_epoch` contient la procédure d'entrainement. Remplissez-la en suivant la procédure présentée durant la matinée, puis exécutez le script. 

Si votre algorithme d'entrainement est correctement écrit, vous devriez voir les courbes d'apprentissage apparaitre. Que pensez vous de ces courbes ? Que pensez vous du niveau de performance à la fin de l'entrainement? 

A la suite de l'entrainement, le script vous présente les noyaux de la couche d'entrée du réseau. Qu'en pensez vous ? Ont ils évolué par rapport à tout à l'heure ? 

Enfin, pouvez vous entrainer le modêle pendant suffisament longtemps pour voir la phase d'overfitting apparaître ? 

## 3. Partie amont (Après-midi)

### 3.1 Détection

Exécutez le fichier `src/detection.py`.  Les données sont chargées depuis le disque dans un tableau numpy, dont un example vous est montré. Remplissez la fonction `rescale` qui permet de ré-échelloner les valeurs du tableau entre 0 et 1.

La première étape du pipeline de détection consiste à binariser l'image. Remplissez la fonction `binarize` et ré-exécutez le code. Observez chacun des exemples. Les chiffres ont ils bien disparus ? Les cubes sont ils couverts chacun par une unique zone blanche? Cette zone blanche est elle proche de la surface du cube ? 

Tant que vous ne pouvez répondre positivement à toutes ces questions (dans la mesure du raisonnable), ameliorez votre programme! Une fois fini, vous pouvez mettre `binarize(im, debug=False)` pour éviter de revoir toutes les images à chaque fois.

Suite à cela, nous allons effectuer la recherche des cubes dans l'image. Remplissez la fonction `get_box contour` et ré-exécutez le code. À nouveau, les cubes sont ils bien détectés ? Tant que votre algorithme ne positionne pas correctement les coins des différentes boites de l'image, améliorez-le.

Encore une fois quand vous aurez terminé, désactivez la visualisation avec `get_box_contour(im, debug=False)`.

### 3.2 Pré-processing

Nous avons maintenant une fonction `get_box_contours` qui nous donne, à partir d'une image d'entrée, une liste de contours de quadrilatères. Maintenant, il s'ait de transformer ces quaqdrilatères en images de la taille des images de mnist. Pour cela, nous allons calculer une transformation projective pour chacun des cubes. 

Remplissez la fonction `get_sprites`, puis ré-executez le code. Si votre code est correct, vous devriez voir s'afficher des imagettes ressemblant (au moin par la taille) aux images de mnist.

Une fois cela fait, il ne vous reste plus qu'à remplir la fonction `preprocess_sprites`. Le but de cette fonction est de faire en sorte que les images données au réseau de neurones ressemblent le plus possible aux images de l'entrainement. Étudiez bien les données d'entrée avant de vous mettre au travail (vous aurez peut être besoin de retourner regarder `src/models.py` ou de regarder `src/misc.py`).


## 4. Final

Une fois terminé, vous pouvez vous rendre dans `main.py` pour tester l'intégration de tout votre travail. Bravo !