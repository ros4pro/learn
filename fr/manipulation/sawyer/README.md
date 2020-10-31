# III. Robotique de manipulation avec Sawyer

La robotique de manipulation regroupe la manipulation d'objets avec des robots : des bras articulés à 5 ou 6 axes, les robots [SCARA](https://en.wikipedia.org/wiki/SCARA) (Selective Compliance Assembly Robot Arm), les robots [cartésiens](https://en.wikipedia.org/wiki/Cartesian_coordinate_robot) (linéaires), les robots [parallèles](https://en.wikipedia.org/wiki/Parallel_manipulator) ... Dans ce TP nous utilisons un robot Sawyer du fabriquant Rethink Robotics.

## Prérequis

* BAC+2 et +
* Aisance en Python et commandes dans un terminal
* Aisance en géométrie 3D
* Le [TP d'introduction](../introduction)

## Diapositives

{% pdf src="https://files.ros4.pro/manipulation.pdf", width="100%", height="565px" %}{% endpdf %}

## 1. Documentation
### 1.1. Les liens
* [Tutoriaux de MoveIt](https://ros-planning.github.io/moveit_tutorials/)
* [Code du MoveIt Commander Python](https://github.com/ros-planning/moveit/tree/master/moveit_commander/src/moveit_commander)
* [Documentation de l’API MoveIt en Python](http://docs.ros.org/melodic/api/moveit_python/html/namespacemoveit__python.html)
* [Tutoriaux du SDK Sawyer](https://sdk.rethinkrobotics.com/intera/Tutorials)

### 1.2. Le plus important
Voici les quelques fonctions les plus importantes traduites depuis la documentation.

#### 1.2.1. Déclarer un commandeur de robot 
```
commander = MoveGroupCommander("right_arm")
```
Ce commandeur est celui qui identifie notre robot et permettra d'en prendre le contrôle.

#### 1.2.2. Exécuter un mouvement vers une cible
Définir une cible, dans l'espace des joints :
```
commander.set_joint_value_target([-1.0, -2.0, 2.8, 1.5, 0.1, -0.3, 3.0])
```
Les 7 valeurs sont les angles cibles des 7 joints en radians
Attention : les cibles dans l'espace des joints n'auront pas d'évitement de collisions

Ou bien définir une cible dans l'espace cartésien :
```
commander.set_pose_target([0.5, 0.05, 1.1, 0, 0, 0, 1])
```
Les 7 valeurs sont la **position** et l'*orientation* [**x, y, z**, *qx, qy, qz, qw*] cible de l'effecteur dans le repère `base`

#### 1.2.3. Planifier & exécuter le mouvement vers la cible
La fonction "go" déclenche le calcul de trajectoire et l'exécution instantanée si le calcul a réussi :
```
commander.go()
```

### 1.2.4. Exécuter une trajectoire cartésienne
Par opposition à la cible cartésienne, dans cet exemple au lieu de ne définir qu'une cible finale, on demande à MoveIt de suivre une trajectoire rectiligne dans l'espace cartésien. Cette trajectoire est précalculée en entier grâce à la fonction `commander.compute_cartesian_path([pose1, pose2]), resolution, jump)`
où :
* `[pose1, pose2]` est la liste des points à suivre de manière rectiligne de type `geometry_msgs.Pose`
* `resolution` est la distance en mètre entre 2 points cartésiens générés (par exemple `0.01` pour générer un point tous les centimètres)
* `jump` est le seuil maximal autorisé au delà duquel un saut trop important entre 2 positions angulaires en radians ne sera pas exécutée car il demanderait une vitesse excessive. `jump` est la somme des seuils sur tous les joints (par exemple `3.14`).

La fonction `commander.compute_cartesian_path(...)` renvoie :
* `trajectory`: la trajectoire cartésienne calculée
* `ratio`: Un ratio entre 0 et 1 indique la quantité de la trajectoire qui a pu être calculée avec succès sans générer de saut. Un ratio inférieur à 0.95 signifie probablement que la trajectoire est inutilisble car elle ne suit pas le chemin demandé. 

Par exemple, étant données 2 points `p1` et `p2` de type `geometry_msgs.Pose`, cet appel est valide :
```
trajectory, ratio = commander.compute_cartesian_path([pose1, pose2]), 0.01, 3.14)
```

Enfin, exécuter la trajectoire, uniquement si le ratio indique au moins 95% de succès :
```
commander.execute(trajectory)
```

#### 1.2.5. Définir des objets de collision
Lorsqu'on ajoute des objets de collision, les appels à `go()` dans l'espace cartésien planifieront, si possible, une trajectoire en évitant les objets déclarés comme objets de collision.

La scène de planification est notre interface pour ajouter ou supprimer des objets de collision :
```
scene = PlanningSceneInterface()
```
On peut ensuite effectuer les ajouts ou suppressions. Par exemple, on ajoute un objet de collision cubique de taille 10x8x2 cm à la position [1.2, 0.5, 0.55] et avec l'orientation [0, 0, 0, 1] (= rotation identité) dans le repère `base`:  

```
ps = PoseStamped()
ps.header.frame_id = "base"
ps.pose.position.x = 1.2
ps.pose.position.y = 0.5
ps.pose.position.z = 0.55
ps.pose.orientation.x = 0
ps.pose.orientation.y = 0
ps.pose.orientation.z = 0
ps.pose.orientation.w = 1
scene.add_box("ma_boite", list_to_pose_stamped2([[1.2, 0.5, 0.55], [0, 0, 0, 1]]), (0.10, 0.08, 0.02))
```
Les objets de collision apparaissent en vert dans RViz s'ils sont définis correctement.

Note: après une modification de la scène, est généralement utile de faire une pause `rospy.sleep(1)` afin de laisser le temps à la scène d'être mise à jour sur tout le système avant toute nouvelle planification de trajectoire

## 2. Travaux pratiques
### 2.1. Utiliser MoveIt dans le visualisateur Rviz
Avec roslaunch, lancer `sawyer_moveit.launch` provenant du package `sawyer_moveit_config`:
```
roslaunch sawyer_moveit_config sawyer_moveit.launch
```

Via l’interface graphique, changer l’orientation et la position de l’effecteur puis demander à MoveIt de planifier et exécuter une trajectoire pour l’atteindre.

* Cochez la bonne réponse : Cette méthode permet-elle de définir une cible dans l’espace : ◻ cartésien ◻ des joints

* Trois robots semblent superposés en 3D, quelles sont leurs différences :
  * Le robot orange est ... ?
  * Le robot rapide est ... ?
  * Le robot lent est ... ?

* Utilisez `rostopic echo` pour afficher en temps réel les messages du topic `/robot/joint_states`. Exécutez un mouvement et observer les valeurs changer. Que représente le topic /robot/joint_states ? 

* Indiquez comment se nomment les 7 joints de Sawyer depuis la base jusqu’à l’effecteur :
  * Premier joint :
  * Deuxième joint :
  * Troisième joint :
  * Quatrième joint :
  * Cinquième joint :
  * Sixième joint :
  * Dernier joint :

### 2.2. Utiliser MoveIt via son client Python
Dans le package `ros4pro`, ouvrir le nœud `manipulate.py`. Repérez les 3 exemples :
* d'exécution d'une trajectoire cartésienne
* de planification vers une cible cartésienne
* de planification vers une cible dans l'espace des joints

Durant la suite du TP, nous démarrerons notre programme de manipulation avec le launchfile `manipulate.launch` du package `ros4pro` qui fonctionne par défaut en mode simulé, à savoir :

```
roslaunch ros4pro manipulate.launch
```
Celui-ci démarre automatiquement `manipulate.py`, il est donc inutile de le démarrer par un autre moyen.

#### 2.2.1. Modifier la scène de planification
La scène représente tout ce qui rentre en compte dans les mouvements du robot et qui n’est pas le robot lui-même : les obstacles et/ou les objets à attraper. Ces éléments sont déclarés à MoveIt comme des objets de collision (détachés du robot ou attachés c’est-à-dire qu’ils bougent avec lui).

* Prenez les mesures du feeder puis déclarez-les comme objets de collision dans votre noeud Python via l’interface `PlanningSceneInterface`. Complétez le TODO associé à la question 3.2.1. dans `manipulate.py`.

* Planifiez et exécutez un mouvement RViz pour vérifier que les collisions sont évitées. Déclarez un nouvel obstacle qui entrave forcément le chemin du robot et vérifiez. Vérifiez ce qu’il se passe lorsque le planner ne trouve aucune solution.

#### 2.2.2. Effectuer un pick-and-place avec un cube simulé (optionnel)
Nous considérons un cube situé à la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ` `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ = [[0.32, 0.52, 0.32], [1, 0, 0, 0]]`, ce qui correspond exactement à l'emplacement entouré en vert sur le feeder.

Pour l’approche, on positionnera le gripper 18cm au dessus du cube le long de son axe z.

* Sachant cela, déduire la matrice de transformation `ᶜᵘᵇᵉP₉ᵣᵢₚₚₑᵣ` en notation `[[x, y, z], [x, y, z, w]]` ?
* Exprimez `ᵇᵃˢᵉP₉ᵣᵢₚₚₑᵣ` en fonction de la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ` du cube dans le repère `base` via une multiplication matricielle

Ensuite, dans votre code :

* Inventez un cube en simulation dans votre code à la pose `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ = [[0.32, 0.52, 0.32], [1, 0, 0, 0]]` c’est-à-dire à la surface du feeder. Pour ce faire utilisez :
  * l’interface `PlanningSceneInterface` pour ajouter, supprimer un cube ou l’attacher à l’effecteur `right_gripper_tip` du robot
  * `TransformBroadcaster` pour publier la frame tf nommée `cube` au centre du cube à attraper

#### 2.2.3. Générer les 4 trajectoires du pick-and-place
Pour rappel, voici les 4 étapes d'un pick pour attraper et relacher le cube simulé :
    1. trajectoire d’approche : aller sans collision à `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ` c'est à dire 18cm au dessus du cube sur son axe z (axe bleu)
    2. trajectoire de descente : suivre une trajectoire cartésienne de 50 points descendant le long de l'axe z pour atteindre  `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ`  avec l’effecteur `right_gripper_tip`. Puis fermer l'effecteur.
    3. trajectoire de retraite : retourner au point d’approche par une trajectoire cartésienne
    4. trajectoire de dépose : si le cube a bel-et-bien été attrapé avec succès, aller sans collision au point de dépose `ᵇᵃˢᵉP꜀ₔₑₚₒₛₑ = [[0.5, 0, 0.1], [0.707, 0.707, 0, 0]]`

* Dans `manipulate.py`, les deux dernières trajectoires sont incomplètes : retraite et dépose nommées `release` et `place`. Compléter les 2 TODO associés à la question 3.2.3 dans `manipulate.py`

* Vérifier que votre pick-and-place a l'air correct **en simulation d'abord** et que vous distinguez correctement **les 4 étapes du pick-and-place**.

### 2.3. Exécutez le pick-and-place sur le Sawyer réel
* Changez votre rosmaster pour celui de Sawyer :
```
export ROS_MASTER_URI=http://021608CP00013.local:11311
```
* Le centre du cube `ᵇᵃˢᵉP꜀ᵤ₆ₑ_ᵥₑᵣₜ` précédent est celui de la zone verte sur le feeder. Vérifiez que votre pick-and-place fonctionne avec 1 seul cube à l’emplacement vert.

* Pour commander le robot réel modifiez le paramètre `simulate` :
```
roslaunch ros4pro manipulate.launch simulate:=false
```

Vous devriez constater que votre pick-and-place fonctionne parfois et qu'il échoue dans certaines situations.

### 2.4. Préparer le pipeline du scenario final (optionnel)
Ajoutez à `manipulate.py` le code nécessaire pour votre scenario final :
* **Prise de photo (scan)** :
  * positionner l’effecteur de telle manière que  « right_hand_camera » se trouve à la verticale du feeder
  * Récupérer l’image rectifiée sur le topic dédié :
    * image_view peut aider à visualiser l’image pour déboguer dans un terminal
    * pour récupérer l’image en Python, implémentez un Subscriber. Cette photo sera envoyée au réseau de neurones lorsqu’il sera implémenté
* **Pick-and-Place successifs** : Effectuez 3 pick-and-place successifs sans intervention humaine  de 3 cubes dont la position est connue à l’avance. Ces positions seront retournées par le réseau de neurones lorsqu’il sera implémenté
* Si vous n’avez pas de collision de manière reproductible, vous pouvez accélérer les vitesses dans sawyer_moveit_config/config/joint_limits.yaml (pas les accélérations)
        
### 2.5. Prévenir les échecs de planification
Le path-planning étant réalisé pendant l’exécution et non précalculé, il est possible que MoveIt ne trouve aucune solution de mouvement. Pour remédier à cela, plusieurs pistes s’offrent à nous :
* Réessayer encore et encore … bêtement, jusqu’à un potentiel succès
* Planifier toute les trajectoires d’un coup avant exécution, si l’une ne peut être calculée, regénérer la précédente et recommencer. Cela montre ses limites : et si le robot n’est pas à l’emplacement attendu entre deux trajectoires, par exemple si l’opérateur l’a bougé manuellement ?
* **Fournir une « seed » à l’IK ou au path-planning**. Cette solution est approfondie ci-après :

MoveIt possède un système de calcul de la **géométrie directe et inverse**, respectivement via les services `/compute_fk` et `/compute_ik`.
Observer les types de service (`rosservice info`) et le contenu de la requête (`rossrv show`) puis appeler ces deux services sur ces deux exemples :
* Calculer la FK pour les angles `-π/2, π/2, -π/2, 0, 0, 0, 0`. Donner le résultat au format `[[x, y, z,], [qx, qy, qz, qw]]`
* Calculer l’IK pour la position d’effecteur `[[0.5, 0, 0.1], [0.707, 0.707, 0, 0]]`. Donner le résultat au format `[angle1, angle2, angle3, angle4, angle5, angle6, angle7]`

Réexécutez le même calcul d’IK sur la même position d’effecteur, une seconde puis une troisième fois. Observez que le résultat est différent. Pourquoi ?

* Fournissez une « seed » au format `[angle1, angle2, angle3, angle4, angle5, angle6, angle7]` de votre choix à l’IK pour influencer le résultat.
* Quelle seed proposez-vous pour maximiser les chances de succès du path-planning ?
