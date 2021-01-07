# III. Robotique de manipulation avec Poppy Ergo Jr

La robotique de manipulation regroupe la manipulation d'objets avec des robots. Dans ce TP nous utilisons un robot opensource [Poppy Ergo Jr](https://www.poppy-project.org/fr/robots/poppy-ergo-jr/) qui peut être 100% imprimé en 3D à la maison ou à l'école. 

## Prérequis

* Lycée et +
* Notions de commandes dans un terminal et d'adressage IP
* Notions de Python
* Notions de géométrie 3D
* Le [TP d'introduction](../introduction)
* Ce TP est compatible avec la simulation si vous n'avez pas de Poppy Ergo Jr : sauter directement au 2.3.bis

## Diapositives

{% pdf src="https://files.ros4.pro/manipulation.pdf", width="100%", height="565px" %}{% endpdf %}

## 1. Préparer le matériel (avec un robot réel)
### 1.1. Préparer la carte SD
📥 Pour éviter tout problème lié à une précédente utilisation du robot, commencez par flasher la carte SD fournie avec l'image ROS en utilisant [la procédure vue lors de l'introduction](https://github.com/ros4pro/ros4pro/blob/poppy_tb3_keras/tp/1_INTRODUCTION.md#1-images-des-cartes-sd). Pendant cette étape, assemblez votre robot en parrallèle. 

### 1.2. Assembler Poppy Ergo Jr
🔧 Pour assembler votre robot, veuillez suivre [le guide d'assemblage](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/), en suivant les étapes faîtes pour ROS le cas échéant ; et en comparant minutieusement chaque pièce aux photos pour vérifier leur orientation car il est très facile d'assembler ce robot à l'envers même s'il a au final la même allure. Si votre robot est pré-assemblé, recommencez à minima toutes les configurations des moteurs qui pourraient être incorrectes.

✅ **Vérification :** Pour vérifier que votre assemblage est correct, connectez-vous en SSH au robot (si ce n'est pas déjà fait) puis exécutez :
```bash
ssh pi@poppy.local      # password raspberry
# Effacer éventuellement l'ancienne clé ECDSA si vous avez un message d'erreur
roslaunch poppy_controllers control.launch
```
Vous devriez voir apparaître `Connection successful`. Si l'erreur `"Connection to the robot can't be established"` est affichée, alors votre robot n'a pas été monté correctement. La suite de ce message d'erreur indique quel(s) moteur(s) pose(nt) problème pour vous aider à le résoudre. Fermez avec Ctrl+C puis utilisez de nouveau Poppy Configure si un moteur est mal configuré.

**Remarque :** Si vos moteurs clignotent en rouge : votre code a créé une collision et ils se sont mis en alarme. Pour désactiver l'alarme il faut débrancher et rebrancher l'alimentation, ce qui fera aussi redémarrer le robot

## 2. Travaux pratiques
### 2.1. Comprendre la représentation d'un robot ROS

Un robot intégré à ROS est composé d'au minimum :
* un descripteur URDF
* un contrôleur qui gère les E/S avec le robot

#### 2.1.1. Comprendre le descripteur URDF

💻📀 Clonez le package ROS Poppy Ergo Jr Description sur votre PC, il contient le fichier de description URDF du robot :
```bash
git clone https://github.com/poppy-project/poppy_ergo_jr_description.git
```

💻 Compilez votre workspace puis sourcez votre `.bashrc`, enfin rdv dans le dossier `urdf` de ce package, puis exécutez la commande `urdf_to_graphiz` qui convertit un fichier URDF en représentation graphique dans un PDF :
```bash
sudo apt install liburdfdom-tools
roscd poppy_ergo_jr_description/urdf
urdf_to_graphiz poppy_ergo_jr.urdf
```

Ouvrez le PDF obtenu puis déterminez :
* Que représentent les rectangles ?
* Que représentent les bulles ?
* Que représentent les flèches et surtout les valeurs `xyz` et `rpy` associées ?

![URDF de Poppy Ergo Jr](https://raw.githubusercontent.com/poppy-project/poppy_ergo_jr_description/master/doc/img/rviz.png)

#### 2.1.2. Comprendre les E/S du contrôleur

🤖 Le contrôleur se trouve déjà sur le robot. Vous pouvez directement vous connecter au robot et le démarrer :

```bash
ssh pi@poppy.local      # password raspberrypi
# Effacer éventuellement l'ancienne clé ECDSA si vous avez un message d'erreur
roslaunch poppy_controllers control.launch
```

💻 Sur votre PC, faîtes pointer votre `ROS_MASTER_URI` sur `poppy.local`. Rappel :
```bash
nano ~/.bashrc      # Pour changer votre ROS_MASTER_URI
source ~/.bashrc    # Pour charger votre .bashrc et donc le nouveau master
```

##### 2.1.2.a. Topics du robot
✍ Avec l'utilitaire `rostopic`, lister les topics disponibles puis consultez celui qui décrit l'état courant des joints, en particulier :

* Quel est son nom ?
* Quel est le type de message qu'il transmet ?
* A quelle fréquence (en Hertz) est-ce qu'il met à jour l'état des joints ?

##### 2.1.2.b. Services du robot
✍ Avec les utilitaires `rosservice` et `rossrv`, listez les services disponibles puis consultez celui qui met le robot en mode **compliant**. En particulier :

* Quel est le nom de topic du service mettant le robot en compliant ?
* Quel est le type de ce service ?
* Consultez le détail des champs. Quels sont les champs de la **requête** de ce service ?
* Consultez le détail des champs. Quels sont les champs de la **réponse** de ce service ?
* Appelez ce service pour activer et désactiver le mode compliant et essayez de faire bouger votre robot à la main à chaque fois. Que déduisez-vous de la signification du **mode compliant** ? *Conseil :* aidez-vous de l'autocomplétion avec la touche <TAB>

##### 2.1.2.c. Tracer la courbe des positions des moteurs en temps réel
Mettez votre robot en mode compliant, démarrez `rqt_plot` pour tracer les positions des 6 moteurs ... bougez les moteurs à la main et vérifiez que `rqt_plot` actualise la courbe en temps réel.

### 2.2. Cinématique, et planification avec MoveIt dans RViz
#### 2.2.1. Démarrer avec MoveIt
💻📀 Installez MoveIt puis clonez le package ROS **Poppy Ergo Jr MoveIt Configuration**, il contient le code nécessaire pour que ce robot fonctionne avec MoveIt :
```bash
sudo apt install ros-noetic-moveit
git clone https://github.com/poppy-project/poppy_ergo_jr_moveit_config.git
```

💻 Compilez votre workspace puis sourcez votre `.bashrc`. Démarrez MoveIt avec `roslaunch` avec le paramètre `fake_execution` à false pour se connecter au vrai robot :
```
roslaunch poppy_ergo_jr_moveit_config demo.launch fake_execution:=false gripper:=true
```
Rviz doit démarrer avec un Poppy Ergo Jr en visu.

Note : si vous devez passer en simulation à ce moment suite à un défaut matériel, pensez à changer votre `ROS_MASTER_URI` pour `localhost` puis mettre simplement `fake_execution` à `true`.

![MoveIt avec Poppy Ergo Jr](https://raw.githubusercontent.com/poppy-project/poppy_ergo_jr_moveit_config/36ffb295cf115a080b81aa6475ae512e88c9957a/doc/img/MoveIt.gif)

#### 2.2.2. Planification

💻 Dans l'onglet Planning, section **Query** puis **Planning group**, sélectionnez le groupe `arm_and_finger`, bougez le goal (la sphère 3D bleue) en position et en orientation puis cliquez sur **Plan**.

✍ Trois représentations 3D de robots se superposent, déterminez le rôle de chacun d'entre eux en testant également la fonctionnalité **Plan and Execute** :

* Que désigne le robot gris parfois mobile mais lent ?
* Que désigne le robot orange (fixe) ?
* Que désigne le robot gris qui répète infiniment un mouvement rapide ?
* Dans RViz, activer l'affichage du modèle de collision dans `Displays`, `Scene Robot`, `Show Robot Collision`, quelle est la forme de ce modèle utilisé par OMPL pour simplifier le calcul des collisions ?

#### 2.2.3. Planning groups
💻✍ Testez également le groupe `arm` en plus du premier `arm_and_finger` et lancez des planifications de mouvement pour tester :

* Quelle est la différence entre ces 2 groupes ?
* Quel est le groupe pour lequel le goal est le plus facilement manipulable ?
* Pourquoi ce groupe est-il plus facilement manipulable que l'autre ?
* Déduisez-en ce que désigne exactement un `planning group`

#### 2.2.4. Transformations `tf`
Nous allons visualiser et interroger l'arbre des transformations nommé `tf`

💻✍ Démarrer MoveIt puis dans un autre terminal lancer `rosrun tf2_tools view_frames.py`. Un fichier PDF nommé `frames.pdf` a été créé : les `frames` (repères géométriques) qu'ils contient sont les mêmes que ceux dessinés par Rviz en rouge-vert-bleu.

* Comment est nommé le repère de base ?
* Comment sont nommés les deux effecteurs finaux possibles ?
* La commande `rosrun tf2_tools echo.py frameA frameB` renvoie la transformation actuelle de frameB dans frameA. Modifiez cette commande pour déterminer quelle est la position actuelle d'un des effecteurs dans le repère de base. Ses coordonnées peuvent vous servir par la suite, pour les définir comme cible à atteindre.

### 2.3. Ecrire un noeud Python ROS pour l'Ergo Jr
#### 2.3.1. Créer un nouveau package et un nouveau noeud Python
💻 Nous allons créer un nouveau package ROS nommé **ros4pro_custom** sur votre laptop de développement, qui contient notre code:
```bash
cd ~/catkin_ws/src
catkin_create_pkg ros4pro_custom             # Cette commande créé le package
mkdir -p ros4pro_custom/src                  # On créé un dossier src dans le package
touch ros4pro_custom/src/manipulate.py       # On créé un noeud Python "manipulate.py"
chmod +x ros4pro_custom/src/manipulate.py    # On rend ce noeud exécutable pour pouvoir le lancer avec rosrun
```

💻🐍 Bien que vous devriez avoir compris comment créer un noeud ROS en Python dans les tutoriels d'introduction, voici un rappel de noeud ROS minimal qui boucle toutes les secondes en Python :
```python
#!/usr/bin/env python

import rospy

rospy.init_node('ros4pro_custom_node')
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rospy.loginfo("Hello world from our new node!")
    rate.sleep()
```


💻 Compilez votre workspace puis sourcez votre `.bashrc`. Exécutez votre noeud avec rosrun :
```bash
cd ~/ros_ws
catkin_make
rosrun ros4pro_custom manipulate.py
```

Votre noeud doit afficher un message toutes les secondes, vous pouvez le tuer avec Ctrl+C. Nous allons ajouter du code petit à petit. Attention à l'ajouter au bon endroit pour créer un script cohérent.

#### 2.3.2. Planifier et exécuter des mouvements avec MoveIt

Le `MoveGroupCommander` est le commandeur de robot de MoveIt, il suffit de lui indiquer quel est le nom du groupe à commander puis donner une cible et appeler la fonction `go()` pour l'atteindre en évitant les obstacles. Cette cible peut être dans l'espace cartésien ou dans l'espace des joints :

##### 2.3.2.a. 🐍 Cible dans l'espace cartésien

```python
from moveit_commander.move_group import MoveGroupCommander
commander = MoveGroupCommander("arm_and_finger")
commander.set_pose_target([0.00, 0.079, 0.220] + [0.871, -0.014, 0.079, 0.484])
commander.go()
```

Les coordonnées cartésiennes de la cible sont les coordonnées de l'effecteur (càd `moving_tip` pour le groupe `arm_and_finger` ou bien `fixed_tip` pour le groupe `arm`) dans le repère `base_link`, exprimées sous la forme `x, y, z, qx, qy, qz, qw`.

##### 2.3.2.b. 🐍 Cible dans l'espace des joints (sans évitement de collision)

Il est également possible de définir une cible dans l'espace des joints en fournissant une liste des 6 angles moteurs  dans ce cas il n'y a pas d'évitement de collision:

```python
commander.set_joint_value_target([0, 0, 0, 0, 0, 0])
commander.go()
```

##### 2.3.2.c. ✍ Mise en pratique

* A l'aide des fonctions et commandes vues en 2.2.4. et 2.3.2.a., vérifiez que vous savez prendre les coordonnées cartésiennes courante et les définir comme cible puis l'atteindre
* A l'aide des fonctions et commandes vues en 2.1.2.a. et 2.3.2.b., vérifiez que vous savez prendre les positions des joints courantes et les définir comme cible puis l'atteindre
* A l'aide du mode compliant, prendre les coordonnées cartésiennes de l'effecteur et et les positions des joints pour deux configurations différentes du robot A et B (e.g. effecteur vers le haut et effecteur vers le bas)
* Faîtes bouger le robot infiniement entre les cibles cartésiennes A et B, nous y ajouterons des obstacles plus tard

#### 2.3.3. Déclarer des obstacles
Afin que les algorithmes de planification de trajectoire d'OMPL (tels que `RRTConnect`) puissent éviter les obstacles, il est nécessaire que MoveIt ait connaissance de leur position et leur forme. Il est possible d'utiliser une caméra de profondeur (aka caméra RGB-D, mais nous n'en avons pas ici) ou bien déclarer les objets depuis le code Python grâce à l'interface `PlanningSceneInterface`.

🐍 Par exemple, ce code déclarer une boite de céréales comme objet de collision en spécifiant sa position et son orientation sous forme d'objet `PosteStamped` ainsi que sa taille en mètres :

```python
from geometry_msgs.msg import PoseStamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface

scene = PlanningSceneInterface()
rospy.sleep(1)

ps = PoseStamped()
ps.header.frame_id = "base_link"
ps.pose.position.x = 0.15
ps.pose.position.y = 0
ps.pose.position.z = 0.15
ps.pose.orientation.w = 1
scene.add_box("boite_de_cereales", ps, (0.08, 0.24, 0.3))

rospy.sleep(1)
```

Les coordonnées des objets de collision sont données sous la forme d'objet `PoseStamped` incluant la `position`, l'`orientation` et le repère `frame_id`, et la taille est donnée sous forme de tuple (longueur, largeur, hauteur).

* Modifier l'obstacle "boite_de_cereales" proposé en exemple afin qu'un obstacle viennent perturber le mouvement entre les deux poses de votre programme en 3.2.2. et vérifiez que MoveIt contourne toujours ces obstacles sans jamais les collisionner.

**Note**: Accessoirement, il est possible d'attacher et de détacher les objets de collision au robot, ceci permet par exemple de simuler la saisie et la dépose d'objets physique dans RViz avec MoveIt. cf [la documentation MoveIt pour Python](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html) ou même [le code de `PlanningSceneInterface`](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_commander/src/moveit_commander/planning_scene_interface.py#L56)

#### 2.3.4. Enregistrer et rejouer un mouvement de pick-and-place
Référez-vous à la documentation du [Poppy Controllers](https://github.com/poppy-project/poppy_controllers/#3-trajectory-record-and-playback-feature) afin d'enregistrer et de rejouer des mouvements en utilisant la compliance du robot. Faîtes quelques essais avec plusieurs mouvements qui s'alternent pour bien comprendre le fonctionnement.

* Enregistrez un mouvement de pick-and-place pour attraper un cube et le déposer à un autre endroit

#### 2.4. Récupérer les images de la caméra en Python

💻📀 Avec la carte SD ROS, l'image de la caméra est accessible par appel d'un service dédié. Nous aurons besoin de récupérer le package Poppy Controllers et le compiler d'abord :

```bash
cd ~/ros_ws/src
git clone https://github.com/poppy-project/poppy_controllers.git    # Nous aurons besoin de ce package
cd ~/ros_ws/
catkin_make
source ~/.bashrc
```

🐍 Testez ce code pour vérifier que vous pouvez récupérer l'image en Python via le service ROS `/get_image` fourni par le contrôleur.

```python
import cv2
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

get_image = rospy.ServiceProxy("get_image", GetImage)
response = get_image()
bridge = CvBridge()
image = bridge.imgmsg_to_cv2(response.image)
cv2.imshow("Poppy camera", image)
cv2.waitKey(200)

```
Cette image peut ensuite être traitée par un réseau de neurones, une fonction OpenCV, etc ... 

## Documentation
* [Tutoriaux de MoveIt](https://ros-planning.github.io/moveit_tutorials/)
* [Code du MoveIt Commander Python](https://github.com/ros-planning/moveit/tree/master/moveit_commander/src/moveit_commander)
* [Documentation de l’API MoveIt en Python](http://docs.ros.org/melodic/api/moveit_python/html/namespacemoveit__python.html)
* [Documentation de Poppy Ergo Jr](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/)
