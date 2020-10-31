# III. Robotique de manipulation avec E.DO

La robotique de manipulation regroupe la manipulation d'objets avec des robots : des bras articulés à 5 ou 6 axes, les robots [SCARA](https://en.wikipedia.org/wiki/SCARA) (Selective Compliance Assembly Robot Arm), les robots [cartésiens](https://en.wikipedia.org/wiki/Cartesian_coordinate_robot) (linéaires), les robots [parallèles](https://en.wikipedia.org/wiki/Parallel_manipulator) ... Dans ce TP nous utilisons un robot [E.DO du fabriquant Comau](https://edo.cloud/).

## Prérequis

* Lycée et +
* Notions de commandes dans un terminal et d'adressage IP
* Le [TP d'introduction](../introduction)
* Ce TP est compatible avec la simulation si vous n'avez pas d'E.DO : sauter directement au 2.3.bis

## Diapositives

{% pdf src="https://files.ros4.pro/manipulation.pdf", width="100%", height="565px" %}{% endpdf %}

## 1. Préparer le matériel

🔌 Votre E.DO comporte un connecteur Ethernet sur la base, c'est sur ce connecteur que vous pouvez connecter un câble réseau. Nous vous conseillons de brancher un câble RJ45 entre le robot et votre ordinateur pour commencer.Plus tard vous pourrez aussi vous connecter au point d'accès Wifi du robot pour communiquer en sans-fil.

A l'intérieur de la base se trouve une Raspberry Pi et une carte SD préconfigurée par le fabriquant, elle est accessible en dévissant les trappes. Votre robot est donc compatible avec la plupart des procédures de la [FAQ Raspberry Pi](../../faq/pi). 

⚠️ Le robot peut être vendu avec plusieurs modèles d'effecteurs (= pince) ou bien sans effecteur du tout. Ce TP présuppose que vous avez la version avec la pince à 2 états : ouvert et fermé.

### 1.1 Changer l'adresse IP de l'ordinateur ROS
E.DO est livré préconfiguré avec son propre réseau IP. Veuillez éditer la configuration réseau Ethernet (câblé) de votre ordinateur Ubuntu en utilisant le gestionnaire réseau (network manager, en haut à droite, vers l'horloge). Attribuez la configuration IP fixe suivante :

* Adresse IP statique `10.42.0.1`
* Masque de sous-réseau `24` ou bien `255.255.255.0`

⚠️ Attention, si vous souhaitez disposer aussi d'une connexion à Internet, cela ne focntionnera plus à cause du changement d'adresse IP. Vous pouvez cependant connecter votre ordinateur à un réseau Wifi pour disposer aussi d'un accès Internet.

### 1.2 Pinguer le robot

Mettez votre robot sous-tension puis depuis un terminal Ubuntu tapez `ping 10.42.0.49`. Si tout va bien, un message apparaitra chaque seconde en indiquant le délai de communication avec le robot en millisecondes (ms), jsuqu'à ce que vous l'interompiez avec Ctrl + C :

```bash
$ ping 10.42.0.49
PING 10.42.0.49 (10.42.0.49) 56(84) bytes of data.
64 bytes from 10.42.0.49: icmp_seq=1 ttl=114 time=3.7 ms
64 bytes from 10.42.0.49: icmp_seq=2 ttl=114 time=5.6 ms
^C
```

Si un message d'erreur s'affichage à la place du délai en millisecondes, vous avez un problème réseau. Vérifiez l'étape 1.1 et que votre robot a bien démarré avec sa configuraiton réseau d'origine. Si vous voyez les délais en millisecondes, vous pouvez continuer.

### 1.3. Configurer l'environnement ROS du robot
💻 Avec SSH, connectez-vous à la Raspberry Pi du robot :
```bash
ssh edo@10.42.0.49
```
Le mot de passe par défaut est `raspberry`.

Si la connexion est un succès vous deviez voir un message d'information italien `comandi tmux` 🇮🇹 avec quelques explications à propos de rostopic, que nous devez connaître puisqu nous l'déjà abordée dans l'[introduction](../../introduction). Si ce message n'apparait pas, vérifiez la configuration réseau du titre 1.1.

🤖 Avec la commande nano, éditez ensuite le script `ministarter` du robot via SSH :
```
nano ~/ministarter
``` 

🤖 Descendez avec les flèches du clavier pour identifier ces deux lignes :
```
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
```
puis remplacez-les par ces 2 lignes modifiées :
```
export ROS_MASTER_URI=http://10.42.0.49:11311
export ROS_IP=10.42.0.49
```
Quittez en tapant `Ctrl-X`, puis `y` et `Entrée` pour valider le nom de fichier et l'enregistrer.

🤖 Tapez ensuite `sudo reboot` pour redémarrer le robot et attendez qu'il soit de nouveau prêt.

⚠️ Il n'est nécessaire de configurer l'environnement ROS qu'une seule fois sur chaque nouveau robot, car la nouvelle configuration est ensuite enregistrée définitivement sur la carte SD de la Raspberry Pi internet à E.DO.

### 1.4 Configurer l'environnement ROS de l'ordinateur

📀💻 Pour installer tous les packages ROS nécessaires sur votre ordinateur, exécutez les commandes suivantes :

```bash
roscd && cd src
git clone https://github.com/eDO-community/eDO_control_v3.git
git clone https://github.com/eDO-community/eDO_moveit.git
git clone https://github.com/eDO-community/eDO_description.git
git clone https://github.com/eDO-community/eDO_core_msgs.git
pip install getkey numpy
cd ..
catkin_make
```

Un script de démarrage nommé [`start.bash`](https://github.com/eDO-community/eDO_control_v3/blob/master/start.bash) configure les variables d'environnement our vous à chque fois que vous devrez travailler avec votre E.DO. Ce script ajoute un préfixe jaune devant l'invite de commande pour savoir quel est le ROS master actuellement sélectionné.

📀💻 Pour exécutez ce script tapez :

```bash
roscd edo_control
./start.bash
```

Vous devriez voir apparaître en préfixe le ROS master de votre E.DO, avant de taper toute autre commande ROS, comme ci-dessous. Essayez un `rostopic echo` pour vérifier si tout va bien :

```bash
[http://10.42.0.49:11311] me@workstation :~$ rostopic echo /machine_state -n1
current_state: 0
opcode: 0
```

Les valeurs `current_state: 0` et `opcode: 0` indiquent l'état actuel du robot, dans un topic ROS dédié nommé `/machine_state`. Si vous ne voyez pas ces deux valeurs en tapant la commande, vous pourriez avoir un problème de réseau ou de configuration ROS.

## 2. Travaux pratique
### 2.1. Calibrer le robot

**Pourquoi calibrer ?** Chacun des joints (axes moteurs) de votre robot possède un encodeur : un capteur permettant au moteur de déterminer sa position angulaire courante (par exemple tourné à 90° ou à 150°, etc). Plusieurs technologies d'encodeurs existent ayant chacune des avantages et inconvénients. Un des inconvénients des encodeurs d'E.DO est qu'ils ne peuvent mesurer que des déplacements angulaires relatifs à leur angle de démarrage, mais ne savent pas où est l'angle 0°. Vous ne pouvez pas commander un moteur d'aller en position 90° s'il ne sait pas où et le 0°. L'étape de calibration sert à indique au robot où sont les angles de référence 0° de chacun de ses moteurs, un par un.

**Quand calibrer ?** Vous devrez calibrer votre robot après chaque cycle de marche-arrêt. La raison pour cela est que seule une partie des moteurs possèdent des freins, pour éviter q'ils se décalibrent en étant à l'arrêt. En effet, si vos moteurs sont à l'arrêt et sans frein les empêchant de tourner, la gravité ou une action humaine pourrait les faire tourner malgré eux, sans qu'ils puissent enregistrer ces rotations puisqu'ils ne sont pas sous tension. Ils perdraient ainsi la trace de leur angle 0°. Il faut donc calibrer à chaque démarrage, ou alors s'assurer qu'aucun des moteurs ne bouge pendant que le robot n'est pas sous tension, ce qui est difficile à s'assurer sans frein. 

**Comment calibrer ?** E.DO est livré avec une application Android, permettant nottamment de le calibrer. Le constructeur fournit une application Android permettant entre autre de calibrer le robot. Vous pourriez utiliser cette application mais puisque nous sommes sur un TP ROS, nous allons le faire avec ROS.

💻 Démarrez la procédure de calibration en tapant :
```bash
 roslaunch edo_control calibrate.launch
```

Vous devirez d'avord voir un message d'avertissement `JOINT_UNCALIBRATED` qui indique que les joints ne sont effectivement pas calibrés, et qui détaille la procédure de calibration en anglais. Suivez cette procédure jusqu'au bout.

A chaque fois que vous voyez `Calibrating joint X` cela signifie que la procédure va calibrer le joint X. Pour chaque joint appuyez sur les touches gauche et droite pour aligner physiquement les marqueurs d'alignement de chaque moteur. Une fois que votre joint est calibré appuyez sur Entrée pour passer au suivant.

ℹ️ La position 0° calibrée de chaque joint doit conduire petit-à-petit votre robot à se tenir droit comme un i. Si à la fin de la calibration votre robot n'est pas droit comme un i pointant vers le plafond, vous vous êtes trompé sur l'alignement d'un ou plusieurs moteurs, vous pouvez relancer la procédure.

La commande de calibration se ferme d'elle-même lorsque tous les joints ont été calibrés.

### 2.2. Démarrer le contrôle du robot et ouvrir/fermer la pince

💻 Lancez le launchfile de contrôle du robot sur votre ordinateur :

```bash
 roslaunch edo_control control.launch
```
Cela va activer les interfaces de communication suivant avec le robot :

* Le topic `/joint_states`, qui affiche tout l'état des joints à environ 90 Hz : positions angulaires, vitesses et torques
* Le serveur d'action `/follow_joint_trajectory` qui permettra d'exécuter des trajectoires avec MoveIt un peu plus tard dans le TP
* Le topic `/open_gripper` qui permet d'ouvrir et de ferme la pince

**Tester la commande de la pince** : Sur ce dernier topic, vous pouvez publier `true` pour ouvrir la pince (course maximale de 60 mm) et `false` pour la fermer (course minimale de 0 mm).

💻 Conservez le contrôle du robot démarré dans un terminal. Dans un autre terminal testez l'ouverture de la pince avec `rostopic` :
```
rostopic pub /open_gripper std_msgs/Bool "data: true"
```

Remplacez `true` par `false` pour refermer la pince !

### 2.3. Démarrer MoveIt pour planifier des trajectoires (avec un robot réel)

💻 Conservez le contrôle du robot démarré dans un terminal. Tapez la commande suivante pour démarrer MoveIt avec E.DO :
```bash
roslaunch edo_moveit demo.launch
```

Vous devriez voir l'interface de MoveIt qui démarre dans RViz similairement à la capture d'écran ci-dessous :

![MoveIt avec E.DO](https://raw.githubusercontent.com/eDO-community/eDO_moveit/master/img/screen.png)

Utilisez ensuite les outils MoveIt pour **planifier et exécuter des trajectoires sur votre robot** :

1. Dans la zone `Motion Planning` en bas à gauche de RViz, sélectionnez l'onglet `Planning`
2. Dans la vue 3D du robot, bougez la balle bleue correspondant à l'effecteur (la pince) quelque part dans l'espace autour du robot
3. Le robot orange correspond à la configuration cible que vous allez demander d'atteindre à votre robot
4. Cliquez sur `Plan and Execute` pour planifier une trajectoire vers cette cible et l'exécuter sur le robot réel

Si vous ne voyez pas votre vrai robot bouger, vérifier dans le terminal à partir duquel vous avez démarré MoveIt : il se peut que des messages d'erreurs apparaissent en rouge pour vous aider à localiser le problème.

Ce TP s'arrête ici mais il est possible d'ajouter des éléments de collision (obstacles) que le planificateur de trajectoire contournera. Pour ce faire vous pouvez charger un fichier comprenant les coordonnées des obstacles dans l'onglet `Scene Objects` de la zone `Motion Planning` ou bien pour le faire via Python vous pouvez vous inspirer de la partie de déclaration des obstacles de cet [autre TP](../ergo-jr/#233-déclarer-des-obstacles).

#### Que faire si les freins des moteurs s'activent aléatoirement pendant le mouvement
Si vos trajectoires sont interrompues par des activations intempestives des freins (suivies d'une désactivation), vérifiez d'abord que votre robot a été calibré avec précision. Si c'est le cas, cela signifie que votre robot subit trop de force pour exécuter le mouvement que vous lui demander. Le système de sécurité active donc les freins. Ralentissez les vitesses des joints dans [`kinematics.yaml`](https://github.com/eDO-community/eDO_moveit/blob/master/config/joint_limits.yaml) ou bien alléger le poids de votre robot ou bien demandez une position cible qui fera moins forcer les moteurs.

### 2.3.bis. Démarrer MoveIt pour planifier des trajectoires (avec un robot simulé)

📀💻 Pour installer tous les packages ROS nécessaires sur votre ordinateur et travailler avec un robot simulé, il ne faut pas avoir configuré l'environnement décrit en 1.4, exécutez directement les commandes suivantes :

```bash
roscd && cd src
git clone https://github.com/eDO-community/eDO_control_v3.git
git clone https://github.com/eDO-community/eDO_moveit.git
git clone https://github.com/eDO-community/eDO_description.git
git clone https://github.com/eDO-community/eDO_core_msgs.git
roscd && catkin_make
source ~/.bashrc
```

💻 Enfin, cette commande ci-dessous lancera MoveIt avec un E.DO simulé :

```bash
roslaunch edo_moveit demo.launch simulated:=true
```

Vous devriez voir l'interface de MoveIt qui démarre dans RViz similairement à la capture d'écran ci-dessous :

![MoveIt avec E.DO](https://raw.githubusercontent.com/eDO-community/eDO_moveit/master/img/screen.png)

Utilisez ensuite les outils MoveIt pour planifier et exécuter des trajectoires sur votre robot simulé :

1. Dans la zone `Motion Planning` en bas à gauche de RViz, sélectionnez l'onglet `Planning`
2. Dans la vue 3D du robot, bougez la balle bleue correspondant à l'effecteur (la pince) quelque part dans l'espace autour du robot
3. Le robot orange correspond à la configuration cible que vous allez demander d'atteindre à votre robot
4. Cliquez sur `Plan and Execute` pour planifier une trajectoire vers cette cible et l'exécuter sur le robot simulé

Ce TP s'arrête ici mais il est possible d'ajouter des éléments de collision (obstacles) que le planificateur de trajectoire contournera. Pour ce faire vous pouvez charger un fichier comprenant les coordonnées des obstacles dans l'onglet `Scene Objects` de la zone `Motion Planning` ou bien pour le faire via Python vous pouvez vous inspirer de la partie de déclaration des obstacles de cet [autre TP](../ergo-jr/#233-déclarer-des-obstacles).