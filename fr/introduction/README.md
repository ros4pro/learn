# I. Introduction à Linux, ROS et ROS-I
## Prérequis

* Lycée et +
* Notions de programmes informatiques, terminaux et commandes

## Diapositives

{% pdf src="https://files.ros4.pro/introduction.pdf", width="100%", height="565px" %}{% endpdf %}

## 1. Démarrer Ubuntu et ROS

Selon votre situation, une clé USB bootable nommée "clé Live" peut vous être fournie.

### Cas 1.A : Une clé USB bootable m'est fournie
Vous devez faire "booter" votre poste de travail sur la clé USB Live fournie en vous aidant si nécessaire de [la procédure dédiée](https://files.ros4.pro/boot.pdf).

Votre clé est fournie avec tous les paquets préinstallés pour le workshop. Ainsi, dans les TP vous devrez sauter toutes les étapes précédées du pictogramme "disque" suivant : 📀 car ces étapes ont déjà été réalisées. 

Localisez l'application "Terminator" sur votre clé USB Live puis passez directement au titre 2. ci-dessous dès que vous avez réussi à ouvrir un terminal.

### Cas 1.B : Je n'ai pas de clé USB bootable, j'installe Ubuntu et ROS moi-même

Installez Ubuntu 20.04 et ROS Noetic selon [les prérequis](https://files.ros4.pro/prerequis.pdf). Vous aurez également à installer vous-même des éléments supplémentaires tout le long des travaux pratiques : ces étapes d'installation sont signalées par un pictogramme "disque" 📀. Nous recommandons l'installation et l'usage de [Visual Studio Code](https://code.visualstudio.com/Download) comme environnement de développement intégré (optionnel).

Les utilisateurs dans le cas B doivent également configurer leur environnement ROS :
- 1.B.1. en créant un espace de travail dédié à ROS : tous vos packages ROS se trouveront dans le dossier `src` de l'espace de travail nommé `catkin_ws` :

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

- 1.B.2. en plaçant les lignes suivantes à la fin du fichier `~/.bashrc` (commentaires compris) :

```bash
# ROS workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#export ROS_HOSTNAME=$(hostname).local
export ROS_IP=`ip address|grep inet|grep dynamic|tr ' ' ':'|cut -d':' -f6|cut -d'/' -f1|head -n1`
export TURTLEBOT3_MODEL=burger

# CHOOSE A ROS MASTER: local host, Poppy robot or Turtlebot robot
# Add # in front of the lines that you want to disable
ROS_MASTER_URI=http://localhost:11311
#ROS_MASTER_URI=http://poppy.local:11311
#ROS_MASTER_URI=http://raspberrypi.local:11311

```

- 1.B.3. en fermant et rouvrant votre terminal. Vous ne devez visualiser aucune erreur lors de la réouverture du terminal et devez pouvoir taper la commande `roscore` sans erreur (tapez Ctrl+C pour quitter roscore).

Cette configuration est partiellement expliquée dans les diapositives. Pour plus d'information, référez-vous à la documentation ROS concernant [la configuration de l'environnement ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

## 2. Prise en main du terminal : le rosier 🌹
⌨️ Pour prendre en main le terminal Linux et les commandes de base, à partir d'un terminal, créez les fichiers et dossiers nécessaires pour réaliser cette hierarchie de fichiers ressemblant à un rosier :

![Hierarchie de fichier du rosier](../img/rosier.png)

Vous aurez besoin des commandes suivantes :h
* `ls`, pour lister les fichiers et dossiers courants
* `cd`, pour changer le dossier courant
* `mkdir`, pour créer un nouveau dossier
* `touch`, pour créer un nouveau fichier vide
* `nano`, pour créer un nouveau fichier et écrire à l'intérieur
* `tree`, pour afficher la hierarchie de fichiers

## 3. Tutoriels

🧑‍🏫 Vous êtes désormais prêt à utiliser ROS ! Suivez les tutoriels ROS suivants pour découvrir et tester les concepts de base, sachant que votre distribution ROS s'appelle `noetic` :

* [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) : Maîtriser ROS master (`roscore`) et lancer des nœuds (`rosrun`)
* [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics) : Envoyer et recevoir des messages dans un topic (`rostopic`)
* [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams) : Déclarer et appeler un service requête/réponse (`rosservice`, `rossrv`)


❓ [**Quizz** : quizz au tableau pour mémoriser les commandes importantes](quizz.pdf)

## 4. ⚙️ Préparer vos robots

Pour l'un ou l'autre de vos 2 robots, réalisez les étapes de préparation suivantes expliquées [dans la FAQ robots](../faq/pi/) :
1. Flasher sa carte SD
2. Connecter le robot en wifi
3. Se connecter via SSH au robot
4. Personnaliser le nom de votre robot (si nécessaire)

## 5. FAQ
### 📥 Mise à jour pendant le TP
Il se peut que l'enseignant mette à jour les ressources pendant le cours. Dans ce cas exécutez les commandes suivantes pour récupérer les dernières mises-à-jour :
```bash
roscd ros4pro
git pull origin poppy_tb3_keras
```
Si l'erreur suivante survient :
```
error: Vos modifications locales aux fichiers suivants seraient écrasées par la fusion :
	<LISTE DE FICHIERS>
Veuillez valider ou remiser vos modifications avant la fusion.
Abandon
```
Alors les fichiers spécifiés ne peuvent pas être mis à jour car cela détruirait les modifications que vous avez apportées à la liste des fichiers indiquée. Il est recommandé de demander conseil avant d'essayer une autre action pour récupérer la mise à jour.
