# ELIKOS_SIM
## Buts
* Simuler le drône et son environnement afin de pouvoir tester le logiciel sans le matériel

## Simulation avec rviz (2017-2018)

Cloner `elikos_roomba` et `create_autonomy` dans le dossier `src/` du même workspace  
````
git clone https://github.com/elikos/elikos_roomba.git
git clone https://github.com/AutonomyLab/create_autonomy.git
````

S'assurer de init et update les submodules (on a besoin des modèles!)  
````
git submodule init
git submodule update --recursive --remote
````

Utiliser le launch file `roomba_sim.launch`  
````
roslaunch elikos_sim roomba_sim.launch
````

Pour faire bouger les robots  
````
rosservice call /sim/toggle_activate
````

Pour reset les robots  
````
rosservice call /sim/reset
````

Pour envoyer un setpoint au quad, publier un message `DMCmd` avec la pose voulue sur `/elikos_decisionmaking_cmd`  
````
rostopic pub /elikos_decisionmaking_cmd <tab> <tab>
````


## Étapes préliminaires pour effectuer une simulation avec gazebo
1. Aller dans le dossier de simulation de gazebo

        roscd elikos_sim/src/gazebo_sim

2. Mettre à jour les sous-modules git (le Firmware du pixhawk et les modèles gazebo)

        git submodule init
        git submodule update --recursive --remote

3. Initialiser les variables d'environnement pour gazebo

    Note: Assurez-vous qu'un différent package px4 n'a pas déjà été sourcé avec `rospack find px4`
          Le package px4 ne devrait pas exister.

        source setup.bash

    Ce script est à éxécuter pour chaque nouvelle console.

    (Optionnel) Ajouter la commande à votre ~/.bashrc pour ne pas avoir 
    à réexécuter le script.

        echo "source $(pwd)/setup.sh" >> ~/.bashrc

4. Builder le code du Firmware

    Suivre les instructions pour installer les dépendances requises : http://dev.px4.io/starting-installing-linux.html
    Note: Installer les dépendances pour Nuttx based hardware.

    Ajouter les packages de osrfoundation.org pour gazebo : http://gazebosim.org/tutorials?tut=install_ubuntu

    Installer gazebo 7 pour ros

        sudo apt-get install ros-$(ROS_DISTRO)-gazebo-ros-pkgs

    Builder le firmware pour gazebo

        cd Firmware
        make posix_sitl_default gazebo

    Si tout fonctionne, Gazebo devrait se lancer en affichant le modèle iris par défaut.

    Fermer la simulation pour les étapes suivantes.

## Launch files 

* `roomba_sim.launch`
    * Lance une simulation avec rviz 

* `simulation.launch`
    * Lance une simulation rviz
 
* `gazebo_sim.launch`
    * Lance une simulation gazebo, le firmware du pixhawk et mavros
    * `world` : Le chemin vers le fichier .world qui décrit l'environnement
    * `sdf` : Le chemin vers le fichier .sdf qui décrit le véhicule simulé
    * `config_yaml` : Le chemin vers le fichier .yaml qui contient les configurations pour mavros.

* `integration_sim.launch`
    * Lance les différents nodes d'elikos_main pour la simulation

* `robots_sim.launch`
    * Ajoute des cibles et des obstacles se déplaçant dans la simulation.

## Positionner le drône
Après avoir lancé le launch file pour la simulation gazebo, envoyer un setpoint à une position dans les airs

`rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped (*Peser tab et remplir le message*) -r 10`

Si `/mavros/setpoint_position/tf/listen` est à `true` faire plutôt : 

`rosrun tf static_transform_publisher 0 0 2 0 0 0 1 elikos_arena_origin elikos_setpoint 100`

Ensuite, il suffit d'armer le quad:

`rosservice call /mavros/cmd/arming 1`

`rosservice call /mavros/set_mode 216 OFFBOARD`

Voir le [tutoriel de simulation](https://elikos.gitbooks.io/wiki-elikos/content/Navigation/lancer_une_simulation_avec_gazebo.html) pour plus de détails.

