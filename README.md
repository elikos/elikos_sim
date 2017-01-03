# ELIKOS_SIM
## Buts
* Simuler le drône et son environnement afin de pouvoir tester le logiciel sans le matériel

## Étapes préliminaires pour effectuer une simulation avec gazebo
1. Aller dans le dossier de simulation de gazebo

        roscd elikos_sim/src/gazebo_sim

2. Mettre à jour les sous-modules git (le Firmware du pixhawk et les modèles gazebo)

        git submodule init
        git submodule update

3. Initialiser les variables d'environnement pour gazebo

        source setup.bash

    Ce script est à éxécuter pour chaque nouvelle console.

    (Optionnel) Ajouter la commande à votre ~/.bashrc pour ne pas avoir 
    à réexécuter le script.

        echo "source $(pwd)/setup.sh" >> ~/.bashrc

4. Builder le code du Firmware

    Suivre les instructions pour installer les dépendances requises : http://dev.px4.io/starting-building.html

        cd Firmware
        make posix_sitl_default gazebo

    Si tout fonctionne, Gazebo devrait se lancer en affichant le modèle iris par défaut.

## Launch files 
* `simulation.launch`
*Descritpion*
    * Lance une simulation rviz
        
* `gazebo_sim.launch`
*Description*
    * Lance une simulation gazebo, le firmware du pixhawk et mavros
    * `world` : Le chemin vers le fichier .world qui décrit l'environnement
    * `sdf` : Le chemin vers le fichier .sdf qui décrit le véhicule simulé

## Initilaliser la simulation gazebo
Après avoir lancé le launch file pour la simulation gazebo, envoyer un setpoint à une position dans les airs
`rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped (*Peser tab et remplir le message*) -r 10`
`rosservice call /mavros/cms/arming 1`
`rosservice call /mavros/set_mode 216 OFFBOARD`


