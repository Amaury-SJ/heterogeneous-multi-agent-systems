# GPS-RTK experiments: precision and stability

This section details how to capture the data and reproduce the results obtained, as presented in the thesis of this research project. To consult the results of this section, we invite you to read our [published article](https://hal.science/hal-04311426), which can also be found in our list of published works in the section {doc}`papers-articles-thesis-citations`.

```{warning}
The tests were carried out in a preliminary phase of the project, so the format of the SatMsgRcv message provided by the gps_msg_pckg package is different. The old format should be recovered with the file `/////SatMsgRcv.msg`, then overwritten with the current format contained in our `gps_msg_pckg` package, which is the file `gps_msg_pckg/msg/SatMsgRcv.msg`. Remember to keep a copy of the overwritten format.
If this procedure is not followed using the appropriate format, it will not be possible to read the topics.
```

## Experimentation procedure

Tous les codes sont contenus dont le package associé à ces expérimentations : `////`.
Nous pouvons lancer les différentes parties, que ce soit l’acquisition des données des 4 GPS, le calcul des distances ou l’affichage en 3D, tout est configurable et exécutable via le fichier launch `///square4gps.launch.py` Le fichier est réglé par défaut pour ne faire que les calculs et l’affichage 3D via RViz2. Il faut donc lancer un bag file à part et rqt pour le tracer de courbe.

La récupération des données des 4 GPS se fait à l’aide de nodes gps_talker du package `gps_rtk_pckg`. Et pour la partie visualisation 3D à l’aide de RViz, cela est fait grâce à notre package `display_rviz2_pckg`.

```{note}
Il se peut que la map dans RViz2 ne s’affiche pas directement, car dans le fichier launch nous avons précisé que nous utilisons le GPS en bas à gauche pour calibrer la position de la map, et ce dernier doit être en solution fixe pour lancer l’affichage de la map.
```

Une fois qu’on a les différents topics contenant la position des récepteurs GNSS, on souhaite calculer les distance euclidienne 3D entre 2 antennes. Nous avons réalisé une node `calculate_distance_gps.py` permettant de subscribe à deux topics au choix et de publier ensuite un topic sur la distance obtenue.

Voici le graph ROS 2 du système avec la partie tracé des courbes et affichage 3D sous RViz 2. (image)

## Registered bag files

To find out how to save your data with ROS 2, we recommend that you first read the section on {doc}`bag-data-ros2`.
Plusieurs expérimentations ont été menées, et chaque bag file correspond à une situation. Ils sont stockés dans le dossier `///`.

- rosbag2_2022_11_08-17_11_34_sur_place
- rosbag2_2022_11_08-17_14_57_sur_place_fix_avec_perturbation
- rosbag2_2022_11_08-17_23_08_tourne_sur_place
- rosbag2_2022_11_08-17_24_30_ligne_droite
- rosbag2_2022_11_08-17_26_21_carre