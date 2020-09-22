# Uuden kombinaation tekeminen

Yhdistä työkalu käsivarteen URDF määrityksellä:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="samk_<työkalu>">
  <xacro:include filename="$(find elfin_description)/urdf/elfin5.urdf.xacro" />
  <xacro:include filename="<työkalu paketti>" />

  <joint name="tool_connector" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="elfin_end_link" />
    <child link="<työkalu>" />
  </joint>
</robot>
```

Tallenna kokoonpano nimellä samk_<työkalu>.urdf.xacro

Varmista toimivuus: `roslaunch samk_description display.launch model:=samk_<työkalu>`

## MoveIt! paketin teko

Jos malli toimii, luodaan MoveIt määritys: `roslaunch moveit_setup_assistant setup_assistant.launch`

![Moveit ikkuna 0](img/moveit_00.png "Moveit ikkuna")

### Self-Collisions

Valitaan vasemmalta "Self-Collisions" ja muutetaan Sampling Density maksimiin ja generoidaan matriisi.

![Moveit ikkuna 1](img/moveit_01.png "Moveit self-collisions")

![Moveit ikkuna 2](img/moveit_02.png "Moveit self-collisions max")

### Planning Groups

![Moveit ikkuna 3](img/moveit_03.png "Moveit planning groups")

Paina "Add Group" nappia, annetaan ensinmäisen ryhmän nimeksi `elfin_arm` ja kinematic solveriksi `elfin5_elfin_arm_kinematics/IKFastKinematicsPlugin`.

Jätetään Kin. Search Reolution ja Kin. Search Timeout vakioiksi. Paina "Add Joints" nappia.

![Moveit ikkuna 4](img/moveit_04.png "Moveit add group")

Valitaan elfin_joint1-6 tähän ryhmään.

![Moveit ikkuna 5](img/moveit_05.png "Moveit nivelet")

Lisätään elfin_end_connector linkki määritysryhmään.

![Moveit ikkuna 6](img/moveit_06.png "Moveit linkki")

![Moveit ikkuna 7](img/moveit_07.png "Moveit linkki 2")

Luodaan työkalusta myös oma planning group.

Tähän ei laiteta mitään kinematic solveria. Valitaan vain linkkejä.

![Moveit ikkuna 8](img/moveit_08.png "Moveit group 2")

![Moveit ikkuna 9](img/moveit_09.png "Moveit linkki 3")

### Robot Poses

Määritetään robotillemme valmiita asentoja, tässä voimme myös testata että kaikki nivelet liikkuvat niinkuin pitää.

![Moveit ikkuna 10](img/moveit_10.png "Moveit robot pose 1")

![Moveit ikkuna 11](img/moveit_11.png "Moveit robot pose 2")

![Moveit ikkuna 12](img/moveit_12.png "Moveit robot pose 3")

Voit lisätä asentoja niin monta kuin haluat, mutta jos koitat poistaa asentoa, MoveIt setup assistant kaatuu.

### End effector

Lisätään MoveIt järjestelmän loppupään käsittelijä, eli työkalu.

![Moveit ikkuna 13](img/moveit_13.png "Moveit end effector 1")

![Moveit ikkuna 14](img/moveit_14.png "Moveit end effector 2")

### ROS Controllers

Määritetään mitkä ohjaimet ohjaavat käsivartta ja työkalua.

![Moveit ikkuna 15](img/moveit_15.png "Moveit controllers 1")

![Moveit ikkuna 16](img/moveit_16.png "Moveit controllers 2")

![Moveit ikkuna 17](img/moveit_17.png "Moveit controllers 3")

![Moveit ikkuna 18](img/moveit_18.png "Moveit controllers 4")

![Moveit ikkuna 19](img/moveit_19.png "Moveit controllers 5")

### Author information

Tämä kohta on __PAKOLLINEN__, tähän ei kuitenkaan tarvitse laittaa oikeita tietoja.
![Moveit ikkuna 20](img/moveit_20.png "Moveit author")

### Configuration files

![Moveit ikkuna 21](img/moveit_21.png "Moveit generate configuration files 1")

Valitse kansio, johon haluat luoda MoveIt konfiguraatio paketin ja paina `Generate Package` nappia. Tämä antaa virheen, että emme ole määrittänyt virtual jointtia, tämä voidaan ohittaa painamalla OK.

![Moveit ikkuna 22](img/moveit_22.png "Moveit generate configuration files 2")

Kun paketti on luotu, paina `Exit Setup Assistant` nappia.

![Moveit ikkuna 23](img/moveit_23.png "Moveit generate configuration files 3")
