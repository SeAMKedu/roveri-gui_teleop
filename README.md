![roveri](/images/roveri.svg)

# GUI Teleop

Graafisen käyttöliittymän omaava sovellus TurtleBot 4:n etäohjaamiseen ROS2-viestien avulla

## TurtleBot 4

TurtleBot 4 on Clearpath Robotics nimisen yrityksen valmistama mobiilirobotti opetus- ja tutkimuskäyttöön. Mobiilirobotin perustana on iRobot Create 3 -kehitysalusta, jonka päälle on liitetty koteloitu Raspberry Pi 4 -minitietokone. Kotelon päällä on lisäksi RPLIDAR A1M8 -valotutka OAK-D Pro -kamera. Lisätietoja TurtleBot 4:sta löytyy alla olevasta linkistä:

[https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)

## Sovelluksen käyttöliittymä

Vasemmassa ylänurkassa näytetään TurtleBotin akun varaustaso prosentteina. Sen alla on nuolinäppäimet TurtleBotin ohjaamiseen eteen ja taakse, ja pyörimiseen paikallaan sekä myötä- että vastapäivään. TurtleBot liikkuu niin kauan kun hiiren painiketta pidetään pohjassa.
Nuolinäppäimien alapuolella on pudotusvalikot, joista saa valita kertoimet mobiilirobotin kulmanopeudelle ja lineeriselle nopeudelle.

Oikella näytetään videokuvaa TurtleBotin kamerasta. Huomaa, että kamera menee valmiustilaan TurtleBotin ollessa latausasemassa.

Oikeassa alanurkassa on toiminnot TurtleBotin käskyttämiseen latausasemaan (*Dock*) ja siitä pois (*Undock*).

![gui](/images/gui.png)

## Sovelluksen ajaminen

Sovellus käynnistyy alla olevalla komennoilla.
```
$ cd myROS2workspace
$ source install/local_setup.bash
$ ros2 run gui_teleop gui_app
```

Sovelluksen voi sammuttaa painamalla Ctrl+c.

## Tekijätiedot

Hannu Hakalahti, Asiantuntija TKI, Seinäjoen ammattikorkeakoulu

## Hanketiedot

* Hankkeen nimi: Autonomiset ajoneuvot esiselvityshanke
* Rahoittaja: Töysän säästöpankkisäätiön tutkimusrahasto
* Aikataulu: 01.08.2023 - 31.06.2024
---
![rahoittajan_logo](/images/toysan_sp_saatio.jpg)

![seamk_logo](/images/SEAMK.jpg)