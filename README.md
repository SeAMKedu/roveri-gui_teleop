![roveri](/images/roveri.svg)

# GUI Teleop

Graafisen käyttöliittymän omaava sovellus TurtleBot 4:n etäohjaamiseen ROS2-viestien avulla.

## TurtleBot 4

TurtleBot 4 on Clearpath Robotics nimisen yrityksen valmistama mobiilirobotti opetus- ja tutkimuskäyttöön. Mobiilirobotin perustana on iRobot Create 3 -kehitysalusta, jonka päälle on liitetty koteloitu Raspberry Pi 4 -minitietokone. Kotelon päällä on lisäksi RPLIDAR A1M8 -valotutka OAK-D Pro -kamera. Lisätietoja TurtleBot 4:sta löytyy alla olevasta linkistä:

[https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)

## Sovelluksen käyttöliittymä

Vasemmassa ylänurkassa näytetään TurtleBotin akun varaustaso prosentteina. Sen alla on nuolinäppäimet TurtleBotin ohjaamiseen eteen ja taakse, ja pyörimiseen paikallaan sekä myötä- että vastapäivään. TurtleBot liikkuu niin kauan kun hiiren painiketta pidetään pohjassa.
Nuolinäppäimien alapuolella on pudotusvalikot, joista saa valita kertoimet mobiilirobotin kulmanopeudelle ja lineeriselle nopeudelle.

Oikella näytetään videokuvaa TurtleBotin kamerasta. Huomaa, että kamera menee valmiustilaan TurtleBotin ollessa latausasemassa.

Oikeassa alanurkassa on toiminnot TurtleBotin käskyttämiseen latausasemaan (*Dock*) ja siitä pois (*Undock*).

![gui](/images/gui.png)

## Ohjelmistoriippuvuudet

Sovellus on kehitty käyttämällä ROS2:n Humble-versiota.

Asenna tarvittavat TurtleBot 4 -ohjelmistopaketit:
[https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html)

Sovellus käyttää alla listattuja Python-paketteja:
* [customtkinter](https://pypi.org/project/customtkinter/) graafinen käyttöliittymä
* [opencv](https://pypi.org/project/opencv-python/) videokuvan käsittely
* [pillow](https://pypi.org/project/pillow/) kuvan käsittely

Lisäksi tarvitaan [cv_bridge](https://index.ros.org/p/cv_bridge/)-paketti, joka muuntaa ROS-kuvaviestit OpenCV:n käyttämään formaattiin.


## Sovelluksen asennus ja ajaminen

Lataa ensin sovelluksen lähdekoodi ROS2 workspace -kansion *src*-alikansioon.
```
$ cd myROS2workspace/src/
$ git clone https://github.com/SeAMKedu/roveri-gui_teleop.git
```

Nimeä sovelluksen kansio uudestaan: *roveri-gui_teleop* -> *gui_teleop*.

Siirry ROS2 workspace -kansion juureen ja käännä sovellus *colcon*-työkalulla:
```
$ cd ..
$ colcon build --packages-select gui_teleop
```

Sovellus voidaan nyt käynnistää alla olevilla komennoilla.
```
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