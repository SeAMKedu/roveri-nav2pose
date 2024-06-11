![roveri](/images/roveri.svg)

# Nav2pose

ROS2-sovellus, jolla TurtleBot 4 navigoi kahden yhteistyörobotin välillä kuljettaen työkappaletta.

## TurtleBot 4

TurtleBot 4 on Clearpath Robotics nimisen yrityksen valmistama mobiilirobotti opetus- ja tutkimuskäyttöön. Mobiilirobotin perustana on iRobot Create 3 -kehitysalusta, jonka päälle on liitetty koteloitu Raspberry Pi 4 -minitietokone. Kotelon päällä on lisäksi RPLIDAR A1M8 -valotutka OAK-D Pro -kamera. Lisätietoja TurtleBot 4:sta löytyy alla olevasta linkistä:

[https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)

## Sovelluksen toiminta

Sovelluksen avulla TurtleBot voi toimia yhdessä kahden Robotiikan laboratoriossa (A140.4) olevan yhteistyörobotin kanssa. TurtleBot navigoi aluksi ABB-yhteistyörobotin luokse, joka lastaa työkappaleen TurtleBotin päälle. Seuraavaksi TurtleBot navigoi UR5-yhteistyörobotin luo, joka poimii konenäön avulla työkappaleen TurtleBotin päältä.

TurtleBot ja yhteistyörobotit saavat tietoa toistensa tiloista erillisen socket-palvelimen kautta.

Sovellus lähettää socket-palvelimelle alla olevie viestejä:
* **"getState,abb"** Sovellus kysyy ABB-yhteistyörobotin tilaa
* **"getState,ur5"** Sovellus kysyy UR5-yhteistyörobotin tilaa
* **"setState,tb4,docked"** TurtleBot on latausasemassa
* **"setState,tb4,moving"** TurtleBot on liikkeessä
* **"setState,tb4,onABB"** TurtleBot on ABB-yhteistyörobotin luona
* **"setState,tb4,onUR5"** TurtleBot on UR5-yhteistyörobotin luona

Kun sovellus käynnistetään, se tarkistaa onko TurtleBot latausasemassa. Jos TurtleBot on irti latausasemasta, sovellus ilmoittaa TurtleBotin olevan liikkeessä ja aloittaa sitten telakointi-toiminnon. Sovellus lähettää viestin "setState,tb4,docked", kun TurtleBot on kiinni latausasemassa.

TurtleBotin ollessa vielä latausasemassa, sovellus asettaa sen alkuasennon käytettävässä kartassa. Asento (engl. *pose*) koostuu X- ja Y-koordinaateista sekä suunnasta, johon TurtleBotin "nenä" osoittaa.

Sovellus jää odottamaan kunnes Nav2-navigointijärjestelmä on aktiivinen.

Alla olevat asiat tapahtuvat while-silmukassa.

Sovellus kysyy käyttäjältä ABB-yhteistyörobotin X- ja Y-koordinaatteja. Jos niitä ei anneta, sovellus käyttää joko kovakoodattuja tai aikaisemmin annettuja koordinaatteja.

Seuraavaksi sovellus käskee TurtleBottia irrottautumaan latausasemasta (*undock*) ja ilmoittaa socket-palvemelle TurtleBotin olevan liikkeessä ("setState,tb4,moving"). Tämän jälkeen alkaa varsinainen navigointi kohti ABB-yhteistyörobottia.

Kun TurtleBot on navigoinut onnistuneesti annettuun koordinaattipisteeseen, sovellus ilmoittaa socket-palvelimelle, että TurtleBot on saapunut ABB-yhteistyörobotin luokse ("setState,tb4,onABB"). Tässä kohtaa on tärkeää huomioida, että TurtleBotin paikannuksen tarkkuus on noin 10-15 cm, mikä tarkoittaa ettei TurtleBot pysähdy aina samaan kohtaan. Sovellus jää kysymään while-silmukassa ABB-yhteistyörobotin tilaa ("getState,abb") kerran sekunnissa, kunnes robotti ilmoittaa olevansa valmis.

Saatuaan tiedon TurtleBotin saapumista, ABB-yhteistyörobotti aloittaa oman työkiertonsa ("setState,abb,working") ja asettaa työkappaleen TurtleBotin päälle. Kun robotti on päästänyt työkappaleesta irti, se ilmoittaa socket-palvelimelle olevansa valmis ("setState,abb,ready").

Kun sovellus saa tiedon ABB-yhteistyörobotin työkierron valmistumisesta, se kertoo socket-palvelimelle TurtleBotin olevan jälleen liikkeessä ("setState,tb4,moving") ja käskee TurtleBottia navigoimaan UR5:n luokse. 

TurtleBot navigoi UR5-yhteistyörobotin luokse kantaen samalla sen päälle asetettua työkappaletta. Huomaa, että tässä sovelluksessa UR5-robotin koordinaatit on kovakoodattu. Kun TurtleBot on navigoinut onnistuneesti kohteeseen, sovellus ilmoittaa socket-palvelimelle, että TurtleBot on saapunut UR5-yhteistyörobotin luokse ("setState,tb4,onUR5")

Sovellus jää jälleen while-silmukassa kysymään UR5-yhteistyörobotin tilaa ("getState,ur5") kerran sekunnissa, kunnes UR5 ilmoittaa olevansa valmis.

Saatuaan tiedon TurtleBotin saapumista, UR5-yhteistyörobotti aloittaa oman työkiertonsa ("setState,ur5,working") ja poimii konenäköä hyödyntäen työkappaleen TurtleBotin päältä. Kun robotti on poiminut työkappaleen, se ilmoittaa socket-palvelimelle olevansa valmis ("setState,ur5,ready").

Kun sovellus saa tiedon UR5-yhteistyörobotin työkierron valmistumisesta, se kertoo socket-palvelimelle TurtleBotin olevan jälleen kerran liikkeessä ("setState,tb4,moving"). Lopuksi sovellus käskee TurtleBottia navigoimaan kohti latausasemaa ja telakoitumaan siihen.

TurtleBotin suurin piirtein kulkema reitti on esitetty punaisella katkoviivalla alla olevassa kuvassa.

![labra](/images/labra.png)

## Sovelluksen ajaminen

Ennen kuin itse *na2pose*-sovellus käynnistetään, on käynnistettävä paikannus, Nav2-navigointijärjestelmä, ja tietenkin socket-palvelin.

Paikannus hyödyntää karttaa tilasta, jossa TurtleBot liikkuu. Käytännössä paikannukselle annetaan parametrina YAML-tiedosto, jossa on muun muassa tiedot kartan tiedostonimestä, resoluutiosta, ja nollapisteestä.

Jos karttaa ei ole saatavilla, se voidaan luoda SLAM-algoritmilla TurtleBotin kotisivuilla olevien ohjeiden mukaisesti:

[https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html)

```
ros2 launch turtlebot4_navigation localization.launch.py map:=robolabra.yaml
```

Nav2-navigointijärjestelmä käynnistetään toisessa terminaalissa komennolla:
```
ros2 launch turtlebot4_navigation nav2.launch.py
```

Socket-palvelin käynnistetään kolmannessa terminaalissa komennolla
```
python3 sockserver.py
```

Sovellus käynnistetään neljännessä terminaalissa alla olevilla komennoilla.
```
$ cd myROS2workspace
$ source install/local_setup.bash
$ ros2 run nav2pose start_nav
```

Sovelluksen voi sammuttaa painamalla Ctrl+c. Mieluiten silloin, kun TurtleBot on kiinni latausasemassa.

## Tekijätiedot

Hannu Hakalahti, Asiantuntija TKI, Seinäjoen ammattikorkeakoulu

## Hanketiedot

* Hankkeen nimi: Autonomiset ajoneuvot esiselvityshanke
* Rahoittaja: Töysän säästöpankkisäätiön tutkimusrahasto
* Aikataulu: 01.08.2023 - 31.06.2024
---
![rahoittajan_logo](/images/toysan_sp_saatio.jpg)

![seamk_logo](/images/SEAMK.jpg)