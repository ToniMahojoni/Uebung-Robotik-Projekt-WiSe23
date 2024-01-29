# Uebung-Robotik-Projekt-WiSe23

Repository für die Übungensaufgaben im Robotik-Projekt des WiSe23/24.

## Aufgabe 3: Publisher/Subscriber

Im Package `timing_tubaf_cpp` wurde ein Publisher in C++ geschrieben, welcher jede Sekunde einen inkrementierenden Integer-Wert (beginnend bei 0) auf dem topic "number" published und in der Konsole "Integer Value: *WERT*" mit dem aktuellen Wert ausgiebt.
Im Package `timing_tubaf_py` wurde ein Publisher/Subscriber in Python geschrieben, welcher zum einen auf das topic `number` subscribed und den Integer-Wert der Nachricht mit "I heard: *WERT*" auf der Konsole ausgiebt und zum anderen die zeitliche Differenz zwischen der aktuellen und letzten empfangenen Nachricht berechnet und diese mit "Zeitdifferenz: *ZEIT*" auf dem topic `diff` published.

Die Software kann durch folgende Schritte ausgeführt werden:
1. Ein Terminal im Root-Verzeichnis des Ros-workspace öffnen, in welchen das Repo geklont wurde und den Befehl `source install/setup.bash` ausführen, damit die Packages gefunden werden können. Um den C++ Publisher zu starten, muss der Befehl `ros2 run timing_tubaf_cpp talker` ausgeführt werden. Auf der Konsole erscheinen nun die Nachrichten
2. Ein weiteres Terminal im Root-Verzeichnis öffnen, wieder `source install/setup.bash` ausführen und für den Python Publisher/Subscriber `ros2 run timing_tubaf_py listener` eingeben. Der Subscriber sollte nun die Nachrichten des Publishers empfangen und diese auf der Konsole wiedergeben.
3. Zur Kontrolle, dass der Python Publisher die Zeitdifferenz der Nachrichten wirklich auf dem "diff" topic published, kann man ein weiteres Terminal im Root-Verzeichnis öffnen und mit `ros2 topic echo diff` die Funktionalität des Python Publishers prüfen.

Die Software wurde in ROS 2 Humble auf Ubuntu 22.04.3 LTS geschrieben.
Weitere ROS 2 Bibliotheken, die benötigt werden, sind `rclpy`, `rclcpp` und `std_msgs`. Desweiteren wird die Python-Bibliothek `datetime` für die Zeitberechnungen benötigt.

Der `src`-Ordner beinhaltet grundsätzlich alle ROS 2 Packages der Software.
Im Ordner `timing_tubaf_cpp` befindet sich das Package für den C++ Publisher, welches unter anderem die `CMakeLists.txt`, `package.xml` und im weiteren untergeordneten `src`-Ordner die `publisher_member_function.cpp` enthält. In der `CMakeLists.txt` und `package.xml` Datei sind die Referenzen für die genutzen Bibliotheken und Infos zum Maintainer hinterlegt, währenddessen der eigentliche 
C++-Code des Publishers in der `publisher_member_function.cpp` zu finden ist.
Im Ordner `timing_tubaf_py` befindet sich die `package.xml` und `setup.py` Datei, welche ebenfalls Referenzen der Bibliotheken und Infos zum Maintainer enthalten. Im weitren Ordner `timing_tubaf_py` innerhalb des `timing_tubaf_py` Ordners befindet sich die `subscriber_member_function.py`, welche den eigentlichen Python-Code des Publisher/Subscribers beinhaltet.

## Aufgabe 4: Hindernisverfolgung per Laserscanner

Im Package `laserscan` wurde ein Node in Python geschrieben, welcher einen TurtleBot so steuert, dass er Hindernissen hinterherfährt und dabei das Hindernis, welches im Scan-Bereich am nähsten ist, priorisiert. Gerät der Roboter zu nah an das Hindernis oder liegt das Objekt zu weit entfernt, dann stoppt der Roboter. Die Distanzen, bei welchen der Roboter zum stehen kommt, die Geschwindigkeit und der Öffnungswinkel des Scan-Bereichs sind per Parameter konfigurierbar.

Die Software kann durch folgendes Vorgehen ausgeführt werden:
Zuerst den Turtlebot starten und über die topic list überprüfen, ob dieser richtig verbunden ist. Ein Terminal im Root-Verzeichnis des Ros-workspace öffnen, in welchen das Repo geklont wurde und den Befehl `source install/setup.bash` ausführen, damit das Package gefunden werden kann. Danach den Node mit dem Befehl `ros2 run laserscan laserscan_chase` starten. Bei erfolgreichen Programmstart sollte auf der Konsole die Nachricht `Program start.` erscheinen.

Die Software wurde in ROS 2 Humble auf Ubuntu 22.04.3 LTS geschrieben.
Weitere ROS 2 Bibliotheken, die benötigt werden, sind `rclpy` und `std_msgs`, `sensor_msgs`, `geometry_msgs` für die Daten der Nachrichten im Terminal. Für die Berechnungen mit Arrays wird zusätzlich noch `numpy` genutzt.

Der `src`-Ordner beinhaltet grundsätzlich alle ROS 2 Packages der Software.
Im Ordner `laserscan` befindet sich das Package für die Laserscan-Software, welches unter anderen die `setup.py`, `package.xml` und im weiteren `laserscan`-Ordner die `obstacle_chasing.py` enthält. In der `setup.py` und `package.xml` Datei sind die Referenzen für die genutzen Bibliotheken und Infos zum Maintainer hinterlegt, währenddessen der eigentliche 
Python-Code der Software in der `obstacle_chasing.py` zu finden ist.

## Aufgabe 5: Linienverfolgung per Kamera

Im Package `line_following` wurde ein Node in Python geschrieben, welcher einen TurtleBot so steuert, dass er einer weißen Linie auf schwarzem Untergrund hinterherfährt. Zu Beginn muss der Roboter dazu mittig auf der weißen Linie stehen. Per Parameter sind einerseits die linke und rechte Bildgrenze einstellbar, welche zwischen 0 und 319 liegen müssen und andererseits die Breite des Streifens in der Mitte des Bildes, in dem sich die Linie befinden soll. Befindet sich die Linie außerhalb des Streifens und innerhalb der Bildgrenzen, so lenkt der Roboter in die entsprechende Richtung, bis sich die Linie wieder im mittleren Streifen befindet. Zuletzt sind auch die Fahrgeschwindigkeit und die Drehgeschwindigkeit des TurtleBots einstellbar.

Die Software kann durch folgendes Vorgehen ausgeführt werden:
Zuerst den Turtlebot starten und über die topic list überprüfen, ob dieser richtig verbunden ist. Ein Terminal im Root-Verzeichnis des Ros-workspace öffnen, in welchen das Repo geklont wurde und den Befehl `source install/setup.bash` ausführen, damit das Package gefunden werden kann. Danach den Node mit dem Befehl `ros2 run line_following line_chase` starten. Bei erfolgreichen Programmstart sollte sich ein Fenster öffnen, in dem die unterste Bildzeile der Kamera anzeigt wird. Weiterhin werden in der Konsole Statusmeldungen über die Position der Linie angezeigt.

Die Software wurde in ROS 2 Humble auf Ubuntu 22.04.3 LTS geschrieben.
Weitere ROS 2 Bibliotheken, die benötigt werden, sind `rclpy` und `std_msgs`, `sensor_msgs`, `geometry_msgs` für die Daten der Nachrichten im Terminal. Für die Verarbeitung des Kamerabildes werden `cv2` und `cv_bridge` benötigt. Für die Berechnungen mit Arrays wird zusätzlich noch `numpy` genutzt.

Der `src`-Ordner beinhaltet grundsätzlich alle ROS 2 Packages der Software.
Im Ordner `line_following` befindet sich das Package für die Linienverfolgungs-Software, welches unter anderen die `setup.py`, `package.xml` und im weiteren `line_following`-Ordner die `line_chasing.py` enthält. In der `setup.py` und `package.xml` Datei sind die Referenzen für die genutzen Bibliotheken und Infos zum Maintainer hinterlegt, währenddessen der eigentliche 
Python-Code der Software in der `line_chasing.py` zu finden ist.