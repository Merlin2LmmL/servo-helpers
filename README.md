# **servo\_helpers**

**Servo Relay Node for ROS 2**

Dieses Repository enthält einen ROS 2–Knoten, der Servo-Befehle von einem Topic empfängt und über einen Service an die Studica-Hardware weiterleitet. Damit lässt sich die normalerweise service-basierte Servo-Ansteuerung in ein Topic-basiertes Paradigma überführen und in Netzwerkanwendungen (z. B. Flask-basierten Steuerungs-UIs) integrieren.

---

## Inhalt

* [`servo_helpers/servo_relay_node.py`](#servo_helpersservo_relay_nodepy)
* [Funktionsweise](#funktionsweise)
* [Installation](#installation)
* [Parameter](#parameter)
* [Starten des Knotens](#starten-des-knotens)
* [Integration mit FlaskRobotControl](#integration-mit-flaskrobotcontrol)
* [Lizenz](#lizenz)

---

## Funktionsweise

1. **Topic Subscription**: Der Knoten lauscht auf `/<servo_name>/angle_cmd` (z. B. `/servo1/angle_cmd`) auf Nachrichten vom Typ `std_msgs/Int32`.
2. **Service-Aufruf**: Eingehende Winkelbefehle werden in einen `SetData`-Service-Request übersetzt und asynchron an `/<servo_name>/set_servo_angle` gesendet.
3. **Feedback-Logging**: Erfolgreiche oder fehlgeschlagene Service-Aufrufe werden über das ROS-Logger-Interface ausgegeben.

---

## Installation

1. **Klonen** dieses Repositories in dein ROS 2–Arbeitsverzeichnis:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Merlin2LmmL/servo-helpers.git
   ```

2. **Abhängigkeiten** installieren (z. B. aus `package.xml`):

   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build**:

   ```bash
   colcon build --packages-select servo_helpers
   source install/setup.bash
   ```

---

## Parameter

| Parameter    | Typ      | Default  | Beschreibung                             |
| ------------ | -------- | -------- | ---------------------------------------- |
| `servo_name` | `string` | `servo1` | Name des Servos (muss konfiguriert sein) |

---

## Starten des Knotens

```bash
ros2 run servo_helpers servo_relay_node --ros-args -p servo_name:=<dein_servo_name>
```

Beispiel:

```bash
ros2 run servo_helpers servo_relay_node --ros-args -p servo_name:=servo2
```

---

## Integration mit FlaskRobotControl

Dieses Paket bildet die Basis, um Servo-Befehle in Web-Interfaces verfügbar zu machen:

1. **Flask-Server**: Nutze [FlaskRobotControl](https://github.com/Merlin2LmmL/FlaskRobotControl), um eine Web-Oberfläche aufzusetzen.
2. **Topic Publisher**: Innerhalb der Flask-Anwendung publishst du auf `/<servo_name>/angle_cmd`.
3. **Netzwerksteuerung**: Dadurch können Servos lokal im Netzwerk vom Browser aus angesteuert werden.

---

## Lizenz

Dieses Projekt steht unter der MIT-Lizenz. Siehe [LICENSE](LICENSE) für Details.
