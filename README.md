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

## `servo_helpers/servo_relay_node.py`

Dieser Knoten:

1. Deklariert den Parameter `servo_name`, um verschiedene Servos anzusteuern.
2. Erstellt einen Service-Client für `/<servo_name>/set_servo_angle` (Typ `studica_control/srv/SetData`).
3. Abonniert das Topic `/<servo_name>/angle_cmd` (Typ `std_msgs/Int32`).
4. Leitet empfangene Winkelbefehle (`angle_cmd`) an den Service weiter.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from studica_control.srv import SetData

class ServoRelayNode(Node):
    def __init__(self):
        super().__init__('servo_relay_node')
        # Parameter: Name des Servos (muss mit Konfiguration übereinstimmen)
        self.declare_parameter('servo_name', 'servo1')
        self.servo_name = self.get_parameter('servo_name').get_parameter_value().string_value

        # Service-Client erstellen
        service_name = f'/{self.servo_name}/set_servo_angle'
        self.cli = self.create_client(SetData, service_name)
        self.get_logger().info(f'Waiting for service {service_name}...')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} not available, exiting')
            rclpy.shutdown()
            return
        self.get_logger().info(f'Connected to service {service_name}')

        # Topic-Abonnement für Winkelbefehle
        topic_name = f'/{self.servo_name}/angle_cmd'
        self.sub = self.create_subscription(
            Int32,
            topic_name,
            self.cmd_callback,
            10)
        self.get_logger().info(f'Subscribed to topic {topic_name}')

    def cmd_callback(self, msg: Int32):
        angle = msg.data
        self.get_logger().info(f'Received angle command: {angle}')
        req = SetData.Request()
        req.params = str(angle)
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Servo set: {resp.message}')
            else:
                self.get_logger().error(f'Service failed: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoRelayNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

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
