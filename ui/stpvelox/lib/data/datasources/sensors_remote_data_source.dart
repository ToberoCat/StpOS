import 'package:stpvelox/data/native/kipr_plugin.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/entities/sensor_category.dart';
import 'package:stpvelox/presentation/screens/sensors/orientation_screen.dart';
import 'package:stpvelox/presentation/screens/sensors/sensor_graph_screen.dart';
import 'package:stpvelox/presentation/screens/sensors/sensor_motor_screen.dart';
import 'package:stpvelox/presentation/screens/sensors/sensor_servo_screen.dart';

abstract class SensorsRemoteDataSource {
  Future<List<Sensor>> fetchSensors();
}

class SensorsRemoteDataSourceImpl implements SensorsRemoteDataSource {
  Sensor getAnalogSensor(int port) {
    return Sensor(
        category: SensorCategory.analog,
        name: 'Analog $port',
        getSensorScreen: (sensor) => SensorGraphScreen(
            sensor: sensor,
            graphMin: 0,
            graphMax: 4095,
            getSensorValue: () =>
                KiprPlugin.getAnalog(port).then((value) => value.toDouble())));
  }

  Sensor getDigitalSensor(int port) {
    return Sensor(
        category: SensorCategory.digital,
        name: 'Digital $port',
        getSensorScreen: (sensor) => SensorGraphScreen(
            sensor: sensor,
            graphMin: 0,
            graphMax: 1,
            getSensorValue: () =>
                KiprPlugin.getDigital(port).then((value) => value.toDouble())));
  }

  Sensor getMotorSensor(int port) {
    return Sensor(
        category: SensorCategory.motor,
        name: 'Motor $port',
        getSensorScreen: (sensor) =>
            SensorMotorScreen(sensor: sensor, port: port));
  }

  Sensor getServoSensor(int port) {
    return Sensor(
        category: SensorCategory.servo,
        name: 'Servo $port',
        getSensorScreen: (sensor) =>
            SensorServoScreen(sensor: sensor, port: port));
  }

  @override
  Future<List<Sensor>> fetchSensors() async {
    return [
      getAnalogSensor(0),
      getAnalogSensor(1),
      getAnalogSensor(2),
      getAnalogSensor(3),
      getAnalogSensor(4),
      getAnalogSensor(5),
      getDigitalSensor(0),
      getDigitalSensor(1),
      getDigitalSensor(2),
      getDigitalSensor(3),
      getDigitalSensor(4),
      getDigitalSensor(5),
      getDigitalSensor(6),
      getDigitalSensor(7),
      getDigitalSensor(8),
      getDigitalSensor(9),
      getDigitalSensor(10),
      getMotorSensor(0),
      getMotorSensor(1),
      getMotorSensor(2),
      getMotorSensor(3),
      getServoSensor(0),
      getServoSensor(1),
      getServoSensor(2),
      getServoSensor(3),
      Sensor(
        category: SensorCategory.gyro,
        name: 'Gyro X',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -180,
          graphMax: 180,
          getSensorValue: () => KiprPlugin.getGyroX(),
        ),
      ),
      Sensor(
        category: SensorCategory.gyro,
        name: 'Gyro Y',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -180,
          graphMax: 180,
          getSensorValue: () => KiprPlugin.getGyroY(),
        ),
      ),
      Sensor(
        category: SensorCategory.gyro,
        name: 'Gyro Z',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -180,
          graphMax: 180,
          getSensorValue: () => KiprPlugin.getGyroZ(),
        ),
      ),
      Sensor(
        category: SensorCategory.accel,
        name: 'Accel X',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -10,
          graphMax: 10,
          getSensorValue: () => KiprPlugin.getAccelX(),
        ),
      ),
      Sensor(
        category: SensorCategory.accel,
        name: 'Accel Y',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -10,
          graphMax: 10,
          getSensorValue: () => KiprPlugin.getAccelY(),
        ),
      ),
      Sensor(
        category: SensorCategory.accel,
        name: 'Accel Z',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -10,
          graphMax: 10,
          getSensorValue: () => KiprPlugin.getAccelZ(),
        ),
      ),
      Sensor(
        category: SensorCategory.mag,
        name: 'Mag X',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -256,
          graphMax: 256,
          getSensorValue: () =>
              KiprPlugin.getMagX().then((value) => value.toDouble()),
        ),
      ),
      Sensor(
        category: SensorCategory.mag,
        name: 'Mag Y',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -256,
          graphMax: 256,
          getSensorValue: () =>
              KiprPlugin.getMagY().then((value) => value.toDouble()),
        ),
      ),
      Sensor(
        category: SensorCategory.mag,
        name: 'Mag Z',
        getSensorScreen: (sensor) => SensorGraphScreen(
          sensor: sensor,
          graphMin: -256,
          graphMax: 256,
          getSensorValue: () => KiprPlugin.getMagZ().then(
            (value) => value.toDouble(),
          ),
        ),
      ),
    ];
  }
}
