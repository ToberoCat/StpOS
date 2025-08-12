import 'package:stpvelox/domain/entities/sensor.dart';

abstract class SensorRepository {
  Future<List<Sensor>> getSensors();
}