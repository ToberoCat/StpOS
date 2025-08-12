import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/repositories/sensor_repository.dart';

class GetSensors {
  final SensorRepository repository;

  GetSensors({required this.repository});

  Future<List<Sensor>> execute() async {
    return await repository.getSensors();
  }
}