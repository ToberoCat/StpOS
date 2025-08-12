import 'package:stpvelox/data/datasources/sensors_remote_data_source.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/repositories/sensor_repository.dart';

class SensorRepositoryImpl implements SensorRepository {
  final SensorsRemoteDataSource remoteDataSource;

  SensorRepositoryImpl({required this.remoteDataSource});

  @override
  Future<List<Sensor>> getSensors() async {
    return await remoteDataSource.fetchSensors();
  }
}