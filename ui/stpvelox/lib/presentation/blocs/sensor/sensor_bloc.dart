import 'package:bloc/bloc.dart';
import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/usecases/get_sensors.dart';

part 'sensor_event.dart';
part 'sensor_state.dart';

class SensorBloc extends Bloc<SensorEvent, SensorState> {
  final GetSensors getSensors;

  SensorBloc({required this.getSensors}) : super(SensorInitial()) {
    on<LoadSensorsEvent>((event, emit) async {
      emit(SensorLoading());
      try {
        final sensors = await getSensors.execute();
        emit(SensorLoaded(sensors: sensors));
      } catch (e) {
        emit(SensorError(message: e.toString()));
      }
    });
  }
}
