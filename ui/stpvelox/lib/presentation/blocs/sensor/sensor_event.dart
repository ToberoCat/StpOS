part of 'sensor_bloc.dart';

abstract class SensorEvent extends Equatable {
  const SensorEvent();

  @override
  List<Object> get props => [];
}

class LoadSensorsEvent extends SensorEvent {}