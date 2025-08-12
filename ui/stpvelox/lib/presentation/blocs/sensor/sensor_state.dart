part of 'sensor_bloc.dart';

abstract class SensorState extends Equatable {
  const SensorState();

  @override
  List<Object> get props => [];
}

class SensorInitial extends SensorState {}

class SensorLoading extends SensorState {}

class SensorLoaded extends SensorState {
  final List<Sensor> sensors;

  const SensorLoaded({required this.sensors});

  @override
  List<Object> get props => [sensors];
}

class SensorError extends SensorState {
  final String message;

  const SensorError({required this.message});

  @override
  List<Object> get props => [message];
}
