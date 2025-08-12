part of 'program_bloc.dart';

abstract class ProgramState extends Equatable {
  @override
  List<Object?> get props => [];
}

class ProgramStarted extends ProgramState {
  final ProgramSession session;

  ProgramStarted({required this.session});

  @override
  List<Object> get props => [session];
}

class ProgramStopped extends ProgramState {}