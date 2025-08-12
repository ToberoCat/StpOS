part of 'program_bloc.dart';

abstract class ProgramEvent extends Equatable {
  const ProgramEvent();

  @override
  List<Object> get props => [];
}

class StartProgramEvent extends ProgramEvent {
  final Program program;
  final Map<String, String> args;

  const StartProgramEvent({required this.program, required this.args});

  @override
  List<Object> get props => [program, args];
}

class StopProgramEvent extends ProgramEvent {}

class RebootEvent extends ProgramEvent {}