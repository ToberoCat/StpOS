import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/program.dart';

abstract class ProgramSelectionEvent extends Equatable {
  const ProgramSelectionEvent();

  @override
  List<Object?> get props => [];
}

class LoadProgramSelectionEvent extends ProgramSelectionEvent {}

class ProgramTappedEvent extends ProgramSelectionEvent {
  final Program program;

  const ProgramTappedEvent({required this.program});

  @override
  List<Object?> get props => [program];
}

class ProgramSelectionErrorEvent extends ProgramSelectionEvent {
  final String message;

  const ProgramSelectionErrorEvent({required this.message});

  @override
  List<Object?> get props => [message];
}
