import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/program.dart';

abstract class ProgramSelectionState extends Equatable {
  const ProgramSelectionState();

  @override
  List<Object?> get props => [];
}

class ProgramSelectionInitial extends ProgramSelectionState {}

class ProgramSelectionLoading extends ProgramSelectionState {}

class ProgramSelectionLoaded extends ProgramSelectionState {
  final List<Program> programs;

  const ProgramSelectionLoaded({required this.programs});

  @override
  List<Object?> get props => [programs];
}

class ProgramSelectionError extends ProgramSelectionState {
  final String message;

  const ProgramSelectionError({required this.message});

  @override
  List<Object?> get props => [message];
}

class ProgramTappedState extends ProgramSelectionState {
  final Program program;

  const ProgramTappedState({required this.program});

  @override
  List<Object?> get props => [program];
}
