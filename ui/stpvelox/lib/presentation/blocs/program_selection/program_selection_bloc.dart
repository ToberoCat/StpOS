import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/usecases/get_programs.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_event.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_state.dart';

class ProgramSelectionBloc extends Bloc<ProgramSelectionEvent, ProgramSelectionState> {
  final GetPrograms getPrograms;

  ProgramSelectionBloc({required this.getPrograms}) : super(ProgramSelectionInitial()) {
    on<LoadProgramSelectionEvent>((event, emit) async {
      emit(ProgramSelectionLoading());
      try {
        final programs = await getPrograms.call();
        emit(ProgramSelectionLoaded(programs: programs));
      } catch (e) {
        emit(ProgramSelectionError(message: e.toString()));
      }
    });
    on<ProgramTappedEvent>((event, emit) {
      emit(ProgramTappedState(program: event.program));
    });

  }
}