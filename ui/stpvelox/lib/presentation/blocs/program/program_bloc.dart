import 'package:bloc/bloc.dart';
import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:stpvelox/domain/service/program_lifecycle_manager.dart';
import 'package:stpvelox/domain/usecases/reboot.dart';
import 'package:stpvelox/domain/usecases/start_program.dart';

part 'program_event.dart';

part 'program_state.dart';

class ProgramBloc extends Bloc<ProgramEvent, ProgramState> {
  final StartProgram startProgram;
  final RebootDevice rebootDevice;

  ProgramBloc({required this.startProgram, required this.rebootDevice})
      : super(ProgramStopped()) {
    on<StartProgramEvent>((event, emit) async {
      var session = startProgram.call(event.program, event.args);

      emit(ProgramStarted(session: session));
    });

    on<StopProgramEvent>((event, emit) {
      var session = (state as ProgramStarted).session;
      session.kill().then((_) => emit(ProgramStopped()));
    });

    on<RebootEvent>((event, emit) {
      rebootDevice.call();
    });
  }
}
