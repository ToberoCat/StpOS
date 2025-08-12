import 'package:stpvelox/domain/service/program_lifecycle_manager.dart';

class StopProgram {
  final ProgramLifecycleManager programLifecycleManager;

  StopProgram({required this.programLifecycleManager});


  Future<int> call() async {
    return programLifecycleManager.stopProgram();
  }
}