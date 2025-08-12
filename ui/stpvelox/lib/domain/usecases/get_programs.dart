import 'package:stpvelox/domain/entities/program.dart';
import 'package:stpvelox/domain/repositories/program_repository.dart';

class GetPrograms {
  final ProgramRepository repository;

  GetPrograms({required this.repository});

  Future<List<Program>> call() => repository.getPrograms();
}
