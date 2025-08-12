import 'package:stpvelox/domain/entities/program.dart';

abstract class ProgramRepository {
  Future<List<String>> startProgram(String arg);
  Future<List<Program>> getPrograms();
}