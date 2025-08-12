import 'package:stpvelox/data/datasources/program_remote_data_source.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:stpvelox/domain/repositories/program_repository.dart';

class ProgramRepositoryImpl implements ProgramRepository {
  final ProgramRemoteDataSource remoteDataSource;

  ProgramRepositoryImpl({required this.remoteDataSource});

  @override
  Future<List<String>> startProgram(String arg) async {
    return await remoteDataSource.executeProgram(arg);
  }

  @override
  Future<List<Program>> getPrograms() => remoteDataSource.getPrograms();
}
