import 'dart:convert';
import 'dart:io';

import 'package:stpvelox/domain/entities/args/arg.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:path/path.dart' as path;
import 'dart:developer' as developer;

abstract class ProgramRemoteDataSource {
  Future<List<String>> executeProgram(String arg);

  Future<List<Program>> getPrograms();
}

class ProgramRemoteDataSourceImpl implements ProgramRemoteDataSource {
  final String programsDirectoryPath;

  ProgramRemoteDataSourceImpl({required this.programsDirectoryPath});

  @override
  Future<List<String>> executeProgram(String arg) async {
    if (arg.isEmpty) {
      throw Exception("No argument provided.");
    }
    return [
      "Program '$arg' started...",
      "Loading data...",
      "Error: Unable to reach server.",
    ];
  }

  @override
  Future<List<Program>> getPrograms() async {
    final programsDir = Directory(programsDirectoryPath);

    if (!await programsDir.exists()) {
      await programsDir.create();
      return [];
    }

    final List<Directory> projectDirs = await programsDir
        .list()
        .where((entity) => entity is Directory)
        .cast<Directory>()
        .toList();

    List<Program> programs = [];

    for (var dir in projectDirs) {
      final folderName = path.basename(dir.path);
      final projectJsonFile = File(path.join(dir.path, 'project.json'));

      String name = folderName;
      String runScript = 'run.sh';
      List<Arg> args = [];

      if (await projectJsonFile.exists()) {
        try {
          final jsonContent = await projectJsonFile.readAsString();
          final Map<String, dynamic> jsonData = jsonDecode(jsonContent);

          if (jsonData.containsKey('name') && jsonData['name'] is String) {
            name = jsonData['name'];
          }

          if (jsonData.containsKey('run_script') &&
              jsonData['run_script'] is String &&
              (jsonData['run_script'] as String).trim().isNotEmpty) {
            runScript = jsonData['run_script'];
          } else {
            runScript = 'sh ./run.sh';
          }

          if (jsonData.containsKey('args') && jsonData['args'] is List) {
            args = (jsonData['args'] as List)
                .map((argJson) => Arg.fromJson(argJson))
                .toList();
          }
        } catch (e) {
          developer.log(
              'Error reading or parsing project.json in ${dir.path}: $e. Using defaults.',
              name: 'ProgramRemoteDataSourceImpl');
        }
      }

      final runShFile = File(path.join(dir.path, runScript));
      if (!await runShFile.exists()) {
        developer.log(
            'Warning: run.sh does not exist in ${dir.path}. The run_script may fail.',
            name: 'ProgramRemoteDataSourceImpl');
      }

      final program = Program(
        name: name,
        parentDir: dir.path,
        runScript: runScript,
        args: args,
      );

      programs.add(program);
    }

    return programs;
  }
}
