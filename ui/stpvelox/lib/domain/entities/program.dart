import 'package:stpvelox/domain/entities/args/arg.dart';

class Program {
  final String name;
  final String runScript;
  final String parentDir;
  final List<Arg> args;

  Program({
    required this.name,
    required this.runScript,
    required this.parentDir,
    required this.args,
  });
}
