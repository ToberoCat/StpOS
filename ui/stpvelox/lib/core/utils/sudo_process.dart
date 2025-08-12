import 'dart:io';

class SudoProcess {
  static Future<ProcessResult> run(String command, List<String> args) async {
    final result = await Process.run('sudo', [command, ...args]);
    stdout.write(result.stdout);
    stderr.write(result.stderr);
    return result;
  }

  static Future<ProcessResult> runWith(List<String> command) async {
    final result = await Process.run('sudo', command);
    stdout.write(result.stdout);
    stderr.write(result.stderr);
    return result;
  }
}
