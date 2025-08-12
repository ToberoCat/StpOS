import 'package:pty/pty.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:xterm/xterm.dart';

class ProgramLifecycleManager {
  ProgramSession? session;

  ProgramSession startProgram(Program program, Map<String, String> args) {
    session = ProgramSession(program, args);
    return session!;
  }

  Future<int> stopProgram() => session?.kill() ?? Future.value(-1);
}

class ProgramSession {
  late Terminal terminal;
  late TerminalController terminalController;
  late PseudoTerminal pty;
  bool _isRunning = false;

  ProgramSession(Program program, Map<String, String> args) {
    terminal = Terminal(
      onOutput: (data) {
        pty.write(data);
      },
      onResize: (width, height, pixelWidth, pixelHeight) {
        pty.resize(height, width);
      },
    );
    terminalController = TerminalController();
    pty = PseudoTerminal.start(
      "bash",
      [],
      environment: {
        "TERM": "xterm-256color",
      },
    );
    pty.resize(800, 480);

    pty.exitCode.then((exitCode) {
      terminal.write("Process finished with exit code $exitCode");
      _isRunning = false;
    });

    pty.out.listen((event) => terminal.write(event));
    pty.write("trap 'kill 0' EXIT; cd ${program.parentDir} && bash ${program.runScript} ${args.entries.map((e) => pairToString(e.key, e.value)).join(' ')}");
    _isRunning = true;
  }

  Future<int> kill() async {
    if (!isRunning) return -1;

    terminal.write("\r\n^C\r\n");
    pty.write("\x03");
    pty.kill();
    _isRunning = false;
    return pty.exitCode;
  }

  String pairToString(String key, String value) {
    return "--$key=$value";
  }

  get isRunning => _isRunning;
}
