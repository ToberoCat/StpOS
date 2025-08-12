import 'package:stpvelox/core/utils/sudo_process.dart';

class RebootDevice {
  Future<void> call([bool isShutdown = false]) async {
    if (isShutdown) {
      await SudoProcess.run('shutdown', [' -h', 'now']);
    } else {
      await SudoProcess.run('reboot', []);
    }
  }
}