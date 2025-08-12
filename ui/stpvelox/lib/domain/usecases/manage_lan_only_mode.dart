import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class ManageLanOnlyMode {
  final IWifiRepository repository;

  ManageLanOnlyMode(this.repository);

  Future<void> enableLanOnlyMode() async {
    await repository.enableLanOnlyMode();
  }

  Future<void> disableLanOnlyMode() async {
    await repository.disableLanOnlyMode();
  }

  Future<bool> isLanOnlyModeActive() async {
    return await repository.isLanOnlyModeActive();
  }
}