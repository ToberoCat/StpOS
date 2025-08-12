import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class SetNetworkMode {
  final IWifiRepository repository;

  SetNetworkMode(this.repository);

  Future<void> call(NetworkMode mode) async {
    await repository.setNetworkMode(mode);
  }
}