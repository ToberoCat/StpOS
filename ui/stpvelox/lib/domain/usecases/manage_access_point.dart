import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/wifi_band.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class ManageAccessPoint {
  final IWifiRepository repository;

  ManageAccessPoint(this.repository);

  Future<void> startAccessPoint(AccessPointConfig config) async {
    await repository.startAccessPoint(config);
  }

  Future<void> stopAccessPoint() async {
    await repository.stopAccessPoint();
  }

  Future<bool> isAccessPointActive() async {
    return await repository.isAccessPointActive();
  }

  Future<AccessPointConfig?> getAccessPointConfig() async {
    return await repository.getAccessPointConfig();
  }

  Future<WifiBand> findBestWifiBand() async {
    return await repository.findBestWifiBand();
  }

  Future<int> findBestChannel(WifiBand band) async {
    return await repository.findBestChannel(band);
  }
}