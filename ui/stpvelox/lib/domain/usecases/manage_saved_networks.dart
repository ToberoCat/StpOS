import 'package:stpvelox/domain/entities/saved_network.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class ManageSavedNetworks {
  final IWifiRepository repository;

  ManageSavedNetworks(this.repository);

  Future<List<SavedNetwork>> getSavedNetworks() async {
    return await repository.getSavedNetworks();
  }

  Future<void> saveNetwork(SavedNetwork network) async {
    await repository.saveNetwork(network);
  }

  Future<void> removeSavedNetwork(String ssid) async {
    await repository.removeSavedNetwork(ssid);
  }

  Future<SavedNetwork?> getSavedNetwork(String ssid) async {
    return await repository.getSavedNetwork(ssid);
  }
}