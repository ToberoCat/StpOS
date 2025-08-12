import 'package:stpvelox/domain/entities/wifi_network.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class GetAvailableNetworks {
  final IWifiRepository repository;

  GetAvailableNetworks({required this.repository});

  Future<List<WifiNetwork>> call() {
    return repository.getAvailableNetworks();
  }
}
