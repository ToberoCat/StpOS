import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class ForgetWifi {
  final IWifiRepository repository;

  ForgetWifi({required this.repository});

  Future<void> call(String ssid) {
    return repository.forgetWifi(ssid);
  }
}
