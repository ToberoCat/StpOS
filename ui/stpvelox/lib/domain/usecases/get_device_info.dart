import 'package:stpvelox/domain/entities/device_info.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

class GetDeviceInfo {
  final IWifiRepository repository;

  GetDeviceInfo({required this.repository});

  Future<DeviceInfo> call() {
    return repository.getDeviceInfo();
  }
}
