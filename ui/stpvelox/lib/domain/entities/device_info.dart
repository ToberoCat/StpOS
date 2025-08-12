import 'package:stpvelox/domain/entities/wifi_network.dart';

class DeviceInfo {
  final String ipAddress;
  final WifiNetwork? connectedNetwork;

  DeviceInfo({required this.ipAddress, this.connectedNetwork});
}
