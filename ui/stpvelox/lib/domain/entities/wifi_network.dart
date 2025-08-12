import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';

class WifiNetwork {
  final String ssid;
  final bool isKnown;
  final bool isConnected;
  final WifiEncryptionType encryptionType;

  WifiNetwork({
    required this.ssid,
    required this.encryptionType,
    this.isKnown = false,
    this.isConnected = false,
  });
}