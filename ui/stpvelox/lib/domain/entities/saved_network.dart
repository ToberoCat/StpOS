import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';

class SavedNetwork {
  final String ssid;
  final WifiEncryptionType encryptionType;
  final WifiCredentials credentials;
  final DateTime lastConnected;
  final bool autoConnect;

  const SavedNetwork({
    required this.ssid,
    required this.encryptionType,
    required this.credentials,
    required this.lastConnected,
    this.autoConnect = true,
  });

  SavedNetwork copyWith({
    String? ssid,
    WifiEncryptionType? encryptionType,
    WifiCredentials? credentials,
    DateTime? lastConnected,
    bool? autoConnect,
  }) {
    return SavedNetwork(
      ssid: ssid ?? this.ssid,
      encryptionType: encryptionType ?? this.encryptionType,
      credentials: credentials ?? this.credentials,
      lastConnected: lastConnected ?? this.lastConnected,
      autoConnect: autoConnect ?? this.autoConnect,
    );
  }
}