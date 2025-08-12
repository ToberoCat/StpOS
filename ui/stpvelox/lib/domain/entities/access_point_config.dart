import 'package:stpvelox/domain/entities/wifi_band.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';

class AccessPointConfig {
  final String ssid;
  final String password;
  final WifiBand band;
  final int channel;
  final WifiEncryptionType encryptionType;
  final bool hidden;
  final int maxClients;

  const AccessPointConfig({
    required this.ssid,
    required this.password,
    this.band = WifiBand.bandAuto,
    this.channel = 0,     this.encryptionType = WifiEncryptionType.wpa3Personal,
    this.hidden = false,
    this.maxClients = 8,
  });

  AccessPointConfig copyWith({
    String? ssid,
    String? password,
    WifiBand? band,
    int? channel,
    WifiEncryptionType? encryptionType,
    bool? hidden,
    int? maxClients,
  }) {
    return AccessPointConfig(
      ssid: ssid ?? this.ssid,
      password: password ?? this.password,
      band: band ?? this.band,
      channel: channel ?? this.channel,
      encryptionType: encryptionType ?? this.encryptionType,
      hidden: hidden ?? this.hidden,
      maxClients: maxClients ?? this.maxClients,
    );
  }
}