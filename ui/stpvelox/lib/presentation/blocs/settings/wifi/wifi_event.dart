import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';

abstract class WifiEvent extends Equatable {
  const WifiEvent();
  @override
  List<Object?> get props => [];
}

class LoadNetworksEvent extends WifiEvent {}

class ConnectToNetworkEvent extends WifiEvent {
  final String ssid;
  final WifiEncryptionType encryptionType;
  final WifiCredentials credentials;

  const ConnectToNetworkEvent(this.ssid, this.encryptionType, this.credentials);

  @override
  List<Object?> get props => [ssid, encryptionType, credentials];
}

class ForgetNetworkEvent extends WifiEvent {
  final String ssid;
  const ForgetNetworkEvent(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class LoadDeviceInfoEvent extends WifiEvent {}

class LoadNetworkModeEvent extends WifiEvent {}

class SetNetworkModeEvent extends WifiEvent {
  final NetworkMode mode;
  const SetNetworkModeEvent(this.mode);

  @override
  List<Object?> get props => [mode];
}

class StartAccessPointEvent extends WifiEvent {
  final AccessPointConfig config;
  const StartAccessPointEvent(this.config);

  @override
  List<Object?> get props => [config];
}

class StopAccessPointEvent extends WifiEvent {}

class LoadAccessPointConfigEvent extends WifiEvent {}

class StartAccessPointWithLastConfigEvent extends WifiEvent {}

class LoadSavedNetworksEvent extends WifiEvent {}

class RemoveSavedNetworkEvent extends WifiEvent {
  final String ssid;
  const RemoveSavedNetworkEvent(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class ConnectToSavedNetworkEvent extends WifiEvent {
  final String ssid;
  const ConnectToSavedNetworkEvent(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class EnableLanOnlyModeEvent extends WifiEvent {}

class DisableLanOnlyModeEvent extends WifiEvent {}
