import 'package:equatable/equatable.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/device_info.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/entities/saved_network.dart';
import 'package:stpvelox/domain/entities/wifi_network.dart';

abstract class WifiState extends Equatable {
  const WifiState();

  @override
  List<Object?> get props => [];
}

class WifiInitialState extends WifiState {}

class WifiLoadingState extends WifiState {}

class WifiLoadedState extends WifiState {
  final List<WifiNetwork> networks;

  const WifiLoadedState(this.networks);

  @override
  List<Object?> get props => [networks];
}

class WifiErrorState extends WifiState {
  final String message;

  const WifiErrorState(this.message);

  @override
  List<Object?> get props => [message];
}

class WifiConnectingState extends WifiState {}

class WifiConnectedState extends WifiState {
  final String ssid;

  const WifiConnectedState(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class WifiForgettingState extends WifiState {}

class WifiForgottenState extends WifiState {
  final String ssid;

  const WifiForgottenState(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class DeviceInfoLoadedState extends WifiState {
  final DeviceInfo deviceInfo;

  const DeviceInfoLoadedState(this.deviceInfo);

  @override
  List<Object?> get props => [deviceInfo];
}

class NetworkModeLoadedState extends WifiState {
  final NetworkMode mode;

  const NetworkModeLoadedState(this.mode);

  @override
  List<Object?> get props => [mode];
}

class NetworkModeChangingState extends WifiState {}

class NetworkModeChangedState extends WifiState {
  final NetworkMode mode;

  const NetworkModeChangedState(this.mode);

  @override
  List<Object?> get props => [mode];
}

class AccessPointStartingState extends WifiState {}

class AccessPointStartedState extends WifiState {
  final AccessPointConfig config;

  const AccessPointStartedState(this.config);

  @override
  List<Object?> get props => [config];
}

class AccessPointStoppingState extends WifiState {}

class AccessPointStoppedState extends WifiState {}

class AccessPointConfigLoadedState extends WifiState {
  final AccessPointConfig? config;

  const AccessPointConfigLoadedState(this.config);

  @override
  List<Object?> get props => [config];
}

class SavedNetworksLoadedState extends WifiState {
  final List<SavedNetwork> networks;

  const SavedNetworksLoadedState(this.networks);

  @override
  List<Object?> get props => [networks];
}

class SavedNetworkRemovedState extends WifiState {
  final String ssid;

  const SavedNetworkRemovedState(this.ssid);

  @override
  List<Object?> get props => [ssid];
}

class LanOnlyModeTogglingState extends WifiState {}

class LanOnlyModeEnabledState extends WifiState {}

class LanOnlyModeDisabledState extends WifiState {}
