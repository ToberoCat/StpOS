import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/entities/saved_network.dart';
import 'package:stpvelox/domain/usecases/connect_to_wifi.dart';
import 'package:stpvelox/domain/usecases/forget_wifi.dart';
import 'package:stpvelox/domain/usecases/get_available_networks.dart';
import 'package:stpvelox/domain/usecases/get_device_info.dart';
import 'package:stpvelox/domain/usecases/get_network_mode.dart';
import 'package:stpvelox/domain/usecases/manage_access_point.dart';
import 'package:stpvelox/domain/usecases/manage_lan_only_mode.dart';
import 'package:stpvelox/domain/usecases/manage_saved_networks.dart';
import 'package:stpvelox/domain/usecases/set_network_mode.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';

class WifiBloc extends Bloc<WifiEvent, WifiState> {
  final GetAvailableNetworks getAvailableNetworks;
  final ConnectToWifi connectToWifi;
  final ForgetWifi forgetWifi;
  final GetDeviceInfo getDeviceInfo;
  final GetNetworkMode getNetworkMode;
  final SetNetworkMode setNetworkMode;
  final ManageAccessPoint manageAccessPoint;
  final ManageSavedNetworks manageSavedNetworks;
  final ManageLanOnlyMode manageLanOnlyMode;

  WifiBloc({
    required this.getAvailableNetworks,
    required this.connectToWifi,
    required this.forgetWifi,
    required this.getDeviceInfo,
    required this.getNetworkMode,
    required this.setNetworkMode,
    required this.manageAccessPoint,
    required this.manageSavedNetworks,
    required this.manageLanOnlyMode,
  }) : super(WifiInitialState()) {
    on<LoadNetworksEvent>(_onLoadNetworks);
    on<ConnectToNetworkEvent>(_onConnectToNetwork);
    on<ForgetNetworkEvent>(_onForgetNetwork);
    on<LoadDeviceInfoEvent>(_onLoadDeviceInfo);
    
        on<LoadNetworkModeEvent>(_onLoadNetworkMode);
    on<SetNetworkModeEvent>(_onSetNetworkMode);
    
        on<StartAccessPointEvent>(_onStartAccessPoint);
    on<StopAccessPointEvent>(_onStopAccessPoint);
    on<LoadAccessPointConfigEvent>(_onLoadAccessPointConfig);
    on<StartAccessPointWithLastConfigEvent>(_onStartAccessPointWithLastConfig);
    
        on<LoadSavedNetworksEvent>(_onLoadSavedNetworks);
    on<RemoveSavedNetworkEvent>(_onRemoveSavedNetwork);
    on<ConnectToSavedNetworkEvent>(_onConnectToSavedNetwork);
    
        on<EnableLanOnlyModeEvent>(_onEnableLanOnlyMode);
    on<DisableLanOnlyModeEvent>(_onDisableLanOnlyMode);
  }

  Future<void> _onLoadNetworks(LoadNetworksEvent event, Emitter<WifiState> emit) async {
    emit(WifiLoadingState());
    try {
      final networks = await getAvailableNetworks();
      emit(WifiLoadedState(networks));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onConnectToNetwork(ConnectToNetworkEvent event, Emitter<WifiState> emit) async {
    emit(WifiConnectingState());
    try {
      await connectToWifi(event.ssid, event.encryptionType, event.credentials);
      
            final savedNetwork = SavedNetwork(
        ssid: event.ssid,
        encryptionType: event.encryptionType,
        credentials: event.credentials,
        lastConnected: DateTime.now(),
      );
      await manageSavedNetworks.saveNetwork(savedNetwork);
      
      emit(WifiConnectedState(event.ssid));
            final networks = await getAvailableNetworks();
      emit(WifiLoadedState(networks));
      add(LoadDeviceInfoEvent());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onForgetNetwork(ForgetNetworkEvent event, Emitter<WifiState> emit) async {
    emit(WifiForgettingState());
    try {
      await forgetWifi(event.ssid);
      emit(WifiForgottenState(event.ssid));
            final networks = await getAvailableNetworks();
      emit(WifiLoadedState(networks));
      add(LoadDeviceInfoEvent());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onLoadDeviceInfo(LoadDeviceInfoEvent event, Emitter<WifiState> emit) async {
    try {
      final deviceInfo = await getDeviceInfo();
      emit(DeviceInfoLoadedState(deviceInfo));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

    Future<void> _onLoadNetworkMode(LoadNetworkModeEvent event, Emitter<WifiState> emit) async {
    try {
      final mode = await getNetworkMode();
      emit(NetworkModeLoadedState(mode));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onSetNetworkMode(SetNetworkModeEvent event, Emitter<WifiState> emit) async {
    emit(NetworkModeChangingState());
    try {
            final currentMode = await getNetworkMode();
      if (currentMode == NetworkMode.accessPoint && event.mode != NetworkMode.accessPoint) {
        await manageAccessPoint.stopAccessPoint();
      } else if (currentMode == NetworkMode.lanOnly && event.mode != NetworkMode.lanOnly) {
        await manageLanOnlyMode.disableLanOnlyMode();
      }
      
            await setNetworkMode(event.mode);
      emit(NetworkModeChangedState(event.mode));
      
            emit(NetworkModeLoadedState(event.mode));
      
    } catch (e) {
      emit(WifiErrorState(e.toString()));
            try {
        final actualMode = await getNetworkMode();
        emit(NetworkModeLoadedState(actualMode));
      } catch (reloadError) {
              }
    }
  }

    Future<void> _onStartAccessPoint(StartAccessPointEvent event, Emitter<WifiState> emit) async {
    emit(AccessPointStartingState());
    try {
      await manageAccessPoint.startAccessPoint(event.config);
      emit(AccessPointStartedState(event.config));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onStopAccessPoint(StopAccessPointEvent event, Emitter<WifiState> emit) async {
    emit(AccessPointStoppingState());
    try {
      await manageAccessPoint.stopAccessPoint();
      emit(AccessPointStoppedState());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onLoadAccessPointConfig(LoadAccessPointConfigEvent event, Emitter<WifiState> emit) async {
    try {
      final config = await manageAccessPoint.getAccessPointConfig();
      emit(AccessPointConfigLoadedState(config));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onStartAccessPointWithLastConfig(StartAccessPointWithLastConfigEvent event, Emitter<WifiState> emit) async {
    emit(AccessPointStartingState());
    try {
      final config = await manageAccessPoint.getAccessPointConfig();
      if (config != null) {
        await manageAccessPoint.startAccessPoint(config);
        emit(AccessPointStartedState(config));
      } else {
                final defaultBand = await manageAccessPoint.findBestWifiBand();
        final defaultConfig = AccessPointConfig(
          ssid: 'STP-Velox-Robot',
          password: 'Robot123!',
          band: defaultBand,
        );
        await manageAccessPoint.startAccessPoint(defaultConfig);
        emit(AccessPointStartedState(defaultConfig));
      }
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

    Future<void> _onLoadSavedNetworks(LoadSavedNetworksEvent event, Emitter<WifiState> emit) async {
    emit(WifiLoadingState());
    try {
      final networks = await manageSavedNetworks.getSavedNetworks();
      emit(SavedNetworksLoadedState(networks));
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onRemoveSavedNetwork(RemoveSavedNetworkEvent event, Emitter<WifiState> emit) async {
    try {
      await manageSavedNetworks.removeSavedNetwork(event.ssid);
      emit(SavedNetworkRemovedState(event.ssid));
            add(LoadSavedNetworksEvent());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onConnectToSavedNetwork(ConnectToSavedNetworkEvent event, Emitter<WifiState> emit) async {
    emit(WifiConnectingState());
    try {
      final savedNetwork = await manageSavedNetworks.getSavedNetwork(event.ssid);
      if (savedNetwork != null) {
        await connectToWifi(savedNetwork.ssid, savedNetwork.encryptionType, savedNetwork.credentials);
        
                final updatedNetwork = savedNetwork.copyWith(lastConnected: DateTime.now());
        await manageSavedNetworks.saveNetwork(updatedNetwork);
        
        emit(WifiConnectedState(event.ssid));
        add(LoadDeviceInfoEvent());
      } else {
        emit(WifiErrorState('Saved network not found'));
      }
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

    Future<void> _onEnableLanOnlyMode(EnableLanOnlyModeEvent event, Emitter<WifiState> emit) async {
    emit(LanOnlyModeTogglingState());
    try {
      await manageLanOnlyMode.enableLanOnlyMode();
      emit(LanOnlyModeEnabledState());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }

  Future<void> _onDisableLanOnlyMode(DisableLanOnlyModeEvent event, Emitter<WifiState> emit) async {
    emit(LanOnlyModeTogglingState());
    try {
      await manageLanOnlyMode.disableLanOnlyMode();
      emit(LanOnlyModeDisabledState());
    } catch (e) {
      emit(WifiErrorState(e.toString()));
    }
  }
}