import 'package:stpvelox/data/datasources/linux_network_manager.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/device_info.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/entities/saved_network.dart';
import 'package:stpvelox/domain/entities/wifi_band.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';
import 'package:stpvelox/domain/repositories/i_wifi_repository.dart';

import '../../domain/entities/wifi_network.dart';

class WifiRepositoryImpl implements IWifiRepository {
  final LinuxNetworkManager networkManager;

  WifiRepositoryImpl({required this.networkManager});

  @override
  Future<List<WifiNetwork>> getAvailableNetworks() async {
    return await networkManager.scanNetworks();
  }

  @override
  Future<void> connectToWifi(String ssid, WifiEncryptionType encryptionType, WifiCredentials credentials) {
    return networkManager.connect(ssid, encryptionType, credentials);
  }

  @override
  Future<void> forgetWifi(String ssid) {
    return networkManager.forget(ssid);
  }

  @override
  Future<DeviceInfo> getDeviceInfo() async {
    try {
      return await networkManager.getDeviceInfo();
    } catch (e) {
      throw Exception('Failed to get device info: $e');
    }
  }

    @override
  Future<NetworkMode> getCurrentNetworkMode() async {
    return await networkManager.getCurrentNetworkMode();
  }

  @override
  Future<void> setNetworkMode(NetworkMode mode) async {
    await networkManager.setNetworkMode(mode);
  }

  @override
  Future<void> initializeNetworkMode() async {
    final currentMode = await networkManager.getCurrentNetworkMode();
    print('Initializing network mode: $currentMode');
    
        if (currentMode == NetworkMode.accessPoint) {
            final config = await networkManager.getAccessPointConfig();
      if (config != null) {
        try {
          await networkManager.startAccessPoint(config);
          print('Successfully auto-started AP with config: ${config.ssid}');
        } catch (e) {
          print('Failed to auto-start AP: $e - but keeping AP mode selection');
                  }
      } else {
        print('No AP config found - keeping AP mode selection');
              }
    } else if (currentMode == NetworkMode.lanOnly) {
            try {
        await networkManager.enableLanOnlyMode();
        print('Successfully enabled LAN only mode');
      } catch (e) {
        print('Failed to enable LAN only mode: $e - but keeping LAN mode selection');
              }
    } else {
            try {
                print('Client mode - ensuring WiFi is enabled');
      } catch (e) {
        print('Note: Issue preparing client mode: $e');
      }
    }
  }

    @override
  Future<void> startAccessPoint(AccessPointConfig config) async {
    await networkManager.startAccessPoint(config);
  }

  @override
  Future<void> stopAccessPoint() async {
    await networkManager.stopAccessPoint();
  }

  @override
  Future<bool> isAccessPointActive() async {
    return await networkManager.isAccessPointActive();
  }

  @override
  Future<AccessPointConfig?> getAccessPointConfig() async {
    return await networkManager.getAccessPointConfig();
  }

  @override
  Future<WifiBand> findBestWifiBand() async {
    return await networkManager.findBestWifiBand();
  }

  @override
  Future<int> findBestChannel(WifiBand band) async {
    return await networkManager.findBestChannel(band);
  }

    @override
  Future<List<SavedNetwork>> getSavedNetworks() async {
    return await networkManager.getSavedNetworks();
  }

  @override
  Future<void> saveNetwork(SavedNetwork network) async {
    await networkManager.saveNetwork(network);
  }

  @override
  Future<void> removeSavedNetwork(String ssid) async {
    await networkManager.removeSavedNetwork(ssid);
  }

  @override
  Future<SavedNetwork?> getSavedNetwork(String ssid) async {
    return await networkManager.getSavedNetwork(ssid);
  }

    @override
  Future<void> enableLanOnlyMode() async {
    await networkManager.enableLanOnlyMode();
  }

  @override
  Future<void> disableLanOnlyMode() async {
    await networkManager.disableLanOnlyMode();
  }

  @override
  Future<bool> isLanOnlyModeActive() async {
    return await networkManager.isLanOnlyModeActive();
  }
}
