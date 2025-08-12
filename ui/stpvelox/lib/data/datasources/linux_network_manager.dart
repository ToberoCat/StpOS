import 'dart:convert';

import 'package:shared_preferences/shared_preferences.dart';
import 'package:stpvelox/core/utils/sudo_process.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/device_info.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';
import 'package:stpvelox/domain/entities/saved_network.dart';
import 'package:stpvelox/domain/entities/wifi_band.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/domain/entities/wifi_network.dart';

import '../../domain/entities/wifi_encryption_type.dart';

class LinuxNetworkManager {
  Future<List<WifiNetwork>> scanNetworks() async {
        await _ensureWifiEnabled();
    
        await SudoProcess.run('nmcli', ['device', 'wifi', 'rescan']);
    
        await Future.delayed(const Duration(milliseconds: 1000));
    
        final result = await SudoProcess.run('nmcli', ['-f', 'SSID,SECURITY,IN-USE', 'dev', 'wifi']);
    if (result.exitCode != 0) {
      throw Exception('Failed to scan WiFi networks: ${result.stderr}');
    }

        final savedNetworks = await getSavedNetworks();
    final savedSSIDs = savedNetworks.map((n) => n.ssid).toSet();

                final lines = (result.stdout as String).split('\n').skip(1);
    final networks = <WifiNetwork>[];
    for (var line in lines) {
      if (line.trim().isEmpty) continue;
      final parts =
          line.split(RegExp(r'\s+')).where((p) => p.isNotEmpty).toList();
      if (parts.isEmpty) continue;
      final ssid = parts[0];
      final security = parts.length > 1 ? parts[1] : '';
      final inUse = line.contains('*');

      WifiEncryptionType encType = WifiEncryptionType.open;
      if (security.contains('WPA3')) {
        encType = security.contains('EAP')
            ? WifiEncryptionType.wpa3Enterprise
            : WifiEncryptionType.wpa3Personal;
      } else if (security.contains('WPA2')) {
        encType = security.contains('EAP')
            ? WifiEncryptionType.wpa2Enterprise
            : WifiEncryptionType.wpa2Personal;
      }

      networks.add(WifiNetwork(
        ssid: ssid,
        encryptionType: encType,
        isConnected: inUse,
        isKnown: savedSSIDs.contains(ssid),
      ));
    }
    return networks;
  }

  Future<void> connect(String ssid, WifiEncryptionType encType,
      WifiCredentials credentials) async {
    List<String> cmd = ['device', 'wifi', 'connect', ssid];
            switch (encType) {
      case WifiEncryptionType.open:
                break;
      case WifiEncryptionType.wpa2Personal:
      case WifiEncryptionType.wpa3Personal:
        final passCred = credentials as PersonalCredentials;
        cmd.addAll(['password', passCred.password]);
        break;
      case WifiEncryptionType.wpa2Enterprise:
      case WifiEncryptionType.wpa3Enterprise:
        final entCred = credentials as EnterpriseCredentials;
                                        await SudoProcess.run('nmcli', [
          'connection',
          'add',
          'type',
          'wifi',
          'con-name',
          ssid,
          'ifname',
          'wlan0',
          'ssid',
          ssid,
          'wifi-sec.key-mgmt',
          'wpa-eap',
          '802-1x.eap',
          'peap',
          '802-1x.identity',
          entCred.username,
          '802-1x.password',
          entCred.password,
          if (entCred.caCertificatePath != null) '802-1x.ca-cert' else '',
          if (entCred.caCertificatePath != null) entCred.caCertificatePath!,
        ]);
                await SudoProcess.run('nmcli', ['connection', 'up', ssid]);
        return;
    }

    final result = await SudoProcess.run('nmcli', cmd);
    if (result.exitCode != 0) {
      throw Exception('Failed to connect: ${result.stderr}');
    }
  }

  Future<void> forget(String ssid) async {
    final result = await SudoProcess.run('nmcli', ['connection', 'delete', ssid]);
    if (result.exitCode != 0) {
      throw Exception('Failed to forget network: ${result.stderr}');
    }
  }

  Future<DeviceInfo> getDeviceInfo() async {
    try {
            final ipResult = await SudoProcess.run('hostname', ['-I']);
      if (ipResult.exitCode != 0) {
        throw Exception('Failed to retrieve IP address: ${ipResult.stderr}');
      }
      final ipAddress = (ipResult.stdout as String).trim().split(' ').first;

            final connResult = await SudoProcess.run(
          'nmcli', ['-t', '-f', 'SSID,SECURITY,IN-USE', 'dev', 'wifi']);
      if (connResult.exitCode != 0) {
        throw Exception(
            'Failed to retrieve connected network: ${connResult.stderr}');
      }

      final lines = (connResult.stdout as String).split('\n');
      WifiNetwork? connectedNetwork;
      for (var line in lines) {
        if (line.contains('*')) {
                    final parts = line.split(':');
          if (parts.isNotEmpty) {
            final ssid = parts[0];
            final security = parts.length > 1 ? parts[1] : '';
            WifiEncryptionType encType = WifiEncryptionType.open;
            if (security.contains('WPA3')) {
              encType = security.contains('EAP')
                  ? WifiEncryptionType.wpa3Enterprise
                  : WifiEncryptionType.wpa3Personal;
            } else if (security.contains('WPA2')) {
              encType = security.contains('EAP')
                  ? WifiEncryptionType.wpa2Enterprise
                  : WifiEncryptionType.wpa2Personal;
            }

            connectedNetwork = WifiNetwork(
              ssid: ssid,
              encryptionType: encType,
              isConnected: true,
              isKnown: true,
            );
            break;
          }
        }
      }

      return DeviceInfo(
          ipAddress: ipAddress, connectedNetwork: connectedNetwork);
    } catch (e) {
      throw Exception('Error retrieving device info: $e');
    }
  }

    Future<NetworkMode> getCurrentNetworkMode() async {
    final prefs = await SharedPreferences.getInstance();
    final mode = prefs.getString('network_mode') ?? 'client';
    
    switch (mode) {
      case 'access_point':
        return NetworkMode.accessPoint;
      case 'lan_only':
        return NetworkMode.lanOnly;
      default:
        return NetworkMode.client;
    }
  }

  Future<void> setNetworkMode(NetworkMode mode) async {
    final prefs = await SharedPreferences.getInstance();
    String modeString;
    
    switch (mode) {
      case NetworkMode.accessPoint:
        modeString = 'access_point';
        break;
      case NetworkMode.lanOnly:
        modeString = 'lan_only';
        break;
      default:
        modeString = 'client';
    }
    
    await prefs.setString('network_mode', modeString);
  }

    Future<void> startAccessPoint(AccessPointConfig config) async {
    try {
            await stopAccessPoint();
      
            if (config.channel == 0) {
        final bestChannel = await findBestChannel(config.band);
        config = AccessPointConfig(
          ssid: config.ssid,
          password: config.password,
          band: config.band,
          channel: bestChannel,
          encryptionType: config.encryptionType,
          hidden: config.hidden,
          maxClients: config.maxClients,
        );
      }
      
            final connectionName = 'STP-Velox-AP';
      final args = [
        'connection', 'add',
        'type', 'wifi',
        'ifname', 'wlan0',
        'con-name', connectionName,
        'autoconnect', 'yes',
        'ssid', config.ssid,
        'mode', 'ap',
        'wifi.band', config.band.nmcliValue,
        'wifi-sec.key-mgmt', _getKeyMgmt(config.encryptionType),
        'wifi-sec.psk', config.password,
        'ipv4.method', 'shared',
        'ipv4.addresses', '192.168.4.1/24',
      ];
      
      if (config.channel > 0) {
        args.addAll(['wifi.channel', config.channel.toString()]);
      }
      
      if (config.hidden) {
        args.addAll(['wifi.hidden', 'yes']);
      }
      
      final result = await SudoProcess.run('nmcli', args);
      if (result.exitCode != 0) {
        throw Exception('Failed to create AP: ${result.stderr}');
      }
      
            final activateResult = await SudoProcess.run('nmcli', ['connection', 'up', connectionName]);
      if (activateResult.exitCode != 0) {
        throw Exception('Failed to activate AP: ${activateResult.stderr}');
      }
      
            await _saveAccessPointConfig(config);
      await setNetworkMode(NetworkMode.accessPoint);
      
    } catch (e) {
      throw Exception('Failed to start access point: $e');
    }
  }

  Future<void> stopAccessPoint() async {
    try {
      final connectionName = 'STP-Velox-AP';
      
            await SudoProcess.run('nmcli', ['connection', 'down', connectionName]);
      
            await SudoProcess.run('nmcli', ['connection', 'delete', connectionName]);
      
            await _resetWifiInterface();
      
    } catch (e) {
                  try {
        await _resetWifiInterface();
      } catch (resetError) {
              }
    }
  }

  Future<bool> isAccessPointActive() async {
    try {
      final result = await SudoProcess.run('nmcli', ['-t', '-f', 'NAME,TYPE,DEVICE', 'connection', 'show', '--active']);
      if (result.exitCode != 0) return false;
      
      final lines = (result.stdout as String).split('\n');
      for (var line in lines) {
        if (line.contains('STP-Velox-AP') && line.contains('wifi')) {
          return true;
        }
      }
      return false;
    } catch (e) {
      return false;
    }
  }

  Future<AccessPointConfig?> getAccessPointConfig() async {
    final prefs = await SharedPreferences.getInstance();
    final configJson = prefs.getString('ap_config');
    if (configJson == null) return null;
    
    try {
      final config = json.decode(configJson) as Map<String, dynamic>;
      return AccessPointConfig(
        ssid: config['ssid'] as String,
        password: config['password'] as String,
        band: WifiBand.values.firstWhere(
          (b) => b.toString() == config['band'],
          orElse: () => WifiBand.bandAuto,
        ),
        channel: config['channel'] as int? ?? 0,
        encryptionType: WifiEncryptionType.values.firstWhere(
          (e) => e.toString() == config['encryptionType'],
          orElse: () => WifiEncryptionType.wpa3Personal,
        ),
        hidden: config['hidden'] as bool? ?? false,
        maxClients: config['maxClients'] as int? ?? 8,
      );
    } catch (e) {
      return null;
    }
  }

  Future<void> _saveAccessPointConfig(AccessPointConfig config) async {
    final prefs = await SharedPreferences.getInstance();
    final configJson = json.encode({
      'ssid': config.ssid,
      'password': config.password,
      'band': config.band.toString(),
      'channel': config.channel,
      'encryptionType': config.encryptionType.toString(),
      'hidden': config.hidden,
      'maxClients': config.maxClients,
    });
    await prefs.setString('ap_config', configJson);
  }

  Future<WifiBand> findBestWifiBand() async {
    try {
            final result = await SudoProcess.run('iw', ['phy', 'phy0', 'info']);
      if (result.exitCode == 0 && (result.stdout as String).contains('5180')) {
        return WifiBand.band5GHz;
      } else {
        return WifiBand.band2_4GHz;
      }
    } catch (e) {
      return WifiBand.band2_4GHz;
    }
  }

  Future<int> findBestChannel(WifiBand band) async {
    try {
            final channels = band.channels;
      final interference = <int, int>{};
      
      for (int channel in channels) {
        interference[channel] = 0;
      }
      
            final scanResult = await SudoProcess.run('iwlist', ['wlan0', 'scan']);
      if (scanResult.exitCode == 0) {
        final output = scanResult.stdout as String;
        final lines = output.split('\n');
        
        for (var line in lines) {
          if (line.contains('Channel:')) {
            final match = RegExp(r'Channel:(\d+)').firstMatch(line);
            if (match != null) {
              final channel = int.tryParse(match.group(1)!);
              if (channel != null && interference.containsKey(channel)) {
                interference[channel] = interference[channel]! + 1;
              }
            }
          }
        }
      }
      
            int bestChannel = channels.first;
      int minInterference = interference[bestChannel] ?? 0;
      
      for (var channel in channels) {
        final channelInterference = interference[channel] ?? 0;
        if (channelInterference < minInterference) {
          minInterference = channelInterference;
          bestChannel = channel;
        }
      }
      
      return bestChannel;
    } catch (e) {
            return band.channels.first;
    }
  }

    Future<List<SavedNetwork>> getSavedNetworks() async {
    final prefs = await SharedPreferences.getInstance();
    final savedNetworksJson = prefs.getStringList('saved_networks') ?? [];
    
    return savedNetworksJson.map((networkJson) {
      final network = json.decode(networkJson) as Map<String, dynamic>;
      
      WifiCredentials credentials;
      final credType = network['credentialsType'] as String;
      if (credType == 'personal') {
        credentials = PersonalCredentials(network['password'] as String);
      } else {
        credentials = EnterpriseCredentials(
          username: network['username'] as String,
          password: network['password'] as String,
          caCertificatePath: network['caCertificatePath'] as String?,
        );
      }
      
      return SavedNetwork(
        ssid: network['ssid'] as String,
        encryptionType: WifiEncryptionType.values.firstWhere(
          (e) => e.toString() == network['encryptionType'],
          orElse: () => WifiEncryptionType.wpa2Personal,
        ),
        credentials: credentials,
        lastConnected: DateTime.parse(network['lastConnected'] as String),
        autoConnect: network['autoConnect'] as bool? ?? true,
      );
    }).toList();
  }

  Future<void> saveNetwork(SavedNetwork network) async {
    final prefs = await SharedPreferences.getInstance();
    final savedNetworks = await getSavedNetworks();
    
        savedNetworks.removeWhere((n) => n.ssid == network.ssid);
    
        savedNetworks.add(network);
    
        final networksJson = savedNetworks.map((n) {
      final Map<String, dynamic> networkMap = {
        'ssid': n.ssid,
        'encryptionType': n.encryptionType.toString(),
        'lastConnected': n.lastConnected.toIso8601String(),
        'autoConnect': n.autoConnect,
      };
      
      if (n.credentials is PersonalCredentials) {
        final creds = n.credentials as PersonalCredentials;
        networkMap['credentialsType'] = 'personal';
        networkMap['password'] = creds.password;
      } else if (n.credentials is EnterpriseCredentials) {
        final creds = n.credentials as EnterpriseCredentials;
        networkMap['credentialsType'] = 'enterprise';
        networkMap['username'] = creds.username;
        networkMap['password'] = creds.password;
        networkMap['caCertificatePath'] = creds.caCertificatePath;
      }
      
      return json.encode(networkMap);
    }).toList();
    
    await prefs.setStringList('saved_networks', networksJson);
  }

  Future<void> removeSavedNetwork(String ssid) async {
    final prefs = await SharedPreferences.getInstance();
    final savedNetworks = await getSavedNetworks();
    
    savedNetworks.removeWhere((n) => n.ssid == ssid);
    
    final networksJson = savedNetworks.map((n) {
      final Map<String, dynamic> networkMap = {
        'ssid': n.ssid,
        'encryptionType': n.encryptionType.toString(),
        'lastConnected': n.lastConnected.toIso8601String(),
        'autoConnect': n.autoConnect,
      };
      
      if (n.credentials is PersonalCredentials) {
        final creds = n.credentials as PersonalCredentials;
        networkMap['credentialsType'] = 'personal';
        networkMap['password'] = creds.password;
      } else if (n.credentials is EnterpriseCredentials) {
        final creds = n.credentials as EnterpriseCredentials;
        networkMap['credentialsType'] = 'enterprise';
        networkMap['username'] = creds.username;
        networkMap['password'] = creds.password;
        networkMap['caCertificatePath'] = creds.caCertificatePath;
      }
      
      return json.encode(networkMap);
    }).toList();
    
    await prefs.setStringList('saved_networks', networksJson);
  }

  Future<SavedNetwork?> getSavedNetwork(String ssid) async {
    final savedNetworks = await getSavedNetworks();
    try {
      return savedNetworks.firstWhere((n) => n.ssid == ssid);
    } catch (e) {
      return null;
    }
  }

    Future<void> enableLanOnlyMode() async {
    try {
            await SudoProcess.run('nmcli', ['radio', 'wifi', 'off']);
      
            await SudoProcess.run('nmcli', ['connection', 'up', 'Wired connection 1']);
      
      await setNetworkMode(NetworkMode.lanOnly);
    } catch (e) {
      throw Exception('Failed to enable LAN only mode: $e');
    }
  }

  Future<void> disableLanOnlyMode() async {
    try {
            await SudoProcess.run('nmcli', ['radio', 'wifi', 'on']);
      
      await setNetworkMode(NetworkMode.client);
    } catch (e) {
      throw Exception('Failed to disable LAN only mode: $e');
    }
  }

  Future<bool> isLanOnlyModeActive() async {
    try {
      final result = await SudoProcess.run('nmcli', ['radio', 'wifi']);
      if (result.exitCode != 0) return false;
      
      final output = (result.stdout as String).trim();
      return output.contains('disabled');
    } catch (e) {
      return false;
    }
  }

  String _getKeyMgmt(WifiEncryptionType encryptionType) {
    switch (encryptionType) {
      case WifiEncryptionType.open:
        return 'none';
      case WifiEncryptionType.wpa2Personal:
      case WifiEncryptionType.wpa3Personal:
        return 'wpa-psk';
      case WifiEncryptionType.wpa2Enterprise:
      case WifiEncryptionType.wpa3Enterprise:
        return 'wpa-eap';
    }
  }
  
    Future<void> _ensureWifiEnabled() async {
    try {
            final radioResult = await SudoProcess.run('nmcli', ['radio', 'wifi']);
      if (radioResult.exitCode == 0) {
        final output = (radioResult.stdout as String).trim();
        if (output.contains('disabled')) {
                    await SudoProcess.run('nmcli', ['radio', 'wifi', 'on']);
                    await Future.delayed(const Duration(milliseconds: 2000));
        }
      }
      
            await SudoProcess.run('nmcli', ['device', 'set', 'wlan0', 'managed', 'yes']);
      
    } catch (e) {
            print('Warning: Could not ensure WiFi enabled: $e');
    }
  }
  
    Future<void> _resetWifiInterface() async {
    try {
            await SudoProcess.run('ip', ['link', 'set', 'wlan0', 'down']);
      await Future.delayed(const Duration(milliseconds: 500));
      await SudoProcess.run('ip', ['link', 'set', 'wlan0', 'up']);
      await Future.delayed(const Duration(milliseconds: 500));
      
            await SudoProcess.run('nmcli', ['device', 'set', 'wlan0', 'managed', 'yes']);
      
            await SudoProcess.run('nmcli', ['device', 'wifi', 'rescan']);
      
    } catch (e) {
            print('Warning: Could not reset WiFi interface: $e');
    }
  }
}
