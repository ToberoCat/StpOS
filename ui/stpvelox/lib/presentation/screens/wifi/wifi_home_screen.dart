import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/screens/wifi/access_point_config_screen.dart';
import 'package:stpvelox/presentation/screens/wifi/device_info_screen.dart';
import 'package:stpvelox/presentation/screens/wifi/saved_networks_screen.dart';
import 'package:stpvelox/presentation/screens/wifi/wifi_scan_list_screen.dart';
import 'package:stpvelox/presentation/widgets/dashboard_tile.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';
import 'package:stpvelox/domain/entities/network_mode.dart';

class WifiHomeScreen extends StatefulWidget {
  const WifiHomeScreen({super.key});

  @override
  State<WifiHomeScreen> createState() => _WifiHomeScreenState();
}

class _WifiHomeScreenState extends State<WifiHomeScreen> {
  @override
  void initState() {
    super.initState();
    context.read<WifiBloc>().add(LoadNetworkModeEvent());
  }

  void _handleModeChange(BuildContext context, NetworkMode currentMode, NetworkMode selectedMode) {
    final bloc = context.read<WifiBloc>();
    
    print('Mode change requested: $currentMode -> $selectedMode');
    
        bloc.add(SetNetworkModeEvent(selectedMode));
    
        if (selectedMode == NetworkMode.accessPoint) {
            Future.delayed(const Duration(milliseconds: 300), () {
        bloc.add(StartAccessPointWithLastConfigEvent());
      });
    } else if (selectedMode == NetworkMode.lanOnly) {
      Future.delayed(const Duration(milliseconds: 300), () {
        bloc.add(EnableLanOnlyModeEvent());
      });
    }
    
        ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Switching to ${_getModeDisplayName(selectedMode)}...'),
        duration: const Duration(seconds: 2),
        backgroundColor: Colors.blue[600],
      ),
    );
  }
  
  String _getModeDisplayName(NetworkMode mode) {
    switch (mode) {
      case NetworkMode.client:
        return 'WiFi Client Mode';
      case NetworkMode.accessPoint:
        return 'Hotspot Mode';
      case NetworkMode.lanOnly:
        return 'LAN Only Mode';
    }
  }

  @override  
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Network Control'),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(32.0),
          child: Column(
            children: [
                            BlocBuilder<WifiBloc, WifiState>(
                builder: (context, state) {
                  NetworkMode currentMode = NetworkMode.client;
                  if (state is NetworkModeLoadedState) {
                    currentMode = state.mode;
                  }

                  return Container(
                    decoration: BoxDecoration(
                      color: Colors.grey[900],
                      borderRadius: BorderRadius.circular(12),
                      border: Border.all(color: Colors.grey[700]!, width: 2),
                    ),
                    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                    child: Row(
                      children: [
                        const Text(
                          'Mode:',
                          style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: Colors.white),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: DropdownButtonHideUnderline(
                            child: DropdownButton<NetworkMode>(
                              value: currentMode,
                              dropdownColor: Colors.grey[800],
                              isExpanded: true,
                              icon: const Icon(Icons.arrow_drop_down, color: Colors.white, size: 28),
                              style: const TextStyle(fontSize: 16, color: Colors.white, fontWeight: FontWeight.w500),
                              items: [
                                DropdownMenuItem(
                                  value: NetworkMode.client,
                                  child: Row(
                                    children: [
                                      Icon(Icons.wifi, color: Colors.blue[300], size: 24),
                                      const SizedBox(width: 12),
                                      const Text('WiFi Client'),
                                    ],
                                  ),
                                ),
                                DropdownMenuItem(
                                  value: NetworkMode.accessPoint,
                                  child: Row(
                                    children: [
                                      Icon(Icons.router, color: Colors.purple[300], size: 24),
                                      const SizedBox(width: 12),
                                      const Text('Hotspot Mode'),
                                    ],
                                  ),
                                ),
                                DropdownMenuItem(
                                  value: NetworkMode.lanOnly,
                                  child: Row(
                                    children: [
                                      Icon(Icons.cable, color: Colors.grey[400], size: 24),
                                      const SizedBox(width: 12),
                                      const Text('LAN Only'),
                                    ],
                                  ),
                                ),
                              ],
                              onChanged: (NetworkMode? selectedMode) {
                                if (selectedMode != null && selectedMode != currentMode) {
                                                                    _handleModeChange(context, currentMode, selectedMode);
                                }
                              },
                            ),
                          ),
                        ),
                      ],
                    ),
                  );
                },
              ),
              const SizedBox(height: 12),
                            Expanded(
                child: BlocBuilder<WifiBloc, WifiState>(
                  builder: (context, state) {
                    NetworkMode currentMode = NetworkMode.client;
                    if (state is NetworkModeLoadedState) {
                      currentMode = state.mode;
                    }

                    return GridView.count(
                      crossAxisCount: 2,
                      crossAxisSpacing: 20,
                      mainAxisSpacing: 20,
                      childAspectRatio: 1.1,                       children: [
                                                if (currentMode == NetworkMode.client) ...[
                          DashboardTile(
                            label: "Connect to WiFi",
                            icon: Icons.wifi,
                            destination: const WifiScanListScreen(),
                            color: Colors.blue[600]!,
                          ),
                          DashboardTile(
                            label: "Saved Networks",
                            icon: Icons.bookmark,
                            destination: const SavedNetworksScreen(),
                            color: Colors.green[600]!,
                          ),
                        ],
                        
                                                if (currentMode == NetworkMode.accessPoint) ...[
                          DashboardTile(
                            label: "Hotspot Settings",
                            icon: Icons.router,
                            destination: const AccessPointConfigScreen(),
                            color: Colors.purple[600]!,
                          ),
                          DashboardTile(
                            label: "Network Status",
                            icon: Icons.network_check,
                            destination: const DeviceInfoScreen(),
                            color: Colors.orange[600]!,
                          ),
                        ],
                        
                                                DashboardTile(
                          label: "Device Info",
                          icon: Icons.info,
                          destination: const DeviceInfoScreen(),
                          color: Colors.teal[600]!,
                        ),
                        
                                                if (currentMode == NetworkMode.lanOnly)
                          DashboardTile(
                            label: "LAN Status",
                            icon: Icons.cable,
                            destination: const DeviceInfoScreen(),
                            color: Colors.grey[600]!,
                          ),
                      ],
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
