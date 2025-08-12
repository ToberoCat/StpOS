import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/screens/wifi/wifi_manual_connect_screen.dart';
import 'package:stpvelox/presentation/screens/wifi/wifi_detail_screen.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class WifiScanListScreen extends StatefulWidget {
  const WifiScanListScreen({super.key});

  @override
  State<WifiScanListScreen> createState() => _WifiScanListScreenState();
}

class _WifiScanListScreenState extends State<WifiScanListScreen> {
  @override
  void initState() {
    context.read<WifiBloc>().add(LoadNetworksEvent());
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Scanned WiFi Networks'),
      body: Column(
        children: [
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Row(
              children: [
                                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: () {
                      context.read<WifiBloc>().add(LoadNetworksEvent());
                    },
                    icon: const Icon(Icons.refresh, size: 28),
                    label: const Text(
                      'Refresh',
                      style: TextStyle(fontSize: 20, fontWeight: FontWeight.w600),
                    ),
                    style: ElevatedButton.styleFrom(
                      padding: const EdgeInsets.symmetric(vertical: 16),
                      backgroundColor: Colors.blue[600],
                      foregroundColor: Colors.white,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                      elevation: 4,
                    ),
                  ),
                ),
                const SizedBox(width: 16),
                                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (_) => const WifiManualConnectScreen()),
                      );
                    },
                    icon: const Icon(Icons.add, size: 28),
                    label: const Text(
                      'Manual',
                      style: TextStyle(fontSize: 20, fontWeight: FontWeight.w600),
                    ),
                    style: ElevatedButton.styleFrom(
                      padding: const EdgeInsets.symmetric(vertical: 16),
                      backgroundColor: Colors.green[600],
                      foregroundColor: Colors.white,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                      elevation: 4,
                    ),
                  ),
                ),
              ],
            ),
          ),
          const Divider(height: 1),
                    Expanded(
            child: BlocBuilder<WifiBloc, WifiState>(
              builder: (context, state) {
                if (state is WifiLoadingState) {
                  return const Center(child: CircularProgressIndicator());
                } else if (state is WifiLoadedState) {
                  if (state.networks.isEmpty) {
                    return const Center(child: Text('No networks found.'));
                  }
                  return RefreshIndicator(
                    onRefresh: () async {
                      context.read<WifiBloc>().add(LoadNetworksEvent());
                    },
                    child: ListView.builder(
                      itemCount: state.networks.length,
                      itemBuilder: (context, index) {
                        final network = state.networks[index];
                        return Card(
                          margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                          child: ListTile(
                            title: Text(
                              network.ssid,
                              style: TextStyle(
                                fontSize: 16,
                                fontWeight: network.isConnected
                                    ? FontWeight.bold
                                    : FontWeight.normal,
                              ),
                            ),
                            subtitle: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  network.isConnected
                                      ? 'Connected'
                                      : network.isKnown
                                      ? 'Saved Network - Tap to connect'
                                      : 'Tap to configure',
                                  style: TextStyle(
                                    color: network.isConnected
                                        ? Colors.green
                                        : network.isKnown
                                        ? Colors.blue
                                        : Colors.grey,
                                  ),
                                ),
                                if (network.encryptionType != WifiEncryptionType.open)
                                  Text(
                                    _getEncryptionLabel(network.encryptionType),
                                    style: const TextStyle(
                                      fontSize: 12,
                                      color: Colors.grey,
                                    ),
                                  ),
                              ],
                            ),
                            leading: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Icon(
                                  network.isKnown ? Icons.bookmark : Icons.wifi,
                                  color: network.isConnected
                                      ? Colors.green
                                      : network.isKnown
                                      ? Colors.blue
                                      : Colors.grey,
                                ),
                                if (network.encryptionType != WifiEncryptionType.open)
                                  const Icon(
                                    Icons.lock,
                                    size: 12,
                                    color: Colors.grey,
                                  ),
                              ],
                            ),
                            trailing: network.isKnown && !network.isConnected
                                ? IconButton(
                                    onPressed: () {
                                      context.read<WifiBloc>().add(
                                        ConnectToSavedNetworkEvent(network.ssid),
                                      );
                                    },
                                    icon: const Icon(Icons.play_arrow, color: Colors.green),
                                    tooltip: 'Quick Connect',
                                  )
                                : Icon(
                                    Icons.arrow_forward_ios,
                                    size: 16,
                                    color: Colors.grey.shade400,
                                  ),
                            onTap: () {
                              if (network.isKnown && !network.isConnected) {
                                                                context.read<WifiBloc>().add(
                                  ConnectToSavedNetworkEvent(network.ssid),
                                );
                              } else {
                                                                Navigator.push(
                                  context,
                                  MaterialPageRoute(
                                      builder: (_) =>
                                          WifiDetailScreen(network: network)),
                                );
                              }
                            },
                          ),
                        );
                      },
                    ),
                  );
                } else if (state is WifiErrorState) {
                  return Center(
                      child: Text(
                        'Error: ${state.message}',
                        style: const TextStyle(color: Colors.red),
                      ));
                }
                return const SizedBox();
              },
            ),
          ),
        ],
      ),
    );
  }

  String _getEncryptionLabel(WifiEncryptionType encryptionType) {
    switch (encryptionType) {
      case WifiEncryptionType.wpa3Personal:
        return 'WPA3';
      case WifiEncryptionType.wpa2Personal:
        return 'WPA2';
      case WifiEncryptionType.wpa3Enterprise:
        return 'WPA3 Enterprise';
      case WifiEncryptionType.wpa2Enterprise:
        return 'WPA2 Enterprise';
      case WifiEncryptionType.open:
        return 'Open';
    }
  }
}