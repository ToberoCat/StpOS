import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SavedNetworksScreen extends StatefulWidget {
  const SavedNetworksScreen({super.key});

  @override
  State<SavedNetworksScreen> createState() => _SavedNetworksScreenState();
}

class _SavedNetworksScreenState extends State<SavedNetworksScreen> {
  final Map<String, bool> _showPasswords = {};

  @override
  void initState() {
    super.initState();
    context.read<WifiBloc>().add(LoadSavedNetworksEvent());
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Saved Networks'),
      body: Column(
        children: [
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: () {
                      context.read<WifiBloc>().add(LoadSavedNetworksEvent());
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
              ],
            ),
          ),
          const Divider(height: 1),
          Expanded(
            child: BlocBuilder<WifiBloc, WifiState>(
              builder: (context, state) {
                if (state is WifiLoadingState) {
                  return const Center(child: CircularProgressIndicator());
                } else if (state is SavedNetworksLoadedState) {
                  if (state.networks.isEmpty) {
                    return const Center(
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Icon(Icons.bookmark_border, size: 64, color: Colors.grey),
                          SizedBox(height: 16),
                          Text(
                            'No saved networks',
                            style: TextStyle(fontSize: 18, color: Colors.grey),
                          ),
                          SizedBox(height: 8),
                          Text(
                            'Connect to a WiFi network to save it',
                            style: TextStyle(color: Colors.grey),
                          ),
                        ],
                      ),
                    );
                  }

                                    final sortedNetworks = List.of(state.networks)
                    ..sort((a, b) => b.lastConnected.compareTo(a.lastConnected));

                  return ListView.builder(
                    itemCount: sortedNetworks.length,
                    itemBuilder: (context, index) {
                      final network = sortedNetworks[index];
                      final showPassword = _showPasswords[network.ssid] ?? false;
                      
                      String passwordText = '';
                      if (network.credentials is PersonalCredentials) {
                        final creds = network.credentials as PersonalCredentials;
                        passwordText = showPassword ? creds.password : '••••••••';
                      }

                      return Container(
                        margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                        decoration: BoxDecoration(
                          color: Colors.grey[850],
                          borderRadius: BorderRadius.circular(12),
                          border: Border.all(color: Colors.grey[700]!, width: 1),
                        ),
                        child: ListTile(
                          contentPadding: const EdgeInsets.all(16),
                          title: Text(
                            network.ssid,
                            style: const TextStyle(
                              fontSize: 18,
                              fontWeight: FontWeight.w600,
                              color: Colors.white,
                            ),
                          ),
                          subtitle: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                'Last connected: ${_formatDateTime(network.lastConnected)}',
                                style: const TextStyle(color: Colors.grey),
                              ),
                              if (network.credentials is PersonalCredentials) ...[
                                const SizedBox(height: 4),
                                Row(
                                  children: [
                                    Expanded(
                                      child: Text(
                                        'Password: $passwordText',
                                        style: const TextStyle(
                                          fontFamily: 'monospace',
                                          fontSize: 12,
                                        ),
                                      ),
                                    ),
                                    IconButton(
                                      onPressed: () {
                                        setState(() {
                                          _showPasswords[network.ssid] = !showPassword;
                                        });
                                      },
                                      icon: Icon(
                                        showPassword ? Icons.visibility_off : Icons.visibility,
                                        size: 20,
                                      ),
                                    ),
                                  ],
                                ),
                              ] else if (network.credentials is EnterpriseCredentials) ...[
                                const SizedBox(height: 4),
                                const Text(
                                  'Enterprise Network',
                                  style: TextStyle(
                                    color: Colors.blue,
                                    fontWeight: FontWeight.w500,
                                  ),
                                ),
                              ],
                            ],
                          ),
                          leading: Icon(
                            Icons.bookmark,
                            color: network.autoConnect ? Colors.green : Colors.grey,
                          ),
                          trailing: Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                                                            Container(
                                margin: const EdgeInsets.only(right: 8),
                                child: ElevatedButton(
                                  onPressed: () {
                                    context.read<WifiBloc>().add(
                                      ConnectToSavedNetworkEvent(network.ssid),
                                    );
                                  },
                                  style: ElevatedButton.styleFrom(
                                    backgroundColor: Colors.green[600],
                                    foregroundColor: Colors.white,
                                    minimumSize: const Size(60, 48),
                                    shape: RoundedRectangleBorder(
                                      borderRadius: BorderRadius.circular(8),
                                    ),
                                  ),
                                  child: const Icon(Icons.wifi, size: 24),
                                ),
                              ),
                                                            ElevatedButton(
                                onPressed: () {
                                  _showRemoveDialog(context, network.ssid);
                                },
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: Colors.red[600],
                                  foregroundColor: Colors.white,
                                  minimumSize: const Size(60, 48),
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(8),
                                  ),
                                ),
                                child: const Icon(Icons.delete, size: 24),
                              ),
                            ],
                          ),
                        ),
                      );
                    },
                  );
                } else if (state is WifiErrorState) {
                  return Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        const Icon(Icons.error, size: 64, color: Colors.red),
                        const SizedBox(height: 16),
                        Text(
                          'Error: ${state.message}',
                          style: const TextStyle(color: Colors.red),
                          textAlign: TextAlign.center,
                        ),
                        const SizedBox(height: 16),
                        ElevatedButton(
                          onPressed: () {
                            context.read<WifiBloc>().add(LoadSavedNetworksEvent());
                          },
                          child: const Text('Retry'),
                        ),
                      ],
                    ),
                  );
                }
                return const SizedBox();
              },
            ),
          ),
        ],
      ),
    );
  }

  String _formatDateTime(DateTime dateTime) {
    final now = DateTime.now();
    final difference = now.difference(dateTime);

    if (difference.inMinutes < 1) {
      return 'Just now';
    } else if (difference.inHours < 1) {
      return '${difference.inMinutes}m ago';
    } else if (difference.inDays < 1) {
      return '${difference.inHours}h ago';
    } else if (difference.inDays < 7) {
      return '${difference.inDays}d ago';
    } else {
      return '${dateTime.day}/${dateTime.month}/${dateTime.year}';
    }
  }

  void _showRemoveDialog(BuildContext context, String ssid) {
    showDialog(
      context: context,
      builder: (BuildContext dialogContext) {
        return AlertDialog(
          title: const Text('Remove Network'),
          content: Text('Remove "$ssid" from saved networks?'),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(dialogContext).pop(),
              child: const Text('Cancel'),
            ),
            TextButton(
              onPressed: () {
                Navigator.of(dialogContext).pop();
                context.read<WifiBloc>().add(RemoveSavedNetworkEvent(ssid));
              },
              child: const Text('Remove', style: TextStyle(color: Colors.red)),
            ),
          ],
        );
      },
    );
  }
}