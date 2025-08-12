import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';
import 'package:stpvelox/domain/entities/wifi_network.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/screens/wifi/wifi_enterprise_credential_screen.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class WifiDetailScreen extends StatefulWidget {
  final WifiNetwork network;

  const WifiDetailScreen({super.key, required this.network});

  @override
  State<WifiDetailScreen> createState() => _WifiDetailScreenState();
}

class _WifiDetailScreenState extends State<WifiDetailScreen> {
  final TextEditingController _passwordController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final network = widget.network;
    return Scaffold(
      appBar: createTopBar(context, 'WiFi: ${network.ssid}'),
      body: BlocConsumer<WifiBloc, WifiState>(
        listener: (context, state) {
          if (state is WifiErrorState) {
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(
              content: Text('Error: ${state.message}'),
              backgroundColor: Colors.red,
            ));
          } else if (state is WifiConnectedState) {
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                  content: Text('Connected to ${state.ssid} successfully!'),
                  backgroundColor: Colors.green),
            );
            Navigator.pop(context);
          } else if (state is WifiForgottenState) {
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                  content: Text('Forgotten network ${state.ssid}'),
                  backgroundColor: Colors.green),
            );
            Navigator.pop(context);
          }
        },
        builder: (context, state) {
          final isLoading =
              state is WifiConnectingState || state is WifiForgettingState;
          return Padding(
            padding: const EdgeInsets.all(16),
            child: SingleChildScrollView(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('SSID: ${network.ssid}',
                      style: const TextStyle(fontSize: 18)),
                  const SizedBox(height: 8),
                  Text('Encryption: ${network.encryptionType}'),
                  const SizedBox(height: 8),
                  if (network.isConnected)
                    const Text('Status: Connected',
                        style: TextStyle(color: Colors.green))
                  else if (network.isKnown)
                    const Text('Status: Known (not connected)',
                        style: TextStyle(color: Colors.orange))
                  else
                    const Text('Status: Unknown (not connected)'),
                  const SizedBox(height: 16),
                  if (!network.isConnected && !network.isKnown)
                    _buildConnectionUI(context, network, isLoading),
                  if (network.isKnown || network.isConnected)
                    ElevatedButton(
                      onPressed: isLoading
                          ? null
                          : () {
                              context
                                  .read<WifiBloc>()
                                  .add(ForgetNetworkEvent(network.ssid));
                            },
                      child: isLoading
                          ? const SizedBox(
                              width: 16,
                              height: 16,
                              child: CircularProgressIndicator(
                                  strokeWidth: 2, color: Colors.white))
                          : const Text('Forget Network'),
                    ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }

  Widget _buildConnectionUI(
      BuildContext context, WifiNetwork network, bool isLoading) {
    final isEnterprise =
        network.encryptionType == WifiEncryptionType.wpa2Enterprise ||
            network.encryptionType == WifiEncryptionType.wpa3Enterprise;

    if (isEnterprise) {
      return ElevatedButton(
        onPressed: isLoading
            ? null
            : () async {
                final creds = await Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (_) =>
                        WifiEnterpriseCredentialScreen(ssid: network.ssid),
                  ),
                );
                if (creds == null) return;
                context.read<WifiBloc>().add(
                      ConnectToNetworkEvent(network.ssid,
                          network.encryptionType, creds as WifiCredentials),
                    );
              },
        child: const Text('Enter Enterprise Credentials'),
      );
    } else if (network.encryptionType == WifiEncryptionType.open) {
      return ElevatedButton(
        onPressed: isLoading
            ? null
            : () {
                context.read<WifiBloc>().add(
                      ConnectToNetworkEvent(network.ssid,
                          network.encryptionType, PersonalCredentials('')),
                    );
              },
        child: isLoading
            ? const SizedBox(
                width: 16,
                height: 16,
                child: CircularProgressIndicator(
                    strokeWidth: 2, color: Colors.white))
            : const Text('Connect (No Password)'),
      );
    } else {
            return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text('Password:'),
          TextField(
            controller: _passwordController,
            obscureText: true,
            decoration: const InputDecoration(
              border: OutlineInputBorder(),
              hintText: 'Enter password',
            ),
          ),
          const SizedBox(height: 16),
          ElevatedButton(
            onPressed: isLoading
                ? null
                : () {
                    final password = _passwordController.text.trim();
                    if (password.isEmpty) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          content: Text('Password cannot be empty'),
                          backgroundColor: Colors.red,
                        ),
                      );
                      return;
                    }
                    context.read<WifiBloc>().add(
                          ConnectToNetworkEvent(
                            network.ssid,
                            network.encryptionType,
                            PersonalCredentials(password),
                          ),
                        );
                  },
            child: isLoading
                ? const SizedBox(
                    width: 16,
                    height: 16,
                    child: CircularProgressIndicator(
                        strokeWidth: 2, color: Colors.white))
                : const Text('Connect'),
          ),
        ],
      );
    }
  }
}
