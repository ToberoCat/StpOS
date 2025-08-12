import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class DeviceInfoScreen extends StatefulWidget {
  const DeviceInfoScreen({super.key});

  @override
  State<DeviceInfoScreen> createState() => _DeviceInfoScreenState();
}

class _DeviceInfoScreenState extends State<DeviceInfoScreen> {

  @override
  void initState() {
    context.read<WifiBloc>().add(LoadDeviceInfoEvent());
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Device Information'),
      body: BlocBuilder<WifiBloc, WifiState>(
        builder: (context, state) {
          if (state is WifiLoadingState) {
            return const Center(child: CircularProgressIndicator());
          } else if (state is DeviceInfoLoadedState) {
            return Padding(
              padding: const EdgeInsets.all(16.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('IP Address: ${state.deviceInfo.ipAddress}',
                      style: const TextStyle(fontSize: 18)),
                  const SizedBox(height: 16),
                  if (state.deviceInfo.connectedNetwork != null)
                    Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text('Connected Network:',
                            style: TextStyle(
                                fontSize: 18, fontWeight: FontWeight.bold)),
                        Text('SSID: ${state.deviceInfo.connectedNetwork!.ssid}',
                            style: const TextStyle(fontSize: 16)),
                        Text(
                            'Encryption: ${state.deviceInfo.connectedNetwork!.encryptionType}',
                            style: const TextStyle(fontSize: 16)),
                      ],
                    )
                  else
                    const Text('Not connected to any network.',
                        style: TextStyle(fontSize: 16)),
                ],
              ),
            );
          } else if (state is WifiErrorState) {
            return Center(
                child: Text('Error: ${state.message}',
                    style: const TextStyle(color: Colors.red)));
          } else {
            return const SizedBox();
          }
        },
      ),
    );
  }
}
