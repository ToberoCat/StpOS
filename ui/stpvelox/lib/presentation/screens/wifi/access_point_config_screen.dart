import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/core/utils/password_generator.dart';
import 'package:stpvelox/domain/entities/access_point_config.dart';
import 'package:stpvelox/domain/entities/wifi_band.dart';
import 'package:stpvelox/domain/entities/wifi_encryption_type.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_event.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_state.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class AccessPointConfigScreen extends StatefulWidget {
  const AccessPointConfigScreen({super.key});

  @override
  State<AccessPointConfigScreen> createState() => _AccessPointConfigScreenState();
}

class _AccessPointConfigScreenState extends State<AccessPointConfigScreen> {
  final _formKey = GlobalKey<FormState>();
  final _ssidController = TextEditingController();
  final _passwordController = TextEditingController();
  
  WifiBand _selectedBand = WifiBand.bandAuto;
  WifiEncryptionType _encryptionType = WifiEncryptionType.wpa3Personal;
  bool _isHidden = false;
  bool _showPassword = false;
  bool _isStarted = false;

  @override
  void initState() {
    super.initState();
    context.read<WifiBloc>().add(LoadAccessPointConfigEvent());
    
        _ssidController.text = 'STP-Velox-Robot';
    _passwordController.text = PasswordGenerator.generateReadablePassword();
  }

  @override
  void dispose() {
    _ssidController.dispose();
    _passwordController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Hotspot Settings'),
      body: BlocListener<WifiBloc, WifiState>(
        listener: (context, state) {
          if (state is AccessPointConfigLoadedState && state.config != null) {
            final config = state.config!;
            _ssidController.text = config.ssid;
            _passwordController.text = config.password;
            _selectedBand = config.band;
            _encryptionType = config.encryptionType;
            _isHidden = config.hidden;
            setState(() {});
          } else if (state is AccessPointStartedState) {
            _isStarted = true;
            setState(() {});
            ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(content: Text('Hotspot started successfully')),
            );
          } else if (state is AccessPointStoppedState) {
            _isStarted = false;
            setState(() {});
            ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(content: Text('Hotspot stopped')),
            );
          } else if (state is WifiErrorState) {
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                content: Text('Error: ${state.message}'),
                backgroundColor: Colors.red,
              ),
            );
          }
        },
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(16.0),
          child: Form(
            key: _formKey,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                                Container(
                  decoration: BoxDecoration(
                    color: _isStarted ? Colors.green[900] : Colors.grey[800],
                    borderRadius: BorderRadius.circular(12),
                    border: Border.all(
                      color: _isStarted ? Colors.green[400]! : Colors.grey[600]!,
                      width: 2,
                    ),
                  ),
                  padding: const EdgeInsets.all(20.0),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Icon(
                            _isStarted ? Icons.router : Icons.router_outlined,
                            color: _isStarted ? Colors.green[300] : Colors.grey[400],
                            size: 32,
                          ),
                          const SizedBox(width: 12),
                          Text(
                            _isStarted ? 'Hotspot Active' : 'Hotspot Inactive',
                            style: TextStyle(
                              fontSize: 20,
                              fontWeight: FontWeight.bold,
                              color: _isStarted ? Colors.green[300] : Colors.grey[300],
                            ),
                          ),
                        ],
                      ),
                      if (_isStarted) ...[
                        const SizedBox(height: 12),
                        Text(
                          'Network: ${_ssidController.text}',
                          style: const TextStyle(fontSize: 16, color: Colors.white, fontWeight: FontWeight.w500),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          'IP Address: 192.168.4.1',
                          style: const TextStyle(fontSize: 16, color: Colors.white, fontWeight: FontWeight.w500),
                        ),
                      ],
                    ],
                  ),
                ),
                const SizedBox(height: 16),

                                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text(
                          'Network Configuration',
                          style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                        ),
                        const SizedBox(height: 16),

                                                TextFormField(
                          controller: _ssidController,
                          decoration: const InputDecoration(
                            labelText: 'Network Name (SSID)',
                            border: OutlineInputBorder(),
                            prefixIcon: Icon(Icons.wifi),
                          ),
                          validator: (value) {
                            if (value == null || value.isEmpty) {
                              return 'Please enter a network name';
                            }
                            if (value.length < 3) {
                              return 'Network name must be at least 3 characters';
                            }
                            return null;
                          },
                        ),
                        const SizedBox(height: 16),

                                                TextFormField(
                          controller: _passwordController,
                          obscureText: !_showPassword,
                          decoration: InputDecoration(
                            labelText: 'Password',
                            border: const OutlineInputBorder(),
                            prefixIcon: const Icon(Icons.lock),
                            suffixIcon: Row(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                IconButton(
                                  onPressed: () {
                                    setState(() {
                                      _showPassword = !_showPassword;
                                    });
                                  },
                                  icon: Icon(
                                    _showPassword ? Icons.visibility_off : Icons.visibility,
                                  ),
                                ),
                                IconButton(
                                  onPressed: () {
                                    _passwordController.text = 
                                        PasswordGenerator.generateReadablePassword();
                                  },
                                  icon: const Icon(Icons.refresh),
                                  tooltip: 'Generate Password',
                                ),
                              ],
                            ),
                          ),
                          validator: (value) {
                            if (value == null || value.isEmpty) {
                              return 'Please enter a password';
                            }
                            if (value.length < 8) {
                              return 'Password must be at least 8 characters';
                            }
                            return null;
                          },
                        ),
                        const SizedBox(height: 16),

                                                DropdownButtonFormField<WifiBand>(
                          value: _selectedBand,
                          decoration: const InputDecoration(
                            labelText: 'WiFi Band',
                            border: OutlineInputBorder(),
                            prefixIcon: Icon(Icons.signal_wifi_4_bar),
                          ),
                          items: WifiBand.values.map((band) {
                            return DropdownMenuItem(
                              value: band,
                              child: Text(band.displayName),
                            );
                          }).toList(),
                          onChanged: (WifiBand? value) {
                            if (value != null) {
                              setState(() {
                                _selectedBand = value;
                              });
                            }
                          },
                        ),
                        const SizedBox(height: 16),

                                                DropdownButtonFormField<WifiEncryptionType>(
                          value: _encryptionType,
                          decoration: const InputDecoration(
                            labelText: 'Security',
                            border: OutlineInputBorder(),
                            prefixIcon: Icon(Icons.security),
                          ),
                          items: [
                            WifiEncryptionType.wpa3Personal,
                            WifiEncryptionType.wpa2Personal,
                            WifiEncryptionType.open,
                          ].map((type) {
                            String label;
                            switch (type) {
                              case WifiEncryptionType.wpa3Personal:
                                label = 'WPA3 Personal (Recommended)';
                                break;
                              case WifiEncryptionType.wpa2Personal:
                                label = 'WPA2 Personal';
                                break;
                              case WifiEncryptionType.open:
                                label = 'Open (No Security)';
                                break;
                              default:
                                label = type.toString();
                            }
                            return DropdownMenuItem(
                              value: type,
                              child: Text(label),
                            );
                          }).toList(),
                          onChanged: (WifiEncryptionType? value) {
                            if (value != null) {
                              setState(() {
                                _encryptionType = value;
                              });
                            }
                          },
                        ),
                        const SizedBox(height: 16),

                                                SwitchListTile(
                          title: const Text('Hidden Network'),
                          subtitle: const Text('Network won\'t appear in WiFi scan results'),
                          value: _isHidden,
                          onChanged: (value) {
                            setState(() {
                              _isHidden = value;
                            });
                          },
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 24),

                                Row(
                  children: [
                    if (_isStarted) ...[
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: () {
                            context.read<WifiBloc>().add(StopAccessPointEvent());
                          },
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.red[600],
                            foregroundColor: Colors.white,
                            padding: const EdgeInsets.symmetric(vertical: 20),
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(12),
                            ),
                            elevation: 4,
                          ),
                          icon: const Icon(Icons.stop, size: 28),
                          label: const Text('Stop Hotspot', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                        ),
                      ),
                      const SizedBox(width: 12),
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: () {
                            if (_formKey.currentState!.validate()) {
                              final config = AccessPointConfig(
                                ssid: _ssidController.text,
                                password: _passwordController.text,
                                band: _selectedBand,
                                encryptionType: _encryptionType,
                                hidden: _isHidden,
                              );
                              context.read<WifiBloc>().add(StartAccessPointEvent(config));
                            }
                          },
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.blue[600],
                            foregroundColor: Colors.white,
                            padding: const EdgeInsets.symmetric(vertical: 20),
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(12),
                            ),
                            elevation: 4,
                          ),
                          icon: const Icon(Icons.refresh, size: 28),
                          label: const Text('Restart', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                        ),
                      ),
                    ] else ...[
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: () {
                            if (_formKey.currentState!.validate()) {
                              final config = AccessPointConfig(
                                ssid: _ssidController.text,
                                password: _passwordController.text,
                                band: _selectedBand,
                                encryptionType: _encryptionType,
                                hidden: _isHidden,
                              );
                              context.read<WifiBloc>().add(StartAccessPointEvent(config));
                            }
                          },
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.green[600],
                            foregroundColor: Colors.white,
                            padding: const EdgeInsets.symmetric(vertical: 20),
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(12),
                            ),
                            elevation: 4,
                          ),
                          icon: const Icon(Icons.router, size: 28),
                          label: const Text('Start Hotspot', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                        ),
                      ),
                    ],
                  ],
                ),
                const SizedBox(height: 16),

                                Container(
                  decoration: BoxDecoration(
                    color: Colors.blue[900],
                    borderRadius: BorderRadius.circular(12),
                    border: Border.all(color: Colors.blue[600]!, width: 2),
                  ),
                  padding: const EdgeInsets.all(18.0),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Icon(Icons.info, color: Colors.blue[300], size: 28),
                          const SizedBox(width: 12),
                          Text(
                            'Hotspot Information',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                              color: Colors.blue[300],
                              fontSize: 18,
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 12),
                      const Text(
                        '• Other devices can connect using the network name and password above\n'
                        '• The robot will be accessible at IP address 192.168.4.1\n'
                        '• Up to 8 devices can connect simultaneously\n'
                        '• 5GHz band provides better performance when available',
                        style: TextStyle(
                          fontSize: 16,
                          color: Colors.white,
                          height: 1.4,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}