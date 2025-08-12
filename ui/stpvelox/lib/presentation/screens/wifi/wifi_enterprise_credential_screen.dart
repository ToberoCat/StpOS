import 'package:flutter/material.dart';
import 'package:stpvelox/domain/entities/wifi_credentials.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class WifiEnterpriseCredentialScreen extends StatefulWidget {
  final String ssid;

  const WifiEnterpriseCredentialScreen({super.key, required this.ssid});

  @override
  State<WifiEnterpriseCredentialScreen> createState() =>
      _WifiEnterpriseCredentialScreenState();
}

class _WifiEnterpriseCredentialScreenState
    extends State<WifiEnterpriseCredentialScreen> {
  final _usernameController = TextEditingController();
  final _passwordController = TextEditingController();
  final _caCertController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, 'Enterprise Credentials for ${widget.ssid}'),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: ListView(
          children: [
            const Text('Username:'),
            TextField(
              controller: _usernameController,
              decoration: const InputDecoration(border: OutlineInputBorder()),
            ),
            const SizedBox(height: 16),
            const Text('Password:'),
            TextField(
              controller: _passwordController,
              obscureText: true,
              decoration: const InputDecoration(border: OutlineInputBorder()),
            ),
            const SizedBox(height: 16),
            const Text('CA Certificate (optional):'),
            TextField(
              controller: _caCertController,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                hintText: '/path/to/ca.cert',
              ),
            ),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: () {
                final creds = EnterpriseCredentials(
                  username: _usernameController.text,
                  password: _passwordController.text,
                  caCertificatePath: _caCertController.text.isEmpty
                      ? null
                      : _caCertController.text,
                );
                Navigator.pop(context, creds);
              },
              child: const Text('Submit'),
            ),
          ],
        ),
      ),
    );
  }
}
