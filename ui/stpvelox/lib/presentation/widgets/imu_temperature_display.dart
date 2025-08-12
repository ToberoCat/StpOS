
import 'dart:async';

import 'package:flutter/material.dart';
import 'package:stpvelox/data/native/kipr_plugin.dart';

class ImuTemperatureDisplay extends StatefulWidget {
  const ImuTemperatureDisplay({super.key});

  @override
  State<ImuTemperatureDisplay> createState() => _ImuTemperatureDisplayState();
}

class _ImuTemperatureDisplayState extends State<ImuTemperatureDisplay> {
  late Future<double> _temperatureFuture;
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    _temperatureFuture = KiprPlugin.getImuTemperature();
    _timer = Timer.periodic(const Duration(seconds: 5), (timer) {
      setState(() {
        _temperatureFuture = KiprPlugin.getImuTemperature();
      });
    });
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<double>(
      future: _temperatureFuture,
      builder: (context, snapshot) {
        if (snapshot.hasData) {
          return Text('${snapshot.data!.toStringAsFixed(1)}°C');
        } else if (snapshot.hasError) {
          return const Text('Error');
        }
        return const CircularProgressIndicator();
      },
    );
  }
}
