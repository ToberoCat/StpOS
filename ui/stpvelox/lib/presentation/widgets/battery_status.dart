import 'dart:async';

import 'package:flutter/material.dart';
import 'package:stpvelox/data/native/kipr_plugin.dart';

class BatteryStatus extends StatefulWidget {
  const BatteryStatus({super.key});

  @override
  State<BatteryStatus> createState() => _BatteryStatusState();
}

class _BatteryStatusState extends State<BatteryStatus> {
  double _batteryVoltage = 0.0;
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    _timer = Timer.periodic(const Duration(seconds: 5), (timer) {
      _updateBatteryVoltage();
    });
    _updateBatteryVoltage();
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  Future<void> _updateBatteryVoltage() async {
    final voltage = await KiprPlugin.getBatteryVoltage();
    setState(() {
      _batteryVoltage = voltage;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_batteryVoltage <= 0) {
      return const SizedBox.shrink();
    }

    return Padding(
      padding: const EdgeInsets.fromLTRB(0, 0, 8.0, 0),
      child: Row(
        children: [
          const Icon(
            Icons.battery_4_bar_rounded,
            color: Colors.white,
            size: 40,
          ),
          const SizedBox(width: 8),
          Text(
            '${_batteryVoltage.toStringAsFixed(2)}V',
            style: const TextStyle(
              color: Colors.white,
              fontSize: 20,
            ),
          ),
        ],
      ),
    );
  }
}
