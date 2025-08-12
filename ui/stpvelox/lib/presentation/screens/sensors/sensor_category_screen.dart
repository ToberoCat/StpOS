import 'dart:async';

import 'package:flutter/material.dart';
import 'package:stpvelox/core/utils/colors.dart';
import 'package:stpvelox/data/native/kipr_plugin.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/entities/sensor_category.dart';
import 'package:stpvelox/presentation/screens/flappy_bird_game.dart';
import 'package:stpvelox/presentation/widgets/grid_tile.dart';
import 'package:stpvelox/presentation/widgets/imu_temperature_display.dart';
import 'package:stpvelox/presentation/widgets/responsive_grid.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SensorCategoryScreen extends StatefulWidget {
  final SensorCategory category;
  final List<Sensor> sensor;

  const SensorCategoryScreen({
    super.key,
    required this.category,
    required this.sensor,
  });

  @override
  State<SensorCategoryScreen> createState() => _SensorCategoryScreenState();
}

class _SensorCategoryScreenState extends State<SensorCategoryScreen> {
    Future<void> _stopAllMotors() async {
    for (int i = 0; i < 4; i++) {
      await KiprPlugin.stopMotor(i);
    }
  }

  Future<void> _disableAllServos() async {
    await KiprPlugin.fullyDisableServos();
  }

    static const _holdDuration = Duration(seconds: 5);
  DateTime? _heldStart;
  Timer? _digital10Timer;
  int _prevDigital10 = 0;

  void _startListeningForDigital10Hold() {
    _digital10Timer =
        Timer.periodic(const Duration(milliseconds: 100), (_) async {
      final current = await KiprPlugin.getDigital(10);

      if (current == 1) {
        _heldStart ??= DateTime.now();         final heldTime = DateTime.now().difference(_heldStart!);
        if (heldTime >= _holdDuration) {
          _heldStart = null;           if (mounted) _openFlappyBirdGame();
        }
      } else {
        _heldStart = null;       }

      _prevDigital10 = current;
    });
  }

  void _openFlappyBirdGame() {
    Navigator.of(context).push(
      MaterialPageRoute(builder: (_) => const FlappyBirdGame()),
    );
  }

  @override
  void initState() {
    super.initState();
    if (widget.category.name == 'Digital') {
      _startListeningForDigital10Hold();
    }
  }

  @override
  void dispose() {
    _digital10Timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final bool isMotorCategory = widget.category.name == 'Motor';
    final bool isServoCategory = widget.category.name == 'Servo';
    final bool isDigitalCategory = widget.category.name == 'Digital';
    final bool isIMUCategory = widget.category.name == 'Gyro' ||
        widget.category.name == 'Accel' ||
        widget.category.name == 'Magneto';

    final actions = <Widget>[];
    if (isIMUCategory) {
      actions.add(ImuTemperatureDisplay());
    }

    return Scaffold(
      appBar: createTopBar(context, widget.category.name, actions: actions),
      body: Column(
        children: [
          Expanded(
            child: ResponsiveGrid(
              crossAxisCount: isDigitalCategory ? 5 : null,
              isScrollable: true,
              children: widget.sensor.asMap().entries.map((entry) {
                if (isDigitalCategory) {
                  return _DigitalSensorTile(
                    sensor: entry.value,
                    index: entry.key,
                  );
                }
                return ResponsiveGridTile(
                  label: entry.value.name,
                  icon: Icons.auto_graph,
                  onPressed: () {
                    Navigator.of(context).push(
                      MaterialPageRoute(builder: (_) => entry.value.screen),
                    );
                  },
                  color: AppColors.getTileColor(widget.category.index),
                );
              }).toList(),
            ),
          ),
          if (isMotorCategory)
            _CategoryActionButton(
              label: 'Stop All Motors',
              onPressed: _stopAllMotors,
            ),
          if (isServoCategory)
            _CategoryActionButton(
              label: 'Disable All Servos',
              onPressed: _disableAllServos,
            ),
        ],
      ),
    );
  }
}

class _DigitalSensorTile extends StatefulWidget {
  final Sensor sensor;
  final int index;

  const _DigitalSensorTile({required this.sensor, required this.index});

  @override
  State<_DigitalSensorTile> createState() => _DigitalSensorTileState();
}

class _DigitalSensorTileState extends State<_DigitalSensorTile> {
  late Future<int> _future;
  Timer? _timer;

  @override
  void initState() {
    super.initState();
    _future = KiprPlugin.getDigital(widget.index);
    _timer = Timer.periodic(const Duration(milliseconds: 200), (_) {
      if (mounted) {
        setState(() => _future = KiprPlugin.getDigital(widget.index));
      }
    });
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<int>(
      future: _future,
      builder: (context, snapshot) {
        final isClicked = snapshot.data ?? 0;
        return ResponsiveGridTile(
          label: widget.sensor.name,
          icon: Icons.auto_graph,
          onPressed: () {
            Navigator.of(context).push(
              MaterialPageRoute(builder: (_) => widget.sensor.screen),
            );
          },
          color: isClicked == 1 ? Colors.red : Colors.green,
        );
      },
    );
  }
}

class _CategoryActionButton extends StatelessWidget {
  final String label;
  final VoidCallback onPressed;

  const _CategoryActionButton({required this.label, required this.onPressed});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: SizedBox(
        width: double.infinity,
        height: 70,
        child: ElevatedButton(
          onPressed: onPressed,
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.redAccent,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          ),
          child: Text(
            label,
            style: const TextStyle(fontSize: 30, fontWeight: FontWeight.bold),
          ),
        ),
      ),
    );
  }
}
