import 'dart:async';
import 'dart:developer' as developer;

import 'package:flutter/material.dart';
import 'package:sleek_circular_slider/sleek_circular_slider.dart';
import 'package:stpvelox/data/native/kipr_plugin.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SensorMotorScreen extends StatefulWidget {
  final int port;
  final Sensor sensor;

  const SensorMotorScreen({
    super.key,
    required this.port,
    required this.sensor,
  });

  @override
  State<SensorMotorScreen> createState() => _SensorMotorScreenState();
}

class _SensorMotorScreenState extends State<SensorMotorScreen> {
  double _currentVelocity = 0.0;
  int _motorTicks = 0;
  Timer? _timer;
  final Duration _updateInterval = const Duration(milliseconds: 50);

  @override
  void initState() {
    super.initState();
    _startDataFetching();
  }

  void _startDataFetching() {
    _timer = Timer.periodic(_updateInterval, (timer) async {
      try {
        int position = await _getBAckEMFPosition();
        setState(() {
          _motorTicks = position;
        });
      } catch (e) {
        developer.log('Error fetching motor position: $e');
      }
    });
  }

  @override
  dispose() {
    _timer?.cancel();
    super.dispose();
  }

  Future<int> _getBAckEMFPosition() async =>
      await KiprPlugin.getMotorPosition(widget.port);

  Future<void> _stopMotor() async {
    await KiprPlugin.stopMotor(widget.port);
    setState(() {
      _currentVelocity = 0;
    });
  }

  Future<void> _setMotorVelocity(int velocity) async =>
      await KiprPlugin.setMotorVelocity(widget.port, velocity);

  void _onSliderChange(double value) {
    setState(() {
      _currentVelocity = value;
    });

        _setMotorVelocity(value.toInt());
  }

    void _onSliderChangeEnd(double value) {
    _setMotorVelocity(value.toInt());
  }

  @override
  Widget build(BuildContext context) {
    const double minValue = -1500;
    const double maxValue = 1500;

    return Scaffold(
      appBar: createTopBar(context, widget.sensor.name),
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            Column(
              children: [
                Text(
                  'Ticks: $_motorTicks',
                  style: const TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
            Expanded(
              child: Center(
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    Positioned(
                      bottom: -150,
                      child: SleekCircularSlider(
                        min: minValue,
                        max: maxValue,
                        initialValue: _currentVelocity,
                        onChange: _onSliderChange,
                        onChangeEnd: _onSliderChangeEnd,
                        appearance: CircularSliderAppearance(
                          startAngle: 180,
                          angleRange: 180,
                          customWidths: CustomSliderWidths(
                            trackWidth: 75,
                            progressBarWidth: 75,
                            handlerSize: 30,
                          ),
                          customColors: CustomSliderColors(
                            trackColor: Colors.grey.shade300,
                            progressBarColor: Colors.blue,
                            dotColor: Colors.white,
                            shadowColor: Colors.grey,
                            shadowMaxOpacity: 0.0,
                          ),
                          size: 400,
                          infoProperties: InfoProperties(
                            modifier: (double value) {
                              return '${value.toInt()}';
                            },
                            mainLabelStyle: const TextStyle(
                              fontSize: 32,
                              fontWeight: FontWeight.bold,
                              color: Colors.black,
                            ),
                          ),
                        ),
                        innerWidget: (velocity) {
                          return Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Text(
                                velocity.toStringAsFixed(0),
                                style: const TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                              const Text(
                                'Velocity',
                                style: TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.bold,
                                ),
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
            SizedBox(
              width: double.infinity,
              height: 70,
              child: ElevatedButton(
                onPressed: _stopMotor,
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.redAccent,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
                child: const Text(
                  'Stop',
                  style: TextStyle(
                    fontSize: 30,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
