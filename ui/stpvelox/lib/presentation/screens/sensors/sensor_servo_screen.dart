import 'dart:async';

import 'package:flutter/material.dart';
import 'package:sleek_circular_slider/sleek_circular_slider.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

import '../../../data/native/kipr_plugin.dart';

class ServoUtils {
  static const double minAngle = 0.0;
  static const double maxAngle = 170.0;
  static const double servoSpeedDps = 60 / 0.3;
  static const int minPosition = 0;
  static const int maxPosition = 2047;

  static int angleToPosition(double angle) {
    final clampedAngle = angle.clamp(minAngle, maxAngle);
    return ((clampedAngle / maxAngle) * maxPosition).round();
  }

  static double positionToAngle(int position) {
    final clampedPosition = position.clamp(minPosition, maxPosition);
    return (clampedPosition / maxPosition) * maxAngle;
  }

  static double estimateServoMoveTime(double startAngle, double endAngle) {
    final delta = (endAngle - startAngle).abs();
    return delta / servoSpeedDps;
  }
}

class SensorServoScreen extends StatefulWidget {
  final int port;
  final Sensor sensor;

  const SensorServoScreen({super.key, required this.port, required this.sensor});

  @override
  State<SensorServoScreen> createState() => _SensorServoScreenState();
}

class _SensorServoScreenState extends State<SensorServoScreen> {
  double _currentPosition = 0.0;
  bool _angleMode = true; 
    double get currentAngle => ServoUtils.positionToAngle(_currentPosition.toInt());

  Future<void> _setServoPosition(int position) async {
    await KiprPlugin.enableServo(widget.port);
    await KiprPlugin.setServoPosition(widget.port, position);
  }

  Future<void> _disableServo() async {
    await KiprPlugin.disableServo(widget.port);
  }

  void _onSliderChange(double value) {
    setState(() {
      if (_angleMode) {
                _currentPosition = ServoUtils.angleToPosition(value).toDouble();
      } else {
        _currentPosition = value;
      }
    });

        if (_angleMode) {
      _setServoPosition(ServoUtils.angleToPosition(value));
    } else {
      _setServoPosition(value.toInt());
    }
  }

  void _onSliderChangeEnd(double value) {
    if (_angleMode) {
            _setServoPosition(ServoUtils.angleToPosition(value));
    } else {
      _setServoPosition(value.toInt());
    }
  }

  void _toggleMode() {
    setState(() {
      _angleMode = !_angleMode;
    });
  }

  @override
  Widget build(BuildContext context) {
        final double minValue = _angleMode ? ServoUtils.minAngle : ServoUtils.minPosition.toDouble();
    final double maxValue = _angleMode ? ServoUtils.maxAngle : ServoUtils.maxPosition.toDouble();

        final double sliderValue = _angleMode
        ? ServoUtils.positionToAngle(_currentPosition.toInt())
        : _currentPosition;

        final modeToggle = Container(
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: ElevatedButton(
        onPressed: _toggleMode,
        style: ElevatedButton.styleFrom(
          backgroundColor: Colors.blue,
          foregroundColor: Colors.white,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(20),
          ),
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        ),
        child: Text(
          _angleMode ? 'Angle Mode' : 'Position Mode',
          style: const TextStyle(
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
    );

    return Scaffold(
      appBar: createTopBar(
        context,
        widget.sensor.name,
        trailing: modeToggle,
      ),
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            Expanded(
              child: Center(
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    Positioned(
                      bottom: -180,
                      child: SleekCircularSlider(
                        min: minValue,
                        max: maxValue,
                        initialValue: sliderValue,
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
                          size: 480,
                          infoProperties: InfoProperties(
                            modifier: (double value) {
                              return '${value.toInt()}${_angleMode ? '°' : ''}';
                            },
                            mainLabelStyle: const TextStyle(
                              fontSize: 32,
                              fontWeight: FontWeight.bold,
                              color: Colors.black,
                            ),
                          ),
                        ),
                        innerWidget: (value) {
                          return Column(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              Text(
                                _angleMode
                                    ? '${value.toStringAsFixed(1)}°'
                                    : value.toStringAsFixed(0),
                                style: const TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                              Text(
                                _angleMode ? 'Angle' : 'Position',
                                style: const TextStyle(
                                  fontSize: 24,
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                              const SizedBox(height: 20),
                              Text(
                                _angleMode
                                    ? 'Position: ${_currentPosition.toInt()}'
                                    : 'Angle: ${currentAngle.toStringAsFixed(1)}°',
                                style: const TextStyle(
                                  fontSize: 16,
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
              child: Row(
                children: [
                                    Expanded(
                    child: ElevatedButton(
                      onPressed: _disableServo,
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.redAccent,
                        shape: const RoundedRectangleBorder(
                          borderRadius: BorderRadius.only(
                            topLeft: Radius.circular(12),
                            bottomLeft: Radius.circular(12),
                          ),
                        ),
                      ),
                      child: const Text(
                        'Disable',
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
          ],
        ),
      ),
    );
  }
}

