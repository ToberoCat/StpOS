import 'package:flutter/material.dart';
import 'package:stpvelox/domain/entities/sensor_category.dart';

class Sensor {
  final SensorCategory category;
  final String name;
  final Widget Function(Sensor) getSensorScreen;

  Sensor({
    required this.category,
    required this.name,
    required this.getSensorScreen,
  });

  get screen => getSensorScreen(this);
}
