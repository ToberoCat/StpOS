import 'package:flutter/material.dart';

class AppColors {
  static const Color sensors = Colors.blueAccent;
  static const Color programs = Colors.greenAccent;
  static const Color settings = Colors.orangeAccent;
  static const Color background = Colors.black;
  static const Color surface = Colors.grey;

  static const List<Color> tileColors = [
    Colors.blue,
    Colors.green,
    Colors.orange,
    Colors.purple,
    Colors.red,
    Colors.teal,
    Colors.pink,
    Colors.indigo,
    Colors.amber,
    Colors.cyan,
  ];

  static Color getTileColor(int index) => tileColors[index % tileColors.length];
}
