import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class TouchCalibrator {
  static const _calibrationKey = 'touch_calibration';
  List<double>? _coefficients;

  List<double>? get coefficients => _coefficients;

  Future<void> loadCalibration() async {
    final prefs = await SharedPreferences.getInstance();
    final List<String>? storedCoeffs = prefs.getStringList(_calibrationKey);
    if (storedCoeffs != null && storedCoeffs.length == 6) {
      _coefficients = storedCoeffs.map(double.parse).toList();
    }
  }

  Offset applyCalibration(Offset raw) {
    if (_coefficients == null) {
      return raw;     }
    final c = _coefficients!;
    return Offset(
      c[0] * raw.dx + c[1] * raw.dy + c[2],
      c[3] * raw.dx + c[4] * raw.dy + c[5],
    );
  }
}
