import 'package:flutter/services.dart';
import 'dart:developer' as developer;

class KiprPlugin {
  static const MethodChannel _channel = MethodChannel('stpvelox.kipr');

  static Future<double> getOrientationRoll() async {
    try {
      final double roll = await _channel.invokeMethod<double>('orientationRoll') ?? 0;
      return roll;
    } on PlatformException catch (e) {
      developer.log("Failed to get orientation roll: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getOrientationPitch() async {
    try {
      final double pitch = await _channel.invokeMethod<double>('orientationPitch') ?? 0;
      return pitch;
    } on PlatformException catch (e) {
      developer.log("Failed to get orientation pitch: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getOrientationYaw() async {
    try {
      final double yaw = await _channel.invokeMethod<double>('orientationYaw') ?? 0;
      return yaw;
    } on PlatformException catch (e) {
      developer.log("Failed to get orientation yaw: '${e.message}'.");
      return 0;
    }
  }

    static Future<double> getGyroX() async {
    try {
      final double gyroXValue = await _channel.invokeMethod<double>('gyroX') ?? 0;
      return gyroXValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get gyroX: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getGyroY() async {
    try {
      final double gyroYValue = await _channel.invokeMethod<double>('gyroY') ?? 0;
      return gyroYValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get gyroY: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getGyroZ() async {
    try {
      final double gyroZValue = await _channel.invokeMethod<double>('gyroZ') ?? 0;
      return gyroZValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get gyroZ: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getAccelX() async {
    try {
      final double accelXValue = await _channel.invokeMethod<double>('accelX') ?? 0;
      return accelXValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get accelX: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getAccelY() async {
    try {
      final double accelYValue = await _channel.invokeMethod<double>('accelY') ?? 0;
      return accelYValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get accelY: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getAccelZ() async {
    try {
      final double accelZValue = await _channel.invokeMethod<double>('accelZ') ?? 0;
      return accelZValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get accelZ: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getMagX() async {
    try {
      final double magXValue = await _channel.invokeMethod<double>('magnetoX') ?? 0;
      return magXValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get magX: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getMagY() async {
    try {
      final double magYValue = await _channel.invokeMethod<double>('magnetoY') ?? 0;
      return magYValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get magY: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getMagZ() async {
    try {
      final double magZValue = await _channel.invokeMethod<double>('magnetoZ') ?? 0;
      return magZValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get magZ: '${e.message}'.");
      return 0;
    }
  }

    static Future<int> getAnalog(int port) async {
    try {
      final int analogValue = await _channel.invokeMethod<int>(
        'analog',
        {'port': port},
      ) ?? 0;
      return analogValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get analog value: '${e.message}'.");
      return 0;
    }
  }

  static Future<int> getDigital(int port) async {
    try {
      final int digitalValue = await _channel.invokeMethod<int>(
        'digital',
        {'port': port},
      ) ?? 0;
      return digitalValue;
    } on PlatformException catch (e) {
      developer.log("Failed to get digital value: '${e.message}'.");
      return 0;
    }
  }

  static Future<void> enableServo(int port) async {
    try {
      await _channel.invokeMethod(
        'servoEnable',
        {'port': port},
      );
    } on PlatformException catch (e) {
      developer.log("Failed to enable servo on port $port: '${e.message}'.");
    }
  }

  static Future<void> disableServo(int port) async {
    try {
      await _channel.invokeMethod(
        'servoDisable',
        {'port': port},
      );
    } on PlatformException catch (e) {
      developer.log("Failed to disable servo on port $port: '${e.message}'.");
    }
  }

  static Future<void> setServoPosition(int port, int position) async {
    try {
      await _channel.invokeMethod(
        'servoSetPosition',
        {
          'port': port,
          'position': position,
        },
      );
    } on PlatformException catch (e) {
      developer.log("Failed to set servo position on port $port: '${e.message}'.");
    }
  }

  static Future<void> stopMotor(int port) async {
    try {
      await _channel.invokeMethod(
        'motorStop',
        {'port': port},
      );
    } on PlatformException catch (e) {
      developer.log("Failed to stop motor on port $port: '${e.message}'.");
    }
  }

  static Future<void> setMotorVelocity(int port, int velocity) async {
    try {
      await _channel.invokeMethod(
        'motorVelocity',
        {
          'port': port,
          'velocity': velocity,
        },
      );
    } on PlatformException catch (e) {
      developer.log("Failed to set motor velocity on port $port: '${e.message}'.");
    }
  }

  static Future<int> getMotorPosition(int port) async {
    try {
      final int position = await _channel.invokeMethod<int>(
        'motorGetPosition',
        {'port': port},
      ) ?? 0;
      return position;
    } on PlatformException catch (e) {
      developer.log("Failed to get motor position on port $port: '${e.message}'.");
      return 0;
    }
  }

  static Future<void> fullyDisableServos() async {
    try {
      await _channel.invokeMethod('fullyDisableServos');
    } on PlatformException catch (e) {
      developer.log("Failed to fully disable servos: '${e.message}'.");
    }
  }

  static Future<double> getImuTemperature() async {
    try {
      final double temperature = await _channel.invokeMethod<double>('imuTemp') ?? 0;
      return temperature;
    } on PlatformException catch (e) {
      developer.log("Failed to get IMU temperature: '${e.message}'.");
      return 0;
    }
  }

  static Future<double> getBatteryVoltage() async {
    try {
      final double voltage = await _channel.invokeMethod<double>('battery') ?? 0;
      return voltage;
    } on PlatformException catch (e) {
      developer.log("Failed to get battery voltage: '${e.message}'.");
      return 0;
    }
  }

  static Future<void> setSpiMode(bool mode) {
    return _channel.invokeMethod('spiMode', {'mode': mode});
  }
}