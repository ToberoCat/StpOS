import 'package:flutter/material.dart';

enum SettingType { button, toggle }

class Setting {
  final IconData icon;
  final String label;
  final Color color;
  final SettingType type;
  final Function(BuildContext) onTap;
  final bool Function()? value;

  Setting({
    required this.icon,
    required this.label,
    required this.color,
    this.type = SettingType.button,
    required this.onTap,
    this.value,
  }) : assert(
          (type == SettingType.button && value == null) ||
              (type == SettingType.toggle && value != null ),
          'Invalid Setting configuration based on type.',
        );
}