import 'package:flutter/material.dart';
import 'package:flutter/src/widgets/framework.dart';
import 'package:stpvelox/presentation/widgets/large_button.dart';

import 'arg.dart';

class NumberArg extends Arg {
  final double initial;
  final double min;
  final double max;
  final double step;


  NumberArg(String name, {
    required this.initial,
    required this.min,
    required this.max,
    required this.step,
  }) : super(type: 'number', name: name);


  factory NumberArg.fromJson(Map<String, dynamic> json) {
    return NumberArg(
      json['name'],
      initial: (json['initial'] ?? 0).toDouble(),
      min: (json['min'] ?? 0).toDouble(),
      max: (json['max'] ?? 100).toDouble(),
      step: (json['step'] ?? 1).toDouble(),
    );
  }

  @override
  Widget build(BuildContext context, Function(String p1) setValue) {
    setValue(initial.toString());
    return LargeButtonCounter(
      initial: initial,
      min: min,
      max: max,
      divisions: ((max - min) / step).toInt(),
      onChanged: (value) => setValue(value.toString()),
    );
  }
}