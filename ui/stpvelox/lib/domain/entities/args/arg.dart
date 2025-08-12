import 'package:flutter/material.dart';
import 'package:stpvelox/domain/entities/args/bool_arg.dart';
import 'package:stpvelox/domain/entities/args/number_arg.dart';

abstract class Arg {
  final String type;
  final String name;

  Arg({required this.type, required this.name});

  Widget build(BuildContext context, Function(String) setValue);

  factory Arg.fromJson(Map<String, dynamic> json) {
    switch (json['type']) {
      case 'number':
        return NumberArg.fromJson(json);
      case 'bool':
        return BoolArg.fromJson(json);
      default:
        throw UnsupportedError('Unsupported argument type: ${json['type']}');
    }
  }
}
