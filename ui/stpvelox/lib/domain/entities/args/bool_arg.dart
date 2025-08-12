import 'package:flutter/material.dart';
import 'package:stpvelox/presentation/widgets/large_checkbox.dart';

import 'arg.dart';

class BoolArg extends Arg {
  final bool initial;

  BoolArg(String name, {required this.initial})
      : super(type: 'bool', name: name);

  factory BoolArg.fromJson(Map<String, dynamic> json) {
    return BoolArg(
      json['name'],
      initial: json['initial'] ?? false,
    );
  }

  @override
  Widget build(BuildContext context, Function(String) setValue) {
    setValue(initial.toString());
    return LargeCheckbox(
      initialValue: initial,
      onChanged: (value) => setValue(value.toString()),
    );
  }
}
