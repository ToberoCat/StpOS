import 'package:flutter/material.dart';

class LargeButtonCounter extends StatefulWidget {
  final double min;
  final double max;
  final double initial;
  final int divisions;
  final Function(double) onChanged;

  const LargeButtonCounter({
    super.key,
    required this.min,
    required this.max,
    required this.onChanged,
    this.initial = 0,
    this.divisions = 100,
  });

  @override
  State<LargeButtonCounter> createState() => _LargeButtonCounterState();
}

class _LargeButtonCounterState extends State<LargeButtonCounter> {
  late double _value;
  late double _step;

  @override
  void initState() {
    super.initState();
    _value = widget.initial.clamp(widget.min, widget.max);
    _step = (widget.max - widget.min) / widget.divisions;
  }

  void _increment() {
    setState(() {
      _value = (_value + _step).clamp(widget.min, widget.max);
    });
    widget.onChanged(_value);
  }

  void _decrement() {
    setState(() {
      _value = (_value - _step).clamp(widget.min, widget.max);
    });
    widget.onChanged(_value);
  }

  @override
  Widget build(BuildContext context) {
        double buttonSize = MediaQuery.of(context).size.width * 0.15;
    buttonSize = buttonSize < 60 ? 60 : buttonSize; 
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
                ElevatedButton(
          onPressed: _value > widget.min ? _decrement : null,
          style: ElevatedButton.styleFrom(
            shape: const CircleBorder(),
            padding: EdgeInsets.all(buttonSize * 0.2),
            minimumSize: Size(buttonSize, buttonSize),
            backgroundColor:
            _value > widget.min ? Colors.blue : Colors.grey,           ),
          child: const Icon(
            Icons.remove,
            size: 40,
            color: Colors.white,
          ),
        ),
        const SizedBox(width: 30),
                Container(
          padding: const EdgeInsets.symmetric(vertical: 20, horizontal: 40),
          decoration: BoxDecoration(
            color: Colors.grey[200],
            borderRadius: BorderRadius.circular(12),
            boxShadow: [
              BoxShadow(
                color: Colors.grey.withOpacity(0.5),
                spreadRadius: 2,
                blurRadius: 5,
                offset: const Offset(0, 3),               ),
            ],
          ),
          child: Text(
            _value.toStringAsFixed(1),
            style: const TextStyle(
              fontSize: 48,
              fontWeight: FontWeight.bold,
              color: Colors.black,
            ),
          ),
        ),
        const SizedBox(width: 30),
                ElevatedButton(
          onPressed: _value < widget.max ? _increment : null,
          style: ElevatedButton.styleFrom(
            shape: const CircleBorder(),
            padding: EdgeInsets.all(buttonSize * 0.2),
            minimumSize: Size(buttonSize, buttonSize),
            backgroundColor:
            _value < widget.max ? Colors.blue : Colors.grey,           ),
          child: const Icon(
            Icons.add,
            size: 40,
            color: Colors.white,
          ),
        ),
      ],
    );
  }
}