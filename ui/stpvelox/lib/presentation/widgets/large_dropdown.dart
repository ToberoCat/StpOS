import 'package:flutter/material.dart';

class LargeDropdown extends StatefulWidget {
    final List<String> options;

    final String? initialSelected;

    final ValueChanged<String> onChanged;

    final String? hint;

    final String? label;

    final double fontSize;

    final double height;

    final double width;

    final Color backgroundColor;

    final Color textColor;

    final Color borderColor;

  const LargeDropdown({
    Key? key,
    required this.options,
    this.initialSelected,
    required this.onChanged,
    this.hint,
    this.label,
    this.fontSize = 18.0,
    this.height = 60.0,
    this.width = double.infinity,
    this.backgroundColor = Colors.white,
    this.textColor = Colors.black,
    this.borderColor = Colors.grey,
  }) : super(key: key);

  @override
  _LargeDropdownState createState() => _LargeDropdownState();
}

class _LargeDropdownState extends State<LargeDropdown> {
  String? _currentSelected;

  @override
  void initState() {
    super.initState();
        if (widget.initialSelected != null &&
        widget.options.contains(widget.initialSelected)) {
      _currentSelected = widget.initialSelected;
    } else if (widget.options.isNotEmpty) {
      _currentSelected = widget.options[0];
    } else {
      _currentSelected = null;
    }
  }

  @override
  void didUpdateWidget(covariant LargeDropdown oldWidget) {
    super.didUpdateWidget(oldWidget);
        if (!widget.options.contains(_currentSelected)) {
      if (widget.initialSelected != null &&
          widget.options.contains(widget.initialSelected)) {
        _currentSelected = widget.initialSelected;
      } else if (widget.options.isNotEmpty) {
        _currentSelected = widget.options[0];
      } else {
        _currentSelected = null;
      }
    }
  }

  @override
  Widget build(BuildContext context) {
        if (widget.options.isEmpty) {
      return Text(
        'No options available',
        style: TextStyle(fontSize: widget.fontSize, color: widget.textColor),
      );
    }

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
                if (widget.label != null) ...[
          Text(
            widget.label!,
            style: TextStyle(
              fontSize: widget.fontSize,
              fontWeight: FontWeight.bold,
              color: widget.textColor,
            ),
          ),
          SizedBox(height: 8.0),
        ],
        Container(
          height: widget.height,
          width: widget.width,
          padding: const EdgeInsets.symmetric(horizontal: 16.0),
          decoration: BoxDecoration(
            color: widget.backgroundColor,
            border: Border.all(color: widget.borderColor, width: 2.0),
            borderRadius: BorderRadius.circular(12.0),
          ),
          child: DropdownButtonHideUnderline(
            child: DropdownButton<String>(
              isExpanded: true,
              value: _currentSelected,
              hint: widget.hint != null
                  ? Text(
                widget.hint!,
                style: TextStyle(
                  fontSize: widget.fontSize,
                  color: Colors.grey,
                ),
              )
                  : null,
              icon: const Icon(Icons.arrow_drop_down),
              iconSize: 32.0,
              style: TextStyle(
                fontSize: widget.fontSize,
                color: widget.textColor,
              ),
              onChanged: (String? newValue) {
                if (newValue != null) {
                  setState(() {
                    _currentSelected = newValue;
                  });
                  widget.onChanged(newValue);
                }
              },
              items: widget.options
                  .map<DropdownMenuItem<String>>((String value) {
                return DropdownMenuItem<String>(
                  value: value,
                  child: Text(
                    value,
                    style: TextStyle(fontSize: widget.fontSize),
                  ),
                );
              }).toList(),
            ),
          ),
        ),
      ],
    );
  }
}
