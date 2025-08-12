import 'package:flutter/material.dart';
import 'package:stpvelox/presentation/widgets/battery_status.dart';

AppBar createTopBar(BuildContext context, String title,
    {List<Widget>? actions, Widget? trailing}) {
  actions ??= [];
  actions.add(const BatteryStatus());
  return AppBar(
    backgroundColor: Colors.grey[900],
    automaticallyImplyLeading: false,
    actions: actions,
    title: Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        GestureDetector(
          onTap: () => Navigator.of(context).pop(),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              const SizedBox(width: 16),
              IconButton(
                icon: const Icon(Icons.arrow_back),
                onPressed: () => Navigator.of(context).pop(),
                iconSize: 40,
                color: Colors.white,
              ),
              const SizedBox(width: 10),
              Text(
                title,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
        ),
        if (trailing != null) trailing,
      ],
    ),
    toolbarHeight: 80,
  );
}
