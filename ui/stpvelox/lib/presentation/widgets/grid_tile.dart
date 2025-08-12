import 'package:flutter/material.dart';

class ResponsiveGridTile extends StatelessWidget {
  final String label;
  final IconData icon;
  final void Function() onPressed;
  final Color color;

  const ResponsiveGridTile({
    super.key,
    required this.label,
    required this.icon,
    required this.onPressed,
    this.color = Colors.black12,
  });

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
        onTap: onPressed,
        child: Container(
          decoration: BoxDecoration(
            color: color,
            borderRadius: BorderRadius.circular(8.0),
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(
                icon,
                color: Colors.white,
                size: 100,
              ),
              const SizedBox(height: 8),
              Text(
                label,
                style: const TextStyle(color: Colors.white, fontSize: 18),
              )
            ],
          ),
        ),
      );
  }
}
