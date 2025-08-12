import 'package:flutter/material.dart';

class ResponsiveGrid extends StatelessWidget {
  final List<Widget> children;
  final double maxTileWidth;
  final double crossAxisSpacing;
  final double mainAxisSpacing;
  final EdgeInsetsGeometry padding;
  final double? childAspectRatio;
  final bool isScrollable;
  final int? crossAxisCount;

  const ResponsiveGrid({
    super.key,
    required this.children,
    this.maxTileWidth = 150,
    this.crossAxisSpacing = 16,
    this.mainAxisSpacing = 16,
    this.padding = const EdgeInsets.all(8.0),
    this.childAspectRatio,
    this.isScrollable = true,
    this.crossAxisCount,
  });

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        int calculatedCrossAxisCount =
            (constraints.maxWidth / (maxTileWidth + crossAxisSpacing)).floor();

        calculatedCrossAxisCount =
            calculatedCrossAxisCount > 0 ? calculatedCrossAxisCount : 1;

        return GridView.builder(
          padding: padding,
          gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: crossAxisCount ?? calculatedCrossAxisCount,
            crossAxisSpacing: crossAxisSpacing,
            mainAxisSpacing: mainAxisSpacing,
            childAspectRatio: childAspectRatio ?? 1,
          ),
          itemCount: children.length,
          itemBuilder: (context, index) => children[index],
          physics: isScrollable
              ? const AlwaysScrollableScrollPhysics()
              : const NeverScrollableScrollPhysics(),
          shrinkWrap: !isScrollable,
        );
      },
    );
  }
}