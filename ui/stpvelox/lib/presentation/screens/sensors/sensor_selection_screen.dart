import 'package:collection/collection.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/core/utils/colors.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/domain/entities/sensor_category.dart';
import 'package:stpvelox/presentation/blocs/sensor/sensor_bloc.dart';
import 'package:stpvelox/presentation/screens/sensors/sensor_category_screen.dart';
import 'package:stpvelox/presentation/widgets/grid_tile.dart';
import 'package:stpvelox/presentation/widgets/responsive_grid.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SensorSelectionScreen extends StatefulWidget {
  const SensorSelectionScreen({super.key});

  @override
  State<SensorSelectionScreen> createState() => _SensorSelectionScreenState();
}

class _SensorSelectionScreenState extends State<SensorSelectionScreen> {
  @override
  void initState() {
    super.initState();
    context.read<SensorBloc>().add(LoadSensorsEvent());
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, "Sensor Selection"),
      backgroundColor: Colors.black87,
      body: SafeArea(
        child: BlocBuilder<SensorBloc, SensorState>(
          builder: (context, state) {
            if (state is SensorLoading) {
              return const Center(child: CircularProgressIndicator());
            } else if (state is SensorLoaded) {
              final sensors = state.sensors;
              final sensoryByCategory =
              sensors.groupListsBy((sensor) => sensor.category);

              return ResponsiveGrid(
                isScrollable: false,
                children: sensoryByCategory.entries
                    .map((entry) => _buildSensorTile(entry.key, entry.value))
                    .toList(),
              );
            } else if (state is SensorError) {
              return Center(
                child: Text(
                  state.message,
                  style: const TextStyle(color: Colors.red, fontSize: 18),
                ),
              );
            } else {
              return Container();
            }
          },
        ),
      ),
    );
  }

  Widget _buildSensorTile(SensorCategory category, List<Sensor> sensor) {
    return ResponsiveGridTile(
      label: category.name,
      icon: Icons.auto_graph,
      onPressed: () {
        if (sensor.length == 1) {
          Navigator.of(context).push(
            MaterialPageRoute(
              builder: (_) => sensor.first.screen,
            ),
          );
          return;
        }

        Navigator.of(context).push(
        MaterialPageRoute(
          builder: (_) => SensorCategoryScreen(category: category, sensor: sensor),
        ),
      );
      },
      color: AppColors.getTileColor(category.index),
    );
  }
}