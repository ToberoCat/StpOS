import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/setting.dart';
import 'package:stpvelox/presentation/blocs/settings/settings_bloc.dart';
import 'package:stpvelox/presentation/widgets/responsive_grid.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SettingsScreen extends StatelessWidget {
  const SettingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black87,
      appBar: createTopBar(context, "Settings"),
      body: SafeArea(
        child: Column(
          children: [
            Expanded(
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: BlocBuilder<SettingsBloc, SettingsState>(
                  builder: (context, state) {
                    if (state is SettingsLoading) {
                      return const Center(child: CircularProgressIndicator());
                    } else if (state is SettingsLoaded) {
                      final settings = state.settings;
                      return ResponsiveGrid(
                        children: settings.map((setting) {
                          return _buildSettingItem(
                            context,
                            setting,
                          );
                        }).toList(),
                      );
                    } else if (state is SettingsError) {
                      return Center(
                        child: Text(
                          state.message,
                          style:
                              const TextStyle(color: Colors.red, fontSize: 18),
                        ),
                      );
                    } else {
                      context.read<SettingsBloc>().add(LoadSettingsEvent());
                      return Container();
                    }
                  },
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSettingItem(BuildContext context, Setting setting) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.grey[800],
        borderRadius: BorderRadius.circular(8.0),
      ),
      child: setting.type == SettingType.button
          ? InkWell(
              borderRadius: BorderRadius.circular(8.0),
              onTap: () {
                context.read<SettingsBloc>().add(
                    SettingTappedEvent(setting: setting, context: context));
              },
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(setting.icon, size: 48, color: setting.color),
                  const SizedBox(height: 8),
                  Center(
                    child: Text(
                      setting.label,
                      style: const TextStyle(color: Colors.white, fontSize: 18),
                    ),
                  )
                ],
              ),
            )
          : InkWell(
              borderRadius: BorderRadius.circular(8.0),
              onTap: () {
                context.read<SettingsBloc>().add(
                      SettingTappedEvent(
                        setting: setting,
                        context: context,
                      ),
                    );
              },
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(setting.icon, size: 48, color: setting.color),
                    const SizedBox(height: 8),
                    Text(
                      setting.label,
                      style: const TextStyle(color: Colors.white, fontSize: 18),
                    ),
                    Switch(
                      value: setting.value!(),
                      onChanged: (newValue) {
                        context.read<SettingsBloc>().add(
                              SettingTappedEvent(
                                setting: setting,
                                context: context,
                              ),
                            );
                      },
                      activeColor: setting.color,
                    ),
                  ],
                ),
              ),
            ),
    );
  }
}
