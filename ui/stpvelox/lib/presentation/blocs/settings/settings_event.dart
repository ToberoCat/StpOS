part of 'settings_bloc.dart';

abstract class SettingsEvent extends Equatable {
  const SettingsEvent();

  @override
  List<Object> get props => [];
}

class LoadSettingsEvent extends SettingsEvent {}

class SettingTappedEvent extends SettingsEvent {
  final Setting setting;
  final BuildContext context;

  const SettingTappedEvent({required this.setting, required this.context});

  @override
  List<Object> get props => [setting, context];
}
