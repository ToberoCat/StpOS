import 'package:bloc/bloc.dart';
import 'package:equatable/equatable.dart';
import 'package:flutter/cupertino.dart';
import 'package:stpvelox/domain/entities/setting.dart';
import 'package:stpvelox/domain/repositories/settings_repository.dart';

part 'settings_event.dart';

part 'settings_state.dart';

class SettingsBloc extends Bloc<SettingsEvent, SettingsState> {
  final SettingsRepository repository;

  SettingsBloc({required this.repository}) : super(SettingsInitial()) {
    on<LoadSettingsEvent>((event, emit) async {
      emit(SettingsLoading());
      try {
        final settings = await repository.getSettings();
        emit(SettingsLoaded(settings: settings));
      } catch (e) {
        emit(SettingsError(message: e.toString()));
      }
    });

    on<SettingTappedEvent>((event, emit) async {
      emit(SettingsLoading());
      try {
        event.setting.onTap(event.context);
        add(LoadSettingsEvent());
      } catch (e) {
        emit(SettingsError(message: e.toString()));
      }
    });
  }
}
