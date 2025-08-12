import 'package:stpvelox/domain/entities/setting.dart';

abstract class SettingsRepository {
  Future<List<Setting>> getSettings();
}
