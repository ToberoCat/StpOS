import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/core/di/injection.dart' as di;
import 'package:stpvelox/core/utils/colors.dart';
import 'package:stpvelox/presentation/blocs/program/program_bloc.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_bloc.dart';
import 'package:stpvelox/presentation/blocs/sensor/sensor_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/settings_bloc.dart';
import 'package:stpvelox/presentation/blocs/settings/wifi/wifi_bloc.dart';
import 'package:stpvelox/presentation/screens/dashboard_screen.dart';
import 'package:flutter/gestures.dart';

import 'core/utils/touch_calibrator.dart';
import 'package:stpvelox/core/service/battery_check_service.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await di.init();
  await di.sl<TouchCalibrator>().loadCalibration();
  runApp(const StpVeloxApp());
}

class CalibratedTapGestureRecognizer extends TapGestureRecognizer {
  final TouchCalibrator calibrator;

  CalibratedTapGestureRecognizer({required this.calibrator, super.debugOwner});

  @override
  void addAllowedPointer(PointerDownEvent event) {
    final Offset calibratedLocalPosition = calibrator.applyCalibration(event.position);

    final PointerDownEvent calibratedEvent = event.copyWith(
      position: calibratedLocalPosition,
    );
    super.addAllowedPointer(calibratedEvent);
  }
}

class CalibratedGestureRecognizerFactory extends GestureRecognizerFactory<CalibratedTapGestureRecognizer> {
  final TouchCalibrator calibrator;

  CalibratedGestureRecognizerFactory({required this.calibrator});

  @override
  CalibratedTapGestureRecognizer constructor() {
    return CalibratedTapGestureRecognizer(calibrator: calibrator);
  }

  @override
  void initializer(CalibratedTapGestureRecognizer instance) {
      }
}

class StpVeloxApp extends StatefulWidget {
  const StpVeloxApp({super.key});

  @override
  State<StpVeloxApp> createState() => _StpVeloxAppState();
}

class _StpVeloxAppState extends State<StpVeloxApp> {
  late final BatteryCheckService _batteryCheckService;

  @override
  void initState() {
    super.initState();
    _batteryCheckService = di.sl<BatteryCheckService>();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _batteryCheckService.setContext(context);
    });
  }

  @override
  void dispose() {
    _batteryCheckService.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final TouchCalibrator calibrator = di.sl<TouchCalibrator>();

    return RawGestureDetector(
      gestures: {
        TapGestureRecognizer: CalibratedGestureRecognizerFactory(calibrator: calibrator),
      },
      child: MultiBlocProvider(
        providers: [
          BlocProvider(create: (_) => di.sl<SensorBloc>()),
          BlocProvider(create: (_) => di.sl<ProgramBloc>()),
          BlocProvider(create: (_) => di.sl<SettingsBloc>()),
          BlocProvider(create: (_) => di.sl<ProgramSelectionBloc>()),
          BlocProvider(create: (_) => di.sl<WifiBloc>())
        ],
        child: MaterialApp(
          title: 'stpvelox',
          theme: ThemeData(
            brightness: Brightness.dark,
            primaryColor: AppColors.programs,
            scaffoldBackgroundColor: AppColors.background,
            colorScheme: const ColorScheme.dark(
              primary: AppColors.programs,
              secondary: AppColors.settings,
              surface: AppColors.surface,
              error: Colors.redAccent,
              onPrimary: Colors.white,
              onSecondary: Colors.black,
              onSurface: Colors.white,
              onError: Colors.white,
            ),
            textTheme: const TextTheme(
              headlineLarge: TextStyle(color: Colors.white),
              bodyLarge: TextStyle(color: Colors.white70),
            ),
            elevatedButtonTheme: ElevatedButtonThemeData(
              style: ElevatedButton.styleFrom(
                foregroundColor: Colors.white,
                backgroundColor: AppColors.programs,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
            ),
          ),
          home: const DashboardScreen(),
        ),
      ),
    );
  }
}
