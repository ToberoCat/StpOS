import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:bloc/bloc.dart';
import 'package:equatable/equatable.dart';
import 'package:flutter/material.dart';

const String host = "127.0.0.1";
const int port = 9995;

abstract class CalibrationEvent extends Equatable {
  const CalibrationEvent();
  @override
  List<Object?> get props => [];
}

class StartCalibration extends CalibrationEvent {}

class UpdateFrame extends CalibrationEvent {}

class CalibratePotato extends CalibrationEvent {
  final Offset tapPosition;
  const CalibratePotato(this.tapPosition);
  @override
  List<Object?> get props => [tapPosition];
}

class CalibratePomRed extends CalibrationEvent {
  final Offset tapPosition;
  const CalibratePomRed(this.tapPosition);
  @override
  List<Object?> get props => [tapPosition];
}

class CalibratePomOrange extends CalibrationEvent {
  final Offset tapPosition;
  const CalibratePomOrange(this.tapPosition);
  @override
  List<Object?> get props => [tapPosition];
}

class CalibratePomYellow extends CalibrationEvent {
  final Offset tapPosition;
  const CalibratePomYellow(this.tapPosition);
  @override
  List<Object?> get props => [tapPosition];
}

class CalibrationDone extends CalibrationEvent {}

class CalibrationRetry extends CalibrationEvent {}

abstract class CalibrationState extends Equatable {
  const CalibrationState();
  @override
  List<Object?> get props => [];
}

class CalibrationInitial extends CalibrationState {}

class CalibrationInProgress extends CalibrationState {
  final String message;
  final String? frameBase64;
  const CalibrationInProgress(this.message, {this.frameBase64});
  @override
  List<Object?> get props => [message, frameBase64];
}

class CalibrationComplete extends CalibrationState {}

class CalibrationBloc extends Bloc<CalibrationEvent, CalibrationState> {
  Timer? _frameTimer;
    int _step = 0; 
  CalibrationBloc() : super(CalibrationInitial()) {
    on<StartCalibration>((event, emit) async {
            _frameTimer?.cancel();
      _frameTimer = Timer.periodic(const Duration(milliseconds: 50), (_) {
        add(UpdateFrame());
      });
      _step = 0;
      emit(const CalibrationInProgress("Please click the potato"));
    });

    on<UpdateFrame>((event, emit) async {
      final frame = await _getFrame();
      if (state is CalibrationInProgress) {
        final current = state as CalibrationInProgress;
        emit(CalibrationInProgress(current.message, frameBase64: frame));
      }
    });

    on<CalibratePotato>((event, emit) async {
      final response = await _sendCommand(
          "CAL_POTATO ${event.tapPosition.dx.toInt()} ${event.tapPosition.dy.toInt()}");
      emit(const CalibrationInProgress("Potato calibrated. Now click the red pom."));
      _step = 1;
    });

    on<CalibratePomRed>((event, emit) async {
      final response = await _sendCommand(
          "CAL_POM_RED ${event.tapPosition.dx.toInt()} ${event.tapPosition.dy.toInt()}");
      emit(const CalibrationInProgress("Red pom calibrated. Now click the orange pom."));
      _step = 2;
    });

    on<CalibratePomOrange>((event, emit) async {
      final response = await _sendCommand(
          "CAL_POM_ORANGE ${event.tapPosition.dx.toInt()} ${event.tapPosition.dy.toInt()}");
      emit(const CalibrationInProgress("Orange pom calibrated. Now click the yellow pom."));
      _step = 3;
    });

    on<CalibratePomYellow>((event, emit) async {
      final response = await _sendCommand(
          "CAL_POM_YELLOW ${event.tapPosition.dx.toInt()} ${event.tapPosition.dy.toInt()}");
      emit(const CalibrationInProgress("Yellow pom calibrated. Press Done if satisfied or Retry."));
    });

    on<CalibrationDone>((event, emit) async {
      _frameTimer?.cancel();
      emit(CalibrationComplete());
    });

    on<CalibrationRetry>((event, emit) async {
      _frameTimer?.cancel();
      add(StartCalibration());
    });
  }

  Future<String> _sendCommand(String command) async {
    try {
      Socket socket = await Socket.connect(host, port);
      socket.write("$command\n");
      await socket.flush();
      final response = await socket
          .cast<List<int>>()           .transform(utf8.decoder)
          .join();
      socket.destroy();
      return response;
    } catch (e) {
      return "ERROR: $e";
    }
  }

  Future<String?> _getFrame() async {
    try {
      Socket socket = await Socket.connect(host, port);
      socket.write("GET_FRAME\n");
      await socket.flush();
      
            final completer = Completer<String>();
      socket.listen(
        (data) {
          completer.complete(utf8.decode(data));
        },
        onError: (error) {
          if (!completer.isCompleted) completer.completeError(error);
        },
        onDone: () {
          if (!completer.isCompleted) completer.complete("");
        },
        cancelOnError: true,
      );
      
            String response;
      try {
        response = await completer.future.timeout(const Duration(seconds: 1));
      } catch (_) {
        socket.destroy();
        return null;
      }
      
      socket.destroy();
      if (response.startsWith("FRAME ")) {
        String base64Data = response.substring(6).trim();
                while (base64Data.length % 4 != 0) {
          base64Data += '=';
        }
        return base64Data;
      }
      return null;
    } catch (e) {
      return null;
    }
  }
}

