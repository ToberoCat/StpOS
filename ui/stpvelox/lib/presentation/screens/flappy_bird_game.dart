import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:stpvelox/data/native/kipr_plugin.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';


enum GameState { ready, running, gameOver }

class FlappyBirdGame extends StatefulWidget {
  const FlappyBirdGame({super.key});

  @override
  State<FlappyBirdGame> createState() => _FlappyBirdGameState();
}

class _FlappyBirdGameState extends State<FlappyBirdGame>
    with SingleTickerProviderStateMixin {
  /*──────────────────────────────
  │  Gameplay state & scores
  └─────────────────────────────*/
  GameState _gameState = GameState.ready;
  int _score = 0;
  int _highScore = 0;

  /*──────────────────────────────
  │  Bird physics
  └─────────────────────────────*/
  double _birdY = 0;
  double _birdVelocity = 0;
  final double _gravity = 0.5;
  final double _jumpStrength = -10.0;
  final double _birdSize = 50.0;

  /*──────────────────────────────
  │  Pipes
  └─────────────────────────────*/
  final double _pipeWidth = 80.0;
  final double _pipeGap = 200.0;
  final double _pipeSpeed = 4.0;
  final List<Offset> _pipeOffsets = [];

  /*──────────────────────────────
  │  Game loop & Dimensions
  └─────────────────────────────*/
  late final AnimationController _controller;

      double _gameWidth = 0;
  double _gameHeight = 0;

  /*──────────────────────────────
  │  Hardware input (KIPR)
  └─────────────────────────────*/
  Timer? _sensorTimer;
  bool _lastSensorState = false;
  final Duration _pollInterval = const Duration(milliseconds: 50);
  final int _sensorPort = 10; 
  @override
  void initState() {
    super.initState();
    _loadHighScore();

    _controller = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 16),
    )..addListener(_gameLoop);

        _sensorTimer = Timer.periodic(_pollInterval, (_) => _pollSensor());
  }

  @override
  void dispose() {
    _sensorTimer?.cancel();
    _controller.dispose();
    super.dispose();
  }

  /*──────────────────────────────
  │  Persistent high‑score helpers
  └─────────────────────────────*/
  Future<void> _loadHighScore() async {
    final prefs = await SharedPreferences.getInstance();
    if (!mounted) return;
    setState(() {
      _highScore = prefs.getInt('highScore') ?? 0;
    });
  }

  Future<void> _saveHighScore() async {
    if (_score <= _highScore) return;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setInt('highScore', _score);
    _highScore = _score;
  }

  /*──────────────────────────────
  │  Hardware polling & tap simulation
  └─────────────────────────────*/
  Future<void> _pollSensor() async {
    bool current;
    try {
      current = await KiprPlugin.getDigital(_sensorPort) == 1;
    } catch (e) {
      current = false;
    }

    if (current && !_lastSensorState) _onTap();
    _lastSensorState = current;
  }

  /*──────────────────────────────
  │  Game control routines
  └─────────────────────────────*/
  void _resetGame() {
    setState(() {
      _gameState = GameState.ready;
      _birdY = 0;
      _birdVelocity = 0;
      _pipeOffsets.clear();
      _score = 0;
    });
  }

  void _startGame() {
    setState(() {
      _gameState = GameState.running;
      _pipeOffsets
        ..clear()
        ..addAll([
          _generatePipeOffset(_gameWidth + 100),
          _generatePipeOffset(_gameWidth + 100 + _gameWidth / 2),
        ]);
    });
    _controller.repeat();
  }

  Future<void> _gameOver() async {
    _controller.stop();
    await _saveHighScore();
    if (!mounted) return;
    setState(() => _gameState = GameState.gameOver);
  }

  void _jump() => _birdVelocity = _jumpStrength;

  void _onTap() {
    switch (_gameState) {
      case GameState.ready:
        _startGame();
        break;
      case GameState.running:
        _jump();
        break;
      case GameState.gameOver:
        _resetGame();
        break;
    }
  }

  /*──────────────────────────────
  │  Game loop & collisions
  └─────────────────────────────*/
  void _gameLoop() {
    if (_gameState != GameState.running) return;

    setState(() {
            _birdVelocity += _gravity;
      _birdY += _birdVelocity;

            for (var i = 0; i < _pipeOffsets.length; i++) {
        _pipeOffsets[i] = _pipeOffsets[i].translate(-_pipeSpeed, 0);
      }

            for (var offset in _pipeOffsets) {
        final pipeCenterX = offset.dx + _pipeWidth / 2;
        final birdCenterX = _gameWidth / 2;
        if (pipeCenterX <= birdCenterX &&
            pipeCenterX >= birdCenterX - _pipeSpeed) {
          _score++;
        }
      }

            if (_pipeOffsets.isNotEmpty && _pipeOffsets.first.dx < -_pipeWidth) {
        _pipeOffsets.removeAt(0);
        _pipeOffsets.add(_generatePipeOffset(_gameWidth));
      }

      _checkCollisions();
    });
  }

  void _checkCollisions() {
        final birdRect = Rect.fromLTWH(
      _gameWidth / 2 - _birdSize / 2,
      _birdY,
      _birdSize,
      _birdSize,
    );

        if (_birdY < 0) {
      _gameOver();
      return;
    }

        if (_birdY > _gameHeight - _birdSize) {
      _gameOver();
      return;
    }

        for (var offset in _pipeOffsets) {
      final topPipeHeight = offset.dy;
      final bottomPipeY = offset.dy + _pipeGap;

      final topPipe = Rect.fromLTWH(offset.dx, 0, _pipeWidth, topPipeHeight);
      final bottomPipe = Rect.fromLTWH(
          offset.dx, bottomPipeY, _pipeWidth, _gameHeight - bottomPipeY);

      if (birdRect.overlaps(topPipe) || birdRect.overlaps(bottomPipe)) {
        _gameOver();
        return;
      }
    }
  }

  /*──────────────────────────────
  │  Utility
  └─────────────────────────────*/
  Offset _generatePipeOffset(double x) {
    const minTop = 200.0;
        final maxTop = _gameHeight - _pipeGap - 200;
    final topHeight = minTop + Random().nextDouble() * (maxTop - minTop);
    return Offset(x, topHeight);
  }

  /*──────────────────────────────
  │  UI
  └─────────────────────────────*/
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, "Flappy Wombat"),
            body: LayoutBuilder(
        builder: (context, constraints) {
                    _gameWidth = constraints.maxWidth;
          _gameHeight = constraints.maxHeight;

          return Container(
            width: double.infinity,
            height: double.infinity,
            decoration: const BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topCenter,
                end: Alignment.bottomCenter,
                colors: [Colors.blue, Colors.lightBlueAccent],
              ),
            ),
            child: Stack(
              children: [
                                for (final offset in _pipeOffsets) ...[
                                    Positioned(
                    left: offset.dx,
                    top: 0,
                    child: _pipe(offset.dy),
                  ),
                                    Positioned(
                    left: offset.dx,
                    top: offset.dy + _pipeGap,
                                        child: _pipe(_gameHeight - offset.dy - _pipeGap),
                  ),
                ],

                                Positioned(
                  left: _gameWidth / 2 - _birdSize / 2,
                  top: _birdY,
                  child: SizedBox(
                    width: _birdSize,
                    height: _birdSize,
                    child: Image.asset('assets/wombat.png'),
                  ),
                ),

                                Positioned(
                  top: 50,
                  left: 0,
                  right: 0,
                  child: Center(
                    child: Text(
                      '$_score',
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 60,
                        fontWeight: FontWeight.bold,
                        shadows: [
                          Shadow(
                              blurRadius: 3,
                              color: Colors.black,
                              offset: Offset(2, 2))
                        ],
                      ),
                    ),
                  ),
                ),

                                if (_gameState == GameState.ready)
                  _overlayText('TAP BUTTON TO START'),
                if (_gameState == GameState.gameOver) _gameOverOverlay(),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _pipe(double height) => Container(
        width: _pipeWidth,
        height: height,
        decoration: BoxDecoration(
          color: Colors.green,
          border: Border.all(color: Colors.black, width: 2),
          borderRadius: BorderRadius.circular(5),
        ),
      );

  Widget _overlayText(String msg) => Center(
        child: Text(
          msg,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 30,
            fontWeight: FontWeight.bold,
          ),
        ),
      );

  Widget _gameOverOverlay() => Center(
        child: Container(
          padding: const EdgeInsets.all(20),
          decoration: BoxDecoration(
            color: Colors.black.withOpacity(0.7),
            borderRadius: BorderRadius.circular(15),
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                'GAME OVER',
                style: TextStyle(
                  color: Colors.red,
                  fontSize: 30,
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(height: 20),
              Text('Score: $_score',
                  style: const TextStyle(color: Colors.white, fontSize: 24)),
              Text('High Score: $_highScore',
                  style: const TextStyle(color: Colors.white, fontSize: 24)),
              const SizedBox(height: 20),
              const Text('Press button to play again',
                  style: TextStyle(color: Colors.white, fontSize: 18)),
            ],
          ),
        ),
      );
}
