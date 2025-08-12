import 'dart:async';
import 'dart:developer' as developer;
import 'dart:math';

import 'package:fl_chart/fl_chart.dart';
import 'package:flutter/material.dart';
import 'package:stpvelox/domain/entities/sensor.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class SensorGraphScreen extends StatefulWidget {
  final Sensor sensor;
  final Future<num> Function() getSensorValue;   final double graphMin;
  final double graphMax;

  const SensorGraphScreen({
    super.key,
    required this.sensor,
    required this.getSensorValue,
    required this.graphMin,
    required this.graphMax,
  });

  @override
  State<SensorGraphScreen> createState() => _SensorGraphScreenState();
}

class _SensorGraphScreenState extends State<SensorGraphScreen> {
  Timer? _timer;
  final List<double> _dataPoints = [];
  final List<double> _movingAvgPoints = [];
  final int _maxPoints = 50;
  final Duration _updateInterval = const Duration(milliseconds: 50);

  double _average = 0.0;
  double _min = 0;
  double _max = 0;
  double _stdDev = 0.0;

  final int _movingAvgWindow = 10;

  @override
  void initState() {
    super.initState();
    _startDataFetching();
  }

  void _startDataFetching() {
    _timer = Timer.periodic(_updateInterval, (timer) async {
      try {
        num value = await widget.getSensorValue();         setState(() {
          _dataPoints.add(value.toDouble());           if (_dataPoints.length > _maxPoints) {
            _dataPoints.removeAt(0);
          }
          _calculateMetrics();
          _calculateMovingAverage();
        });
      } catch (e) {
        developer.log('Error fetching sensor value: $e');
      }
    });
  }

  void _calculateMetrics() {
    if (_dataPoints.isEmpty) return;

    _average = _dataPoints.reduce((a, b) => a + b) / _dataPoints.length;
    _min = _dataPoints.reduce(min);
    _max = _dataPoints.reduce(max);

    double sumSquaredDiffs = _dataPoints
        .map((value) => pow(value - _average, 2))
        .reduce((a, b) => a + b)
        .toDouble();
    _stdDev = sqrt(sumSquaredDiffs / _dataPoints.length);
  }

  void _calculateMovingAverage() {
    if (_dataPoints.length < _movingAvgWindow) {
      _movingAvgPoints.add(_average);
    } else {
      double windowSum = _dataPoints
          .sublist(_dataPoints.length - _movingAvgWindow)
          .reduce((a, b) => a + b)
          .toDouble();
      double movingAvg = windowSum / _movingAvgWindow;
      _movingAvgPoints.add(movingAvg);
      if (_movingAvgPoints.length > _maxPoints) {
        _movingAvgPoints.removeAt(0);
      }
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, "${widget.sensor.name} Graph"),
      backgroundColor: Colors.black87,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: _buildGraph(),
        ),
      ),
    );
  }

  Widget _buildGraph() {
    if (_dataPoints.isEmpty) {
      return const Center(
        child: Text(
          "Fetching data...",
          style: TextStyle(color: Colors.white, fontSize: 20),
        ),
      );
    }

    List<FlSpot> rawSpots = _dataPoints.asMap().entries.map((entry) {
      int index = entry.key;
      double y = entry.value;
      return FlSpot(index.toDouble(), y);
    }).toList();

    List<FlSpot> movingAvgSpots = _movingAvgPoints.asMap().entries.map((entry) {
      int index = entry.key;
      double y = entry.value;
      return FlSpot(index.toDouble(), y);
    }).toList();

        double minY = widget.graphMin;
    double maxY = widget.graphMax;

    return Column(
      children: [
        Expanded(
          child: RepaintBoundary(
            child: LineChart(
              duration: Duration.zero,
              LineChartData(
                clipData: const FlClipData.all(),
                minX: 0,
                maxX: (_maxPoints - 1).toDouble(),
                minY: minY,
                maxY: maxY,
                gridData: FlGridData(
                  show: true,
                  drawVerticalLine: true,
                  horizontalInterval: 5,
                  verticalInterval: 5,
                  getDrawingHorizontalLine: (value) {
                    return FlLine(
                      color: Colors.grey.withOpacity(0.3),
                      strokeWidth: 1,
                    );
                  },
                  getDrawingVerticalLine: (value) {
                    return FlLine(
                      color: Colors.grey.withOpacity(0.3),
                      strokeWidth: 1,
                    );
                  },
                ),
                titlesData: FlTitlesData(
                  show: true,
                  rightTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false),
                  ),
                  topTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false),
                  ),
                  leftTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 40,
                      interval: max(5, (maxY - minY) / 5),
                      getTitlesWidget: (value, meta) {
                        return Text(
                          value.toInt().toString(),
                          style: const TextStyle(
                              color: Colors.white, fontSize: 12),
                        );
                      },
                    ),
                  ),
                  bottomTitles: AxisTitles(
                    sideTitles: SideTitles(
                      showTitles: true,
                      reservedSize: 30,
                      interval: 5,
                      getTitlesWidget: (value, meta) {
                        if (value.toInt() % 5 == 0 ||
                            value.toInt() == _maxPoints - 1) {
                          return Text(
                            value.toInt().toString(),
                            style: const TextStyle(
                                color: Colors.white, fontSize: 12),
                          );
                        }
                        return Container();
                      },
                    ),
                  ),
                ),
                borderData: FlBorderData(
                  show: true,
                  border: const Border(
                    left: BorderSide(color: Colors.white),
                    bottom: BorderSide(color: Colors.white),
                    right: BorderSide(color: Colors.transparent),
                    top: BorderSide(color: Colors.transparent),
                  ),
                ),
                lineBarsData: [
                  LineChartBarData(
                    spots: rawSpots,
                    isCurved: true,
                    color: Colors.blueAccent,
                    barWidth: 2,
                    isStrokeCapRound: true,
                    preventCurveOverShooting: true,
                    dotData: const FlDotData(
                      show: false,
                    ),
                    belowBarData: BarAreaData(
                      show: true,
                      color: Colors.blueAccent.withOpacity(0.1),
                    ),
                  ),
                  if (_movingAvgPoints.isNotEmpty)
                    LineChartBarData(
                      spots: movingAvgSpots,
                      isCurved: true,
                      color: Colors.orangeAccent,
                      barWidth: 2,
                      isStrokeCapRound: true,
                      preventCurveOverShooting: true,
                      dotData: const FlDotData(
                        show: false,
                      ),
                      belowBarData: BarAreaData(
                        show: false,
                      ),
                    ),
                ],
                lineTouchData: LineTouchData(
                  enabled: true,
                  touchTooltipData: LineTouchTooltipData(
                    getTooltipItems: (touchedSpots) {
                      return touchedSpots.map((spot) {
                        String label = spot.bar.color == Colors.blueAccent
                            ? 'Value: ${spot.y.toInt()}'
                            : 'Moving Avg: ${spot.y.toStringAsFixed(2)}';
                        return LineTooltipItem(
                          label,
                          const TextStyle(color: Colors.white),
                        );
                      }).toList();
                    },
                  ),
                ),
              ),
            ),
          ),
        ),
        const SizedBox(height: 16),
        _buildMetricsPanel(),
      ],
    );
  }

  Widget _buildMetricsPanel() {
    return Container(
      padding: const EdgeInsets.all(12.0),
      decoration: BoxDecoration(
        color: Colors.grey[900],
        borderRadius: BorderRadius.circular(8.0),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _buildMetricItem('Avg', _average.toStringAsFixed(2)),
          _buildMetricItem('Min', _min.toStringAsFixed(2)),
          _buildMetricItem('Max', _max.toStringAsFixed(2)),
          _buildMetricItem('Std', _stdDev.toStringAsFixed(2)),
        ],
      ),
    );
  }

  Widget _buildMetricItem(String label, String value) {
    return Column(
      children: [
        Text(
          label,
          style: const TextStyle(
            color: Colors.grey,
            fontSize: 14,
          ),
        ),
        const SizedBox(height: 4),
        Text(
          value,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 16,
            fontWeight: FontWeight.bold,
          ),
        ),
      ],
    );
  }
}
