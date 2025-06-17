import 'package:flutter/material.dart';

class SensorBlock extends StatelessWidget {
  final String header;
  final double? lat;
  final double? lng;
  final double? temperature;
  final double? humidity;
  final double? feelsLike;
  final String? locationName;

  const SensorBlock({
    Key? key,
    required this.header,
    this.lat,
    this.lng,
    this.temperature,
    this.humidity,
    this.feelsLike,
    this.locationName,
  }) : super(key: key);

  String _toDms(double value) {
    final d = value.truncate();
    final mDec = (value - d) * 60;
    final m = mDec.truncate();
    final s = ((mDec - m) * 60).round();
    return "$d°${m}'${s}\"";
  }

  @override
  Widget build(BuildContext context) {
    final hasGps = lat != null && lng != null;
    return Card(
      margin: const EdgeInsets.symmetric(vertical: 8),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      elevation: 4,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(header,
                style: Theme.of(context)
                    .textTheme
                    .headlineMedium),
            if (hasGps) ...[
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(
                    child: Text(locationName ?? '',
                        style: TextStyle(fontSize: 14)),
                  ),
                  Icon(Icons.satellite_alt, size: 14),
                  const SizedBox(width: 4),
                  Text('${_toDms(lat!)} , ${_toDms(lng!)}',
                      style:
                          TextStyle(fontSize: 12, color: Colors.grey[700])),
                ],
              ),
            ],
            const SizedBox(height: 12),
            Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(Icons.thermostat, size: 24),
                const SizedBox(width: 8),
                Text(
                  temperature != null
                      ? '${temperature!.toStringAsFixed(1)}°C'
                      : '--',
                  style: TextStyle(fontSize: 18),
                ),
                const SizedBox(width: 16),
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      humidity != null
                          ? 'Humidity: ${humidity!.toStringAsFixed(1)}%'
                          : 'Humidity: --',
                      style: TextStyle(fontSize: 12),
                    ),
                    if (feelsLike != null)
                      Text(
                        'Feels like: ${feelsLike!.toStringAsFixed(1)}°C',
                        style: TextStyle(fontSize: 12),
                      ),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
