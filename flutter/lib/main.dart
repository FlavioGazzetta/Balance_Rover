import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

import 'screens/chat.dart';
import 'screens/remote.dart';
import 'screens/track.dart';
import 'screens/weather.dart';

final esp32 = 'http://172.20.10.11:80';
final List mode_names = ['track', 'weather', 'remote', 'chat'];

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ESP32 Mode App',
      theme: ThemeData(useMaterial3: true),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});
  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  // 0: Stream, 1: Sensors, 2: Joystick, 3: Audio
  int _selectedIndex = 1;
  String? _power; 
  String? _percent; 

  late final Timer _timer;
  static const Duration _pollInterval = Duration(seconds: 1);

  @override
  void initState() {
    super.initState();
    // Fetch immediately, then on an interval
    _updateStatus();
    _timer = Timer.periodic(_pollInterval, (_) => _updateStatus());
  }

  static final List<Widget> _modes = <Widget>[
    TrackingScreen(),
    WeatherScreen(),
    JoystickScreen(),
    AudioRecorderScreen(),
  ];

  IconData _batteryIconFor(String? percent) {
    final pct = double.tryParse(percent ?? '')?.round();
    if (pct == null)               return Icons.battery_unknown;
    if (pct <= 5)   return Icons.battery_0_bar;
    if (pct <= 15)  return Icons.battery_1_bar;
    if (pct <= 30)  return Icons.battery_2_bar;
    if (pct <= 45)  return Icons.battery_3_bar;
    if (pct <= 60)  return Icons.battery_4_bar;
    if (pct <= 75)  return Icons.battery_5_bar;
    if (pct <= 90)  return Icons.battery_6_bar;
    return Icons.battery_full;
  }

  Color _batteryColorFor(String? percent, ThemeData theme) {
    final pct = double.tryParse(percent ?? '')?.round();
    if (pct == null)               return theme.colorScheme.onSurfaceVariant;
    if (pct <= 15)  return Colors.red;
    if (pct <= 40)  return Colors.amber;
    return Colors.green;
  }

  void _onDestinationSelected(int index) async {
    if (index == _selectedIndex) return;
    final uri = Uri.parse('$esp32/set_mode');
    final mode = mode_names[index];
    try {
      final response = await http.post(
        uri,
        body: {'mode': mode},
      );
      if (response.statusCode == 200) {
        setState(() {
          _selectedIndex = index;
        });
      } else {
        print('❌ ESP32 rejected mode switch: '
              '${response.statusCode} ${response.body}');
      }
    } catch (e) {
      print('⚠️ Error sending mode switch: $e');
    }
  }

  Future<void> _updateStatus() async {
    final uri = Uri.parse('$esp32/battery'); // adjust endpoint if needed
    try {
      final res = await http.get(uri).timeout(const Duration(seconds: 2));
      if (res.statusCode == 200) {
        final data = jsonDecode(res.body) as Map<String, dynamic>;
        if (!mounted) return;
        setState(() {
          _power   = (data['power'] as num?)?.toStringAsFixed(2);
          _percent = (data['percent'] as num?)?.toStringAsFixed(2);
        });
      } else {
        debugPrint('❌ Status GET failed: ${res.statusCode}');
      }
    } catch (e) {
      debugPrint('⚠️ Status poll error: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        titleSpacing: 0,
        // Use a Row to align everything on the right:
        title: Row(
          mainAxisAlignment: MainAxisAlignment.end,
          children: [
            if (_power != null) ...[
              Text(
                'P = ${_power}W',
                style: const TextStyle(fontSize: 12),
              ),
              const SizedBox(width: 4),
            ],
            // 2) Battery icon
            Icon(
              _batteryIconFor(_percent),
              size: 20,
              color: _batteryColorFor(_percent, Theme.of(context)),
            ),
            if (_percent != null) ...[
              const SizedBox(width: 4),
              Text(
                '$_percent%',
                style: const TextStyle(fontSize: 16),
              ),
            ],
            const SizedBox(width: 8),
          ],
        ),
      ),


      body: _modes[_selectedIndex],
      bottomNavigationBar: NavigationBar(
        selectedIndex: _selectedIndex,
        onDestinationSelected: _onDestinationSelected,
        destinations: const [
          NavigationDestination(
            icon: Icon(Icons.videocam),
            label: 'Stream',
          ),
          NavigationDestination(
            icon: Icon(Icons.monitor_heart),
            label: 'Sensors',
          ),
          NavigationDestination(
            icon: Icon(Icons.gamepad),
            label: 'Joystick',
          ),
          NavigationDestination(
            icon: Icon(Icons.mic),
            label: 'Audio',
          ),
        ],
      ),
    );
  }
}
