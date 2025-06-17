import 'package:flutter/material.dart';
// Your imports for each mode:
import 'screens/chat.dart';
import 'screens/remote.dart';
import 'screens/track.dart';
import 'screens/weather.dart';

import 'package:http/http.dart' as http;

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

  // Map each index to its screen widget
  static final List<Widget> _modes = <Widget>[
    TrackingScreen(),     // 🚀 Livestream
    WeatherScreen(), // 🧪 Sensors
    JoystickScreen(),        // 🎮 Joystick
    AudioRecorderScreen(),   // 🎤 Audio
  ];

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
            // 1) Column with P and V
            Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.end,
              children: const [
                Text(
                  'P = 0W',
                  style: TextStyle(fontSize: 12),
                ),
                Text(
                  'V = 0.09V',
                  style: TextStyle(fontSize: 12),
                ),
              ],
            ),
            const SizedBox(width: 8),
            // 2) Battery icon
            const Icon(
              Icons.battery_full,
              size: 20,
            ),
            const SizedBox(width: 4),
            // 3) Percentage
            const Text(
              '99%',
              style: TextStyle(fontSize: 16),
            ),
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
