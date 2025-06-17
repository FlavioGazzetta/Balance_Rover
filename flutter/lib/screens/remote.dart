import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/status.dart' as status;

class JoystickScreen extends StatefulWidget {
  @override
  _JoystickScreenState createState() => _JoystickScreenState();
}

class _JoystickScreenState extends State<JoystickScreen> {
  late final IOWebSocketChannel _channel;
  final String _esp32Ip = '172.20.10.11';  // your ESP32’s IP on the hotspot
  final String _wsPath  = '/ws';

  @override
  void initState() {
    super.initState();
    _channel = IOWebSocketChannel.connect(
      Uri.parse('ws://$_esp32Ip$_wsPath'),
    );
  }

  void _sendCommand(String cmd) {
    _channel.sink.add(cmd);
    debugPrint('Flutter → ESP32: $cmd');
  }

  @override
  void dispose() {
    _channel.sink.close(status.normalClosure);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Joystick Control')),
      body: Center(
        child: Joystick(
          includeInitialAnimation: false,
          base: JoystickBase(
            decoration: JoystickBaseDecoration(
              middleCircleColor: Colors.transparent,
              drawOuterCircle: true,
              drawInnerCircle: false,
            ),
            arrowsDecoration: JoystickArrowsDecoration(
              color: Colors.blue,
            ),
          ),
          mode: JoystickMode.horizontalAndVertical,
          listener: (details) {
            if (details.x > 0) {
              _sendCommand('right_start');
            } else if (details.x < 0) {
              _sendCommand('left_start');
            } else if (details.y < 0) {
              _sendCommand('up_start');
            } else if (details.y > 0) {
              _sendCommand('down_start');
            } else {
              _sendCommand('left_stop');
            }
          },
        ),
      ),
    );
  }
}
