import 'package:flutter/material.dart';
import '../widgets/slider.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'dart:convert';
import 'dart:io';
import 'package:http/http.dart' as http;

class TrackingScreen extends StatefulWidget {
  const TrackingScreen({Key? key}) : super(key: key);

  @override
  _TrackingScreenState createState() => _TrackingScreenState();
}

class _TrackingScreenState extends State<TrackingScreen> with WidgetsBindingObserver {
  List<int> _visibleIds = [1];
  int? _trackingId;
  final streamUrl = 'http://172.20.10.2:8000/stream';
  bool _isLive = true;
  late RawDatagramSocket _udp;

  void _onTap(int id) async {
    if (id == _trackingId) return;
    setState(() => _trackingId = id);
    final data = utf8.encode(_trackingId.toString());
    _udp.send(data, InternetAddress('172.20.10.2'), 7777);
    print('[UDP] sent $_trackingId → : RPI');
  }

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    RawDatagramSocket.bind(InternetAddress.anyIPv4, 9999)
      .then((sock) {
    _udp = sock;

    _udp.listen((RawSocketEvent evt) {
      if (evt == RawSocketEvent.read) {
        final datagram = _udp.receive();
        if (datagram == null) return;

        final payload = utf8.decode(datagram.data).trim();
        try {
          final List<dynamic> ids = jsonDecode(payload);
          setState(() => _visibleIds = ids.cast<int>());
        } catch (err) {
          print('[UDP] bad payload: $payload – $err');
        }
      }
    });
  });
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _udp.close();
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    setState(() => _isLive = state == AppLifecycleState.resumed);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Person Tracking')),
      body: Container(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              AspectRatio(
                aspectRatio: 1280 / 960,
                child: Mjpeg(
                  stream: streamUrl,
                  isLive: _isLive,
                  timeout: const Duration(seconds: 5),
                  error: (ctx, err, st) => Text(
                    'Stream error:\n$err',
                    style: const TextStyle(color: Colors.red),
                    textAlign: TextAlign.center,
                  ),
                ),
              ),

              const SizedBox(height: 16),

              // 2) ID Tracker bar with right-only fade
              Stack(
                children: [
                  // Bordered container with the bar
                  Container(
                    height: 60,
                    width: double.infinity,
                    decoration: BoxDecoration(
                      color: Colors.white,
                      border: Border.all(color: Colors.grey.shade400, width: 1.5),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: IdTrackerBar(
                      visibleIds: _visibleIds,
                      trackingId: _trackingId,
                      onChange: _onTap,
                    ),
                  ),

                  // Only right “more to scroll” fade
                  Positioned(
                    right: 0,
                    top: 0,
                    bottom: 0,
                    child: IgnorePointer(
                      child: Container(
                        width: 20,
                        decoration: BoxDecoration(
                          gradient: LinearGradient(
                            begin: Alignment.centerRight,
                            end: Alignment.centerLeft,
                            colors: [
                              Theme.of(context).scaffoldBackgroundColor,
                              Colors.transparent,
                            ],
                          ),
                          borderRadius: const BorderRadius.only(
                            topRight: Radius.circular(8),
                            bottomRight: Radius.circular(8),
                          ),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}
