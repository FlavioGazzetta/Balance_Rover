import 'dart:typed_data';
import 'package:flutter/material.dart';
import '../widgets/slider.dart';

class TrackingScreen extends StatefulWidget {
  const TrackingScreen({Key? key}) : super(key: key);

  @override
  _TrackingScreenState createState() => _TrackingScreenState();
}

class _TrackingScreenState extends State<TrackingScreen> {
  Uint8List? _frame;
  final List<int> _visibleIds = [1];
  int? _trackingId;

  void _onTap(int id) {
    if (id == _trackingId) return;
    setState(() => _trackingId = id);
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
              // 1) Top image
              Image.asset(
                'assets/images/image.jpg',
                width: 200,   // adjust to fit your layout
                height: 200,  // adjust as needed
                fit: BoxFit.contain,
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
