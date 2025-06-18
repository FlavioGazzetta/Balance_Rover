import 'package:flutter/material.dart';
import 'package:geolocator/geolocator.dart';
import 'package:http/http.dart' as http;
import '../widgets/sensor.dart';
import 'dart:convert';

class WeatherScreen extends StatefulWidget {
  @override
  _WeatherScreenState createState() => _WeatherScreenState();
}

class _WeatherScreenState extends State<WeatherScreen> {
  double? serverLat;
  double? serverLng;
  double? serverTemp;
  double? serverHumidity;
  double? serverFeelsLike;

  double? espLat;
  double? espLng;
  double? espTemp;
  double? espHumidity;

  @override
  void initState() {
    super.initState();
    _fetchAllData();
  }

  Future<void> _fetchAllData() async {
    await Future.wait([
      _fetchServerData(),
      _fetchEspData(),
    ]);
  }
  
  Future<void> _reloadAll() async {
    await _fetchAllData();
  }

  Future<void> _fetchServerData() async {
    LocationPermission permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }
    try {
      Position position = await Geolocator.getCurrentPosition(
          desiredAccuracy: LocationAccuracy.high);
      serverLat = position.latitude;
      serverLng = position.longitude;
      final uri = Uri.parse('http://longfei.store:8000/api/cart/location/');
      final response = await http.post(
        uri,
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: {
          'id': 'imperial.ac.uk',
          'lat': serverLat.toString(),
          'lng': serverLng.toString(),
        },
      );
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        setState(() {
          serverTemp = (data['temperature'] as num).toDouble();
          serverHumidity = (data['humidity'] as num).toDouble();
          serverFeelsLike = (data['feels_like'] as num).toDouble();
        });
      }
    } catch (e) {
      print(e);
      print('Error getting server data');
    }
  }

  Future<void> _fetchEspData() async {
    try {
      final response = await http.get(
        Uri.parse('http://172.20.10.11/weather'),
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        if (data['lat'] != null) {
          espLat = (data['lat'] as num?)?.toDouble();
          espLng = (data['lng'] as num?)?.toDouble();
        }
        espTemp = (data['temperature'] as num).toDouble();
        espHumidity = (data['humidity'] as num).toDouble();
      } else {
        print('ESP32 error: \${response.statusCode}');
      }
    } catch (e) {
      print('Error fetching ESP32 data: \$e');
    }
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Weather UI'),
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Align(
              alignment: Alignment.centerRight,
              child: TextButton.icon(
                onPressed: _reloadAll,
                icon: Icon(Icons.refresh),
                label: Text('Refresh'),
                style: TextButton.styleFrom(
                  padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                ),
              ),
            ),
            SensorBlock(
              header: "Current Device",
              lat: serverLat,
              lng: serverLng,
              temperature: serverTemp,
              humidity: serverHumidity,
              feelsLike: serverFeelsLike,
              locationName: "South Kensington, London",
            ),
            SensorBlock(
              header: "Robot",
              lat: espLat,
              lng: espLng,
              temperature: espTemp,
              humidity: espHumidity,
              locationName: "South Kensington, London",
            ),
          ],
        ),
      ),
    );
  }
}
