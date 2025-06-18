import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:http/http.dart' as http;
import 'package:path_provider/path_provider.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:audioplayers/audioplayers.dart';

Future<bool> requestMicAndSpeechPermissions() async {
  if (await Permission.microphone.request().isDenied) {
    return false;
  }
  if (await Permission.speech.request().isDenied) {
    return false;
  }
  return true;
}

class AudioRecorderScreen extends StatefulWidget {
  const AudioRecorderScreen({Key? key}) : super(key: key);

  @override
  _AudioRecorderScreenState createState() => _AudioRecorderScreenState();
}

class _AudioRecorderScreenState extends State<AudioRecorderScreen> {
  late stt.SpeechToText _speech;
  late AudioPlayer _audioPlayer;
  bool _isListening = false;
  bool _outputToSpeaker = true;
  String _recognizedText = '';

  @override
  void initState() {
    super.initState();
    _speech = stt.SpeechToText();
    _audioPlayer = AudioPlayer();
  }

  Future<void> _startListening() async {
    bool granted = await requestMicAndSpeechPermissions();
    if (!granted) return;
    bool available = await _speech.initialize(onError: (err) => debugPrint('Init error: $err'),);
    if (available) {
      setState(() {
        _isListening = true;
        _recognizedText = '';
      });
      _speech.listen(
        onResult: (val) {
          setState(() {
            _recognizedText = val.recognizedWords;
          });
        },
      );
    }
  }

  void _stopListening() {
    _speech.stop();
    setState(() {
      _isListening = false;
    });
  }

  Future<void> _sendTextToOpenAITTS(String text) async {
    final apiKey = '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';
    final url = Uri.parse('https://api.openai.com/v1/chat/completions');
    final response = await http.post(
      url,
      headers: {
        'Authorization': 'Bearer $apiKey',
        'Content-Type': 'application/json',
      },
      body: jsonEncode({
        'model': 'gpt-4o-audio-preview-2025-06-03',
        'modalities': ['text', 'audio'],
        'audio': {
          'voice': 'alloy',
          'format': 'mp3'
        },
        'messages': [
          {
            'role': 'user',
            'content': text
          }
        ]
      }),
    );

    if (response.statusCode == 200) {
    final Map<String, dynamic> json = jsonDecode(utf8.decode(response.bodyBytes));
    final String b64Audio = json['choices'][0]['message']['audio']['data'];
    final List<int> mp3Bytes = base64Decode(b64Audio);
    final dir = await getTemporaryDirectory();
    final file = File('${dir.path}/response.mp3');
    await file.writeAsBytes(mp3Bytes, flush: true);

    if (_outputToSpeaker) {
      await _audioPlayer.play(DeviceFileSource(file.path));
    } else {
      _sendFileToESP32(file);
    }
  } else {
    debugPrint('OpenAI API error: ${response.statusCode}');
  }
  }

  Future<void> _sendFileToESP32(File file) async {
    final espUrl = Uri.parse('http://172.20.10.11:80/upload'); // adjust IP/endpoint
    final request = http.MultipartRequest('POST', espUrl)
      ..files.add(await http.MultipartFile.fromPath('file', file.path));
    final streamedResponse = await request.send();

    if (streamedResponse.statusCode == 200) {
      debugPrint('Audio file sent to ESP32 successfully');
    } else {
      debugPrint('ESP32 upload failed: ${streamedResponse.statusCode}');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('AI Chat'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.end,
              children: [
                Column(
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    Switch(
                      value: _outputToSpeaker,
                      onChanged: (val) => setState(() => _outputToSpeaker = val),
                    ),
                    Text(
                      _outputToSpeaker ? 'Device' : 'Robot',
                      textAlign: TextAlign.center,
                    ),
                  ],
                ),
              ],
            ),
            const Spacer(),
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  IconButton(
                    iconSize: 64,
                    icon: Icon(
                      _isListening ? Icons.mic : Icons.mic_none,
                    ),
                    onPressed: _isListening ? _stopListening : _startListening,
                  ),
                  const SizedBox(height: 20),
                  Text(
                    _recognizedText.isEmpty
                        ? 'Tap mic and speak'
                        : 'You said: "$_recognizedText"',
                    textAlign: TextAlign.center,
                  ),
                  const SizedBox(height: 20),
                  if (!_isListening && _recognizedText.isNotEmpty) ...[
                    ElevatedButton(onPressed: () => _sendTextToOpenAITTS(_recognizedText), child: const Text('Confirm Send')),
                  ]
                ],
              ),
            ),
            const Spacer(),
          ],
        ),
      )
    );
  }
}
