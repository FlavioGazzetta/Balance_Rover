import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:http/http.dart' as http;
import 'package:path_provider/path_provider.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:audioplayers/audioplayers.dart';
import 'package:ffmpeg_kit_flutter_new/ffmpeg_kit.dart';
import 'package:flutter/services.dart' show rootBundle;

Future<bool> requestMicAndSpeechPermissions() async {
  if (await Permission.microphone.request().isDenied) {
    return false;
  }
  if (await Permission.speech.request().isDenied) {
    return false;
  }
  return true;
}

Future<String?> _fetchMemory() async {
  try {
    final res = await http.get(Uri.parse('http://longfei.store:8000/api/memory'));
    if (res.statusCode == 200) return res.body;
    debugPrint('Memory API failed: ${res.statusCode}');
  } catch (e) {
    debugPrint('Memory API error: $e');
  }
  return null;
}

Future<String> _loadRagContext() async =>
    await rootBundle.loadString('assets/rag.txt');

Future<File> _attenuateVolume(File input, double factor) async {
  final dir = await getTemporaryDirectory();
  final File out = File('${dir.path}/response_quiet.mp3');

  final cmd = '-y -i "${input.path}" -af "volume=$factor" "${out.path}"';
  final session = await FFmpegKit.execute(cmd);

  if ((await session.getReturnCode())?.isValueSuccess() != true) {
    debugPrint('ffmpeg failed: ${await session.getAllLogsAsString()}');
    return input;
  }
  return out;
}

Future<void> _postUpdate({required String prompt, required String response}) async {
  try {
    final req = http.MultipartRequest('POST', Uri.parse('http://longfei.store:8000/api/update'))
      ..fields['prompt'] = prompt
      ..fields['response'] = response;
    final res = await req.send();
    debugPrint('Update POST => ${res.statusCode}');
  } catch (e) {
    debugPrint('POST update error: $e');
  }
}

Future<void> _clearChatMemoryOnServer() async {
  try {
    final res = await http.get(Uri.parse('http://longfei.store:8000/api/clear'));
    debugPrint('Clear memory => ${res.statusCode}');
  } catch (e) {
    debugPrint('Clear memory error: $e');
  }
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
  bool _isSending = false;
  bool _isPlaying = false;
  bool _showResponseText = false;
  String _responseText = '';

  @override
  void initState() {
    super.initState();
    _speech = stt.SpeechToText();
    _audioPlayer = AudioPlayer();
    _audioPlayer.onPlayerComplete.listen((_) {
      setState(() {
        _isPlaying = false;
        _recognizedText = '';
      });
    });
  }

  @override
  void dispose() {
    _audioPlayer.dispose();
    super.dispose();
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
    final apiKey = 'test';
    final url = Uri.parse('https://api.openai.com/v1/chat/completions');
    setState(() => _isSending = true);
    try {
      final memory = await _fetchMemory() ?? '';
      final rag = await _loadRagContext();
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
            }, 
            {
              'role': 'system',
              'content': memory
            },
            {
              'role': 'system',
              'content': rag
            }
          ]
        }),
      );

      if (response.statusCode == 200) {
        final Map<String, dynamic> json = jsonDecode(utf8.decode(response.bodyBytes));
        final String b64Audio = json['choices'][0]['message']['audio']['data'];
        final String responseText = json['choices'][0]['message']['audio']['transcript'];
        setState(() => _responseText = responseText);
        _postUpdate(prompt: text, response: _responseText);
        final List<int> mp3Bytes = base64Decode(b64Audio);
        final dir = await getTemporaryDirectory();
        final file = File('${dir.path}/response.mp3');
        await file.writeAsBytes(mp3Bytes, flush: true);

        final File quietFile = await _attenuateVolume(file, 0.3);
        if (_outputToSpeaker) {
          await _audioPlayer.play(DeviceFileSource(file.path));
          setState(() => _isPlaying = true);
        } else {
          _sendFileToESP32(quietFile);
        }
      } else {
        debugPrint('OpenAI API error: ${response.statusCode}');
      }
    } finally {
      setState(() => _isSending = false);
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
    final loadingButton = ElevatedButton.icon(
      onPressed: null,
      icon: const SizedBox(
        width: 18,
        height: 18,
        child: CircularProgressIndicator(strokeWidth: 2),
      ),
      label: const Text('Sending…'),
    );

    final stopButton = ElevatedButton.icon(
      icon: const Icon(Icons.stop),
      label: const Text('Stop Audio'),
      onPressed: () async {
        await _audioPlayer.stop();
        setState(() => _isPlaying = false);
        _recognizedText = '';
      },
    );
    final confirmButton = ElevatedButton(
      onPressed: () => _sendTextToOpenAITTS(_recognizedText),
      child: const Text('Confirm Send'),
    );
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
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Row(children: [
                  Column(children: [
                    Switch(
                      value: _showResponseText,
                      onChanged: (val) => setState(() => _showResponseText = val),
                    ),
                    const Text('Output Text')
                  ],)
                  
                ]),
                TextButton.icon(
                  onPressed: () async {
                    await _clearChatMemoryOnServer();
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('Memory cleared ✅')),
                    );
                  },
                  icon: const Icon(Icons.delete_outline),
                  label: const Text('Clear chat memory'),
                ),
                Row(children: [
                  Column(children: [
                    Switch(
                      value: _outputToSpeaker,
                      onChanged: (val) => setState(() => _outputToSpeaker = val),
                    ),
                    Text(_outputToSpeaker ? 'Device' : 'Robot'),
                  ],)
                ]),
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
                  if (!_isListening && _showResponseText && _responseText.isNotEmpty)
                    Padding(
                      padding: const EdgeInsets.only(top: 12.0),
                      child: Text('AI: "$_responseText"', textAlign: TextAlign.center),
                    ),
                  const SizedBox(height: 20),
                  if (!_isListening) ...[
                    if (_isSending) loadingButton
                    else if (_isPlaying) stopButton
                    else if (_recognizedText.isNotEmpty) confirmButton
                  ],
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
