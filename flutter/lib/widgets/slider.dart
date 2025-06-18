import 'package:flutter/material.dart';

class IdTrackerBar extends StatelessWidget {
  final List<int> visibleIds;
  final int? trackingId;
  final ValueChanged<int> onChange;

  const IdTrackerBar({
    Key? key,
    required this.visibleIds,
    required this.trackingId,
    required this.onChange,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final idsToShow = List<int>.from(visibleIds);
    if (trackingId != null && !idsToShow.contains(trackingId)) {
      idsToShow.insert(0, trackingId!);
    }

    return SizedBox(
      height: 60,
      child: ListView.builder(
        scrollDirection: Axis.horizontal,
        itemCount: idsToShow.length,
        padding: const EdgeInsets.symmetric(horizontal: 8),
        itemBuilder: (context, index) {
          final id = idsToShow[index];
          final isSelected = id == trackingId;
          return Padding(
            padding: const EdgeInsets.symmetric(horizontal: 4),
            child: ChoiceChip(
              label: Text(id.toString()),
              selected: isSelected,
              onSelected: (_) => onChange(id),
              selectedColor: Colors.blueAccent,
              backgroundColor: Colors.grey[200],
              labelStyle: TextStyle(
                color: isSelected ? Colors.white : Colors.black87,
              ),
            ),
          );
        },
      ),
    );
  }
}
