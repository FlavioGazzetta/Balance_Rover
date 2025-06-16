'use client'; // Make sure this is a client component since it uses hooks

import React, { useState, useEffect } from 'react';
import { GoogleMap, Polyline, useLoadScript } from '@react-google-maps/api';

const mapContainerStyle = {
  width: '100vw',
  height: '100vh',
};
const defaultCenter = { lat: 51.4985533, lng: -0.176324 };
// A palette of colors for different robots
const COLORS = ['#FF0000', '#00AAFF', '#00CC44', '#FFAA00', '#AA00FF', '#00FFAA'];

// Dummy data (Imperial College London area) in case of connection failure
const DUMMY_ROBOTS = [
  [
    [51.498800, -0.174630],
    [51.499200, -0.174200],
    [51.499600, -0.173800],
  ],
  [
    [51.498000, -0.175000],
    [51.497500, -0.174500],
    [51.497000, -0.174000],
  ],
];

const RobotMovementMap = () => {
  // robots: array of arrays of [lat, lng] tuples
  const [robots, setRobots] = useState(DUMMY_ROBOTS);
  // visible: array of booleans for each robot
  const [visible, setVisible] = useState(DUMMY_ROBOTS.map(() => true));

  // Fetch robot paths from server
  const fetchRobots = () => {
    fetch('http://3.8.78.228:8000/api/cart/location/')
      .then(res => {
        if (!res.ok) {
          // Use dummy if server returns error
          throw new Error('Server error');
        }
        return res.json();
      })
      .then(data => {
        // Expect data as Array<Array<[lat, lng]>>
        setRobots(data);
        // Initialize visibility for new robots, preserving existing where possible
        setVisible(data.map((_, idx) => (visible[idx] !== undefined ? visible[idx] : true)));
      })
      .catch(() => {
        // On any error, fall back to dummy data
        setRobots(DUMMY_ROBOTS);
        setVisible(DUMMY_ROBOTS.map(() => true));
      });
  };

  useEffect(() => {
    fetchRobots();
    const intervalId = setInterval(fetchRobots, 5000);
    return () => clearInterval(intervalId);
  }, []);

  const { isLoaded, loadError } = useLoadScript({
    googleMapsApiKey: process.env.NEXT_PUBLIC_GOOGLE_MAPS_API_KEY || '',
    libraries: ['geometry'],
  });

  if (loadError) return <div>Error loading maps</div>;
  if (!isLoaded) return <div>Loading maps...</div>;

  // Toggle visibility of a robot's path
  const toggleVisibility = idx => {
    setVisible(v => {
      const next = [...v];
      next[idx] = !next[idx];
      return next;
    });
  };

  // Define an arrow symbol for movement direction
  const arrowSymbol = {
    path: window.google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
    scale: 2,
    strokeOpacity: 1,
  };

  return (
    <div style={{ position: 'relative' }}>
      {/* Checkbox controls */}
      <div
        style={{
          position: 'absolute',
          top: 10,
          right: 10,
          zIndex: 100,
          backgroundColor: 'rgba(255,255,255,0.9)',
          padding: 8,
          borderRadius: 4,
        }}
      >
        {robots.map((_, idx) => (
          <label key={idx} style={{ display: 'block', marginBottom: 4 }}>
            <input
              type="checkbox"
              checked={visible[idx] || false}
              onChange={() => toggleVisibility(idx)}
            />{' '}
            Robot {idx + 1}
          </label>
        ))}
      </div>

      {/* Google Map with robot paths */}
      <GoogleMap
        mapContainerStyle={mapContainerStyle}
        zoom={16}
        center={defaultCenter}
      >
        {robots.map((path, idx) =>
          visible[idx] && (
            <Polyline
              key={idx}
              path={path.map(([lat, lng]) => ({ lat, lng }))}
              options={{
                strokeColor: COLORS[idx % COLORS.length],
                strokeOpacity: 1,
                strokeWeight: 2,
                icons: [
                  {
                    icon: arrowSymbol,
                    offset: '100%',
                    repeat: '30px',
                  },
                ],
              }}
            />
          )
        )}
      </GoogleMap>
    </div>
  );
};

export default RobotMovementMap;
