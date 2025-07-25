"use client"; // Make sure this is a client component since it uses hooks

import React, { useState, useEffect } from 'react';
import { GoogleMap, Polyline, useLoadScript } from '@react-google-maps/api';

// === Config ===
const mapContainerStyle = {
  width: "100vw",
  height: "100vh",
};
const defaultCenter = { lat: 51.4985533, lng: -0.176324 };

// Endpoint serving *all* robot trajectories
const API_URL = "http://longfei.store:8000/api/all/";

// A palette of colors for different robots – repeat if we run out
const COLORS = ["#FF0000", "#00AAFF", "#00CC44", "#FFAA00", "#AA00FF", "#00FFAA"];

// Fallback dummy data (Imperial College London area) in case of connection failure
const DUMMY_ROBOTS = [
  [
    [51.4988, -0.17463],
    [51.4992, -0.1742],
    [51.4996, -0.1738],
  ],
  [
    [51.498, -0.175],
    [51.4975, -0.1745],
    [51.497, -0.174],
  ],
];

/**
 * Normalises the response coming back from `API_URL`.
 * Today the API returns an *array of arrays* of [lat, lng] tuples, e.g.
 *     [
 *       [ [51.498, -0.174], [51.499, -0.173] ],
 *       ...
 *     ]
 * If the backend changes its shape (e.g. wraps each robot in an object), we
 * attempt to extract the coordinates automatically so the UI won’t crash.
 */
function normaliseApiData(raw) {
  // Case 1: already the expected [[lat,lng], ...] [][] structure
  if (Array.isArray(raw) && Array.isArray(raw[0]) && Array.isArray(raw[0][0])) {
    return raw;
  }

  // Case 2: array of objects: [{path: [[lat,lng]]}, ...]
  if (Array.isArray(raw) && typeof raw[0] === "object" && raw[0] !== null) {
    return raw.map(r => {
      if (Array.isArray(r.path)) return r.path;
      if (Array.isArray(r.coordinates)) return r.coordinates;
      return [];
    });
  }

  // Unknown shape – fall back to dummy
  return DUMMY_ROBOTS;
}

const RobotMovementMap = () => {
  // robots: array of arrays of [lat, lng] tuples
  const [robots, setRobots] = useState(DUMMY_ROBOTS);
  // visible: array of booleans for each robot
  const [visible, setVisible] = useState(DUMMY_ROBOTS.map(() => true));

  // Fetch robot paths from server
  const fetchRobots = () => {
    fetch(API_URL)
      .then(res => {
        if (!res.ok) throw new Error(`Server responded ${res.status}`);
        return res.json();
      })
      .then(data => {
        const normalised = normaliseApiData(data);
        setRobots(normalised);
        // Preserve previously toggled visibility where possible
        setVisible(v => normalised.map((_, idx) => v[idx] ?? true));
      })
      .catch(err => {
        console.error("Robot API error – falling back to dummy data:", err);
        setRobots(DUMMY_ROBOTS);
        setVisible(DUMMY_ROBOTS.map(() => true));
      });
  };

  // Pull data initially and every 5 s
  useEffect(() => {
    fetchRobots();
    const interval = setInterval(fetchRobots, 5000);
    return () => clearInterval(interval);
  }, []);

  const { isLoaded, loadError } = useLoadScript({
    googleMapsApiKey: process.env.NEXT_PUBLIC_GOOGLE_MAPS_API_KEY || "",
    libraries: ["geometry"],
  });

  if (loadError) return <div>Error loading maps</div>;
  if (!isLoaded) return <div>Loading maps...</div>;

  // Toggle visibility of a robot’s path
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
    <div style={{ position: "relative" }}>
      {/* Checkbox controls */}
      <div
        style={{
          position: "absolute",
          top: 10,
          right: 10,
          zIndex: 100,
          backgroundColor: "rgba(255,255,255,0.9)",
          padding: 8,
          borderRadius: 4,
        }}
      >
        {robots.map((_, idx) => (
          <label key={idx} style={{ display: "block", marginBottom: 4 }}>
            <input
              type="checkbox"
              checked={visible[idx] || false}
              onChange={() => toggleVisibility(idx)}
            />{" "}
            Robot {idx + 1}
          </label>
        ))}
      </div>

      {/* Google Map with robot paths */}
      <GoogleMap mapContainerStyle={mapContainerStyle} zoom={16} center={defaultCenter}>
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
                    offset: "100%",
                    repeat: "30px",
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
