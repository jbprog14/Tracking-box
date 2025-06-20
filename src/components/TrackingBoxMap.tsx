"use client";

import {useEffect, useState} from "react";
import dynamic from "next/dynamic";
import {LatLngExpression} from "leaflet";

// Dynamic import to avoid SSR issues with Leaflet
const MapContainer = dynamic(
  () => import("react-leaflet").then((mod) => mod.MapContainer),
  {ssr: false}
);
const TileLayer = dynamic(
  () => import("react-leaflet").then((mod) => mod.TileLayer),
  {ssr: false}
);
const Marker = dynamic(
  () => import("react-leaflet").then((mod) => mod.Marker),
  {ssr: false}
);
const Popup = dynamic(() => import("react-leaflet").then((mod) => mod.Popup), {
  ssr: false,
});

interface TrackingBoxMapProps {
  setLocation: string; // Manual location set via web app
  currentLocation: string; // GPS location from device
  boxId: string;
  boxName: string;
}

const TrackingBoxMap: React.FC<TrackingBoxMapProps> = ({
  setLocation,
  currentLocation,
  boxId,
  boxName,
}) => {
  const [mapReady, setMapReady] = useState(false);
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const [L, setL] = useState<any>(null);

  useEffect(() => {
    // Import Leaflet only on client side
    import("leaflet").then((leaflet) => {
      setL(leaflet);
      setMapReady(true);

      // Fix for missing marker icons in Next.js
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      delete (leaflet.Icon.Default.prototype as any)._getIconUrl;
      leaflet.Icon.Default.mergeOptions({
        iconRetinaUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png",
        iconUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png",
        shadowUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png",
      });
    });
  }, []);

  // Parse coordinates from string format "lat,lng"
  const parseCoordinates = (coordString: string): [number, number] | null => {
    if (!coordString || coordString === "No GPS Fix") return null;

    const coords = coordString.split(",");
    if (coords.length !== 2) return null;

    const lat = parseFloat(coords[0].trim());
    const lng = parseFloat(coords[1].trim());

    if (isNaN(lat) || isNaN(lng)) return null;
    return [lat, lng];
  };

  const setLocationCoords = parseCoordinates(setLocation);
  const currentLocationCoords = parseCoordinates(currentLocation);

  // Determine map center and zoom
  const getMapCenter = (): LatLngExpression => {
    if (currentLocationCoords) return currentLocationCoords;
    if (setLocationCoords) return setLocationCoords;
    return [40.7128, -74.006]; // Default to NYC
  };

  const getMapZoom = (): number => {
    if (currentLocationCoords || setLocationCoords) return 13;
    return 10;
  };

  // Create custom icons
  const createCustomIcon = (color: string) => {
    if (!L) return undefined;
    return new L.Icon({
      iconUrl: `https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-${color}.png`,
      shadowUrl:
        "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png",
      iconSize: [25, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowSize: [41, 41],
    });
  };

  if (!mapReady || !L) {
    return (
      <div className="w-full h-64 bg-gray-100 rounded-lg flex items-center justify-center">
        <div className="text-gray-500">Loading map...</div>
      </div>
    );
  }

  return (
    <div className="w-full h-64 rounded-lg overflow-hidden border border-gray-200">
      <link
        rel="stylesheet"
        href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"
        integrity="sha512-xodZBNTC5n17Xt2atTPuE1HxjVMSvLVW9ocqUKLsCC5CXdbqCmblAshOMAS6/keqq/sMZMZ19scR4PsZChSR7A=="
        crossOrigin=""
      />
      <MapContainer
        center={getMapCenter()}
        zoom={getMapZoom()}
        style={{height: "100%", width: "100%"}}
        scrollWheelZoom={true}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />

        {/* Set Location Marker (Blue) */}
        {setLocationCoords && (
          <Marker position={setLocationCoords} icon={createCustomIcon("blue")}>
            <Popup>
              <div className="text-center">
                <h3 className="font-bold text-blue-600">📍 Set Location</h3>
                <p className="text-sm font-medium">{boxName}</p>
                <p className="text-xs text-gray-600">ID: {boxId}</p>
                <p className="text-xs text-gray-500 mt-1">
                  Manually configured location
                </p>
                <p className="text-xs font-mono mt-1">
                  {setLocationCoords[0].toFixed(6)},{" "}
                  {setLocationCoords[1].toFixed(6)}
                </p>
              </div>
            </Popup>
          </Marker>
        )}

        {/* Current Location Marker (Red) */}
        {currentLocationCoords && (
          <Marker
            position={currentLocationCoords}
            icon={createCustomIcon("red")}
          >
            <Popup>
              <div className="text-center">
                <h3 className="font-bold text-red-600">
                  🛰️ Current GPS Location
                </h3>
                <p className="text-sm font-medium">{boxName}</p>
                <p className="text-xs text-gray-600">ID: {boxId}</p>
                <p className="text-xs text-gray-500 mt-1">
                  Live GPS coordinates from device
                </p>
                <p className="text-xs font-mono mt-1">
                  {currentLocationCoords[0].toFixed(6)},{" "}
                  {currentLocationCoords[1].toFixed(6)}
                </p>
              </div>
            </Popup>
          </Marker>
        )}
      </MapContainer>
    </div>
  );
};

export default TrackingBoxMap;
