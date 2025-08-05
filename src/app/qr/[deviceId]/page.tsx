"use client";

import { useParams } from "next/navigation";
import { useState, useEffect, useRef } from "react";
// NOTE: We load Firebase dynamically *only* in the browser to keep the
// Cloudflare Pages / Edge bundle free of Node-specific polyfills.

interface DeviceDetails {
  name: string;
  setLocation: string;
  setLocationLabel?: string;
  description?: string;
  referenceCode?: string;
}

interface SensorData {
  temp: number;
  humidity: number;
  accelerometer: string;
  currentLocation: string;
  batteryVoltage?: number;
  wakeUpReason?: string;
  timestamp?: number;
  bootCount?: number;
  altitude?: number;
  limitSwitchPressed?: boolean;
  locationBreach?: boolean;
}

interface DeviceData {
  details: DeviceDetails;
  sensorData: SensorData;
}


// Next.js route runtime configuration
export const runtime = "nodejs";

export default function QRDevicePage() {
  const params = useParams();
  const deviceId = params.deviceId as string;

  const [deviceData, setDeviceData] = useState<DeviceData | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [dataError, setDataError] = useState<string | null>(null);
  const [currentLocationText, setCurrentLocationText] = useState<string>("");
  const [setLocationText, setSetLocationText] = useState<string>("");
  const [accessTime, setAccessTime] = useState<string>("");
  const geocodeCache = useRef<Record<string, string>>({});


  // Set access time on client side only
  useEffect(() => {
    setAccessTime(new Date().toLocaleString());
  }, []);

  useEffect(() => {
    let unsubscribe: (() => void) | undefined;

    const initRealtime = async () => {
      // Dynamically import Firebase only in the browser
      const { getDatabase, ref, onValue } = await import("firebase/database");
      const { initializeApp } = await import("firebase/app");

      // Import shared config
      const { firebaseConfig } = await import("../../firebase");

      const app = initializeApp(firebaseConfig, "qr-viewer");
      const db = getDatabase(app);

      const deviceRef = ref(db, `tracking_box/${deviceId}`);

      unsubscribe = onValue(
        deviceRef,
        (snapshot) => {
          try {
            const data = snapshot.val();
            if (data) {
              setDeviceData({
                details: {
                  name: data.details?.name || "",
                  setLocation: data.details?.setLocation || "",
                  setLocationLabel: data.details?.setLocationLabel || "",
                  description: data.details?.description || "",
                  referenceCode: data.details?.referenceCode || "",
                },
                sensorData: {
                  temp: data.sensorData?.temp || 0,
                  humidity: data.sensorData?.humidity || 0,
                  accelerometer: (() => {
                    const raw = data.sensorData?.accelerometer;
                    if (typeof raw === "string") return raw;
                    if (raw && typeof raw === "object") {
                      if (raw.fallDetected) return "FALL DETECTED";
                      if (raw.tiltDetected) return "TILT DETECTED";
                      return "MOVING"; // generic non-normal motion
                    }
                    return "NORMAL";
                  })(),
                  currentLocation:
                    data.sensorData?.currentLocation || "No GPS Fix",
                  batteryVoltage: data.sensorData?.batteryVoltage || 0,
                  wakeUpReason:
                    data.sensorData?.wakeUpReason ||
                    data.sensorData?.wakeReason ||
                    "",
                  timestamp: data.sensorData?.timestamp || 0,
                  bootCount: data.sensorData?.bootCount || 0,
                  altitude: data.sensorData?.altitude || 0,
                  limitSwitchPressed:
                    data.sensorData?.limitSwitchPressed || false,
                  locationBreach: data.sensorData?.locationBreach || false,
                },
              });
              setDataError(null);
            } else {
              setDataError("Device not found");
            }
            setIsLoading(false);
          } catch (error) {
            console.error("Error loading device data:", error);
            setDataError("Failed to load device data");
            setIsLoading(false);
          }
        },
        (error) => {
          console.error("Firebase connection error:", error);
          setDataError("Connection error");
          setIsLoading(false);
        }
      );
    };

    if (deviceId && typeof window !== "undefined") {
      setIsLoading(true);
      initRealtime().catch((err) => {
        console.error("Firebase init error", err);
        setDataError("Failed to connect to database");
        setIsLoading(false);
      });
    }

    return () => {
      if (unsubscribe) unsubscribe();
    };
  }, [deviceId]);

  // Convert coordinates to human-readable addresses
  const parseCoordinates = (raw: string): [number, number] | null => {
    if (!raw || raw === "No GPS Fix" || raw === "GPS Initializing. Please Wait. . .") return null;
    
    // Try to parse decimal coordinates
    const match = raw.match(/(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)/);
    if (match) {
      return [parseFloat(match[1]), parseFloat(match[2])];
    }
    return null;
  };

  const fetchReverseGeocode = async (lat: number, lon: number): Promise<string> => {
    const key = `${lat.toFixed(4)},${lon.toFixed(4)}`;
    if (geocodeCache.current[key]) return geocodeCache.current[key];

    try {
      const url = `https://nominatim.openstreetmap.org/reverse?format=jsonv2&lat=${lat}&lon=${lon}&zoom=14&addressdetails=1`;
      const res = await fetch(url, {
        headers: { "User-Agent": "tracking-box-dashboard" },
      });
      const json = await res.json();
      
      let human = json.display_name || `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
      if (json.address) {
        const a = json.address;
        human = [
          a.road,
          a.suburb || a.village || a.town,
          a.city || a.municipality,
          a.state,
          a.country,
        ]
          .filter(Boolean)
          .join(", ");
      }
      geocodeCache.current[key] = human;
      return human;
    } catch (e) {
      console.warn("Reverse-geocode failed", e);
      return `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
    }
  };

  // Update location texts whenever device data changes
  useEffect(() => {
    if (!deviceData) return;

    const updateLocationTexts = async () => {
      // Convert current location
      const currentCoords = parseCoordinates(deviceData.sensorData.currentLocation);
      if (currentCoords) {
        const [lat, lon] = currentCoords;
        const address = await fetchReverseGeocode(lat, lon);
        setCurrentLocationText(address);
      } else {
        setCurrentLocationText(deviceData.sensorData.currentLocation);
      }

      // Convert set location
      const setCoords = parseCoordinates(deviceData.details.setLocation);
      if (setCoords) {
        const [lat, lon] = setCoords;
        const address = await fetchReverseGeocode(lat, lon);
        setSetLocationText(address);
      } else {
        setSetLocationText(deviceData.details.setLocation);
      }
    };

    updateLocationTexts();
  }, [deviceData]);


  const getStatusColor = (status: string | boolean) => {
    if (typeof status === "boolean") {
      return status ? "text-red-600" : "text-green-600";
    }
    return status === "NORMAL" ? "text-green-600" : "text-red-600";
  };


  return (
    <div className="min-h-screen bg-gray-50 py-8 px-4">
      <div className="max-w-4xl mx-auto">
        <div className="bg-white rounded-xl shadow-lg p-6 mb-6">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold text-gray-900">
                Device: {deviceId?.toUpperCase()}
              </h1>
              <p className="text-gray-600 mt-1">Offline Device Access Portal</p>
            </div>
            <div className="flex items-center gap-4">
              <div className="text-right">
                <p className="text-sm text-gray-500">Accessed via QR Code</p>
                {accessTime && (
                  <p className="text-xs text-gray-400">
                    {accessTime}
                  </p>
                )}
              </div>
            </div>
          </div>
        </div>

        {isLoading ? (
          <div className="bg-white rounded-xl shadow-lg p-8 text-center">
            <div className="animate-spin h-8 w-8 border-4 border-blue-600 border-t-transparent rounded-full mx-auto mb-4"></div>
            <p className="text-gray-600">Loading device data...</p>
          </div>
        ) : dataError ? (
          <div className="bg-white rounded-xl shadow-lg p-8 text-center">
            <svg
              className="h-12 w-12 text-red-500 mx-auto mb-4"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.854-.833-2.598 0L3.268 19c-.77.833.192 2.5 1.732 2.5z"
              />
            </svg>
            <p className="text-red-600 font-medium">{dataError}</p>
          </div>
        ) : deviceData ? (
          <div className="space-y-6">
            <div className="bg-white rounded-xl shadow-lg p-6">
              <h2 className="text-xl font-bold text-gray-900 mb-4">
                Device Information
              </h2>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Device Name</p>
                  <p className="font-semibold text-gray-900">
                    {deviceData.details.name || "Not Set"}
                  </p>
                </div>
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Reference Code</p>
                  <p className="font-semibold text-gray-900 font-mono">
                    {deviceData.details.referenceCode || "Not Set"}
                  </p>
                </div>
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Set Location</p>
                  <p className="font-semibold text-gray-900">
                    {setLocationText || deviceData.details.setLocation || "Not Set"}
                  </p>
                  {setLocationText && deviceData.details.setLocation && (
                    <p className="text-xs text-gray-500 mt-1">
                      {deviceData.details.setLocation}
                    </p>
                  )}
                </div>
              </div>
            </div>

            <div className="bg-white rounded-xl shadow-lg p-6">
              <h2 className="text-xl font-bold text-gray-900 mb-4">
                Current Sensor Data
              </h2>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                <div className="bg-red-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Temperature</p>
                  <p className="text-2xl font-bold text-red-600">
                    {deviceData.sensorData.temp}°C
                  </p>
                </div>
                <div className="bg-blue-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Humidity</p>
                  <p className="text-2xl font-bold text-blue-600">
                    {deviceData.sensorData.humidity}%
                  </p>
                </div>
                <div className="bg-green-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Battery</p>
                  <p className="text-2xl font-bold text-green-600">
                    {deviceData.sensorData.batteryVoltage?.toFixed(2)}V
                  </p>
                </div>
                <div className="bg-purple-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Current Location</p>
                  <p className="font-semibold text-purple-600">
                    {currentLocationText || deviceData.sensorData.currentLocation}
                  </p>
                  {currentLocationText && deviceData.sensorData.currentLocation !== "No GPS Fix" && (
                    <p className="text-xs text-purple-500 mt-1">
                      {deviceData.sensorData.currentLocation}
                    </p>
                  )}
                </div>
                <div className="bg-orange-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Motion Status</p>
                  <p
                    className={`font-semibold ${getStatusColor(
                      deviceData.sensorData.accelerometer
                    )}`}
                  >
                    {deviceData.sensorData.accelerometer}
                  </p>
                </div>
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Boot Count</p>
                  <p className="text-2xl font-bold text-gray-600">
                    {deviceData.sensorData.bootCount}
                  </p>
                </div>
              </div>
            </div>
          </div>
        ) : null}
      </div>
    </div>
  );
}
