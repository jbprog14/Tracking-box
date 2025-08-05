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
  
  // Package Information Fields
  packDate?: string;
  packWeight?: string;
  productFrom?: string;
  packerShipper?: string;
  supplierIdTracking?: string;
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
                  packDate: data.details?.packDate || "",
                  packWeight: data.details?.packWeight || "",
                  productFrom: data.details?.productFrom || "",
                  packerShipper: data.details?.packerShipper || "",
                  supplierIdTracking: data.details?.supplierIdTracking || "",
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

            {/* Package Information Section */}
            {(deviceData.details.packDate || deviceData.details.packWeight || 
              deviceData.details.productFrom || deviceData.details.packerShipper || 
              deviceData.details.supplierIdTracking) && (
              <div className="bg-gradient-to-br from-orange-50 to-amber-50 rounded-xl shadow-lg p-6 border border-orange-200">
                <h2 className="text-xl font-bold text-gray-900 mb-4 flex items-center gap-2">
                  <svg className="h-6 w-6 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
                  </svg>
                  Package Information
                </h2>
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                  {deviceData.details.packDate && (
                    <div className="bg-white bg-opacity-80 p-4 rounded-lg border border-orange-200">
                      <p className="text-sm text-gray-600 flex items-center gap-1">
                        <svg className="h-4 w-4 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 7V3m8 4V3m-9 8h10M5 21h14a2 2 0 002-2V7a2 2 0 00-2-2H5a2 2 0 00-2 2v12a2 2 0 002 2z" />
                        </svg>
                        Pack Date
                      </p>
                      <p className="font-semibold text-gray-900">
                        {new Date(deviceData.details.packDate).toLocaleDateString()}
                      </p>
                    </div>
                  )}
                  {deviceData.details.packWeight && (
                    <div className="bg-white bg-opacity-80 p-4 rounded-lg border border-orange-200">
                      <p className="text-sm text-gray-600 flex items-center gap-1">
                        <svg className="h-4 w-4 text-yellow-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 6l3 1m0 0l-3 9a5.002 5.002 0 006.001 0M6 7l3 9M6 7l6-2m6 2l3-1m-3 1l-3 9a5.002 5.002 0 006.001 0M18 7l3 9m-3-9l-6-2m0-2v2m0 16V5m0 16H9m3 0h3" />
                        </svg>
                        Pack Weight
                      </p>
                      <p className="font-semibold text-gray-900">
                        {deviceData.details.packWeight}
                      </p>
                    </div>
                  )}
                  {deviceData.details.productFrom && (
                    <div className="bg-white bg-opacity-80 p-4 rounded-lg border border-orange-200">
                      <p className="text-sm text-gray-600 flex items-center gap-1">
                        <svg className="h-4 w-4 text-red-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M17.657 16.657L13.414 20.9a1.998 1.998 0 01-2.827 0l-4.244-4.243a8 8 0 1111.314 0z" />
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 11a3 3 0 11-6 0 3 3 0 016 0z" />
                        </svg>
                        Product From
                      </p>
                      <p className="font-semibold text-gray-900">
                        {deviceData.details.productFrom}
                      </p>
                    </div>
                  )}
                  {deviceData.details.packerShipper && (
                    <div className="bg-white bg-opacity-80 p-4 rounded-lg border border-orange-200">
                      <p className="text-sm text-gray-600 flex items-center gap-1">
                        <svg className="h-4 w-4 text-indigo-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path d="M9 17a2 2 0 11-4 0 2 2 0 014 0zM19 17a2 2 0 11-4 0 2 2 0 014 0z" />
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16V6a1 1 0 00-1-1H4a1 1 0 00-1 1v10a1 1 0 001 1h1m8-1a1 1 0 01-1 1H9m4-1V8a1 1 0 011-1h2.586a1 1 0 01.707.293l3.414 3.414a1 1 0 01.293.707V16a1 1 0 01-1 1h-1m-6-1a1 1 0 001 1h1M5 17a2 2 0 104 0m-4 0a2 2 0 114 0m6 0a2 2 0 104 0m-4 0a2 2 0 114 0" />
                        </svg>
                        Packer / Shipper
                      </p>
                      <p className="font-semibold text-gray-900">
                        {deviceData.details.packerShipper}
                      </p>
                    </div>
                  )}
                  {deviceData.details.supplierIdTracking && (
                    <div className="bg-white bg-opacity-80 p-4 rounded-lg border border-orange-200">
                      <p className="text-sm text-gray-600 flex items-center gap-1">
                        <svg className="h-4 w-4 text-purple-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 20l4-16m2 16l4-16M6 9h14M4 15h14" />
                        </svg>
                        Supplier ID / Tracking #
                      </p>
                      <p className="font-semibold text-gray-900 font-mono text-sm">
                        {deviceData.details.supplierIdTracking}
                      </p>
                    </div>
                  )}
                </div>
              </div>
            )}

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
