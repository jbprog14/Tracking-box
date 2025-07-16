"use client";

import { useParams } from "next/navigation";
import { useState, useEffect } from "react";
// NOTE: We load Firebase dynamically *only* in the browser to keep the
// Cloudflare Pages / Edge bundle free of Node-specific polyfills.

interface DeviceDetails {
  name: string;
  setLocation: string;
  description?: string;
}

interface SensorData {
  temp: number;
  humidity: number;
  accelerometer: string;
  currentLocation: string;
  batteryVoltage?: number;
  wakeReason?: string;
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

// Cloudflare Pages ➜ Next-on-Pages requires each non-static route to declare
// that it runs in the Edge runtime.
export const runtime = "edge";

export default function QRDevicePage() {
  const params = useParams();
  const deviceId = params.deviceId as string;

  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [deviceData, setDeviceData] = useState<DeviceData | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [dataError, setDataError] = useState<string | null>(null);

  useEffect(() => {
    let unsubscribe: (() => void) | undefined;

    const initRealtime = async () => {
      // Dynamically import Firebase only in the browser
      const { getDatabase, ref, onValue } = await import("firebase/database");
      const { initializeApp } = await import("firebase/app");

      // Minimal client-side init (avoid shared singleton to keep things
      // tree-shakable in the edge build)
      const firebaseConfig = {
        apiKey: "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY",
        authDomain: "tracking-box-e17a1.firebaseapp.com",
        databaseURL:
          "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app",
        projectId: "tracking-box-e17a1",
      } as const;

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
                  description: data.details?.description || "",
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
                  wakeReason: data.sensorData?.wakeReason || "",
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

    if (isAuthenticated && deviceId && typeof window !== "undefined") {
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
  }, [isAuthenticated, deviceId]);

  const handleLogin = (e: React.FormEvent) => {
    e.preventDefault();
    if (username === "Admin123" && password === "123123123a") {
      setIsAuthenticated(true);
      setError("");
    } else {
      setError("Invalid username or password");
    }
  };

  const getStatusColor = (status: string | boolean) => {
    if (typeof status === "boolean") {
      return status ? "text-red-600" : "text-green-600";
    }
    return status === "NORMAL" ? "text-green-600" : "text-red-600";
  };

  if (!isAuthenticated) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 flex items-center justify-center p-4">
        <div className="bg-white rounded-xl shadow-2xl p-8 w-full max-w-md">
          <div className="text-center mb-8">
            <div className="mx-auto w-16 h-16 bg-blue-600 rounded-full flex items-center justify-center mb-4">
              <svg
                className="w-8 h-8 text-white"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M12 15v2m-6 4h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2zm10-10V7a4 4 0 00-8 0v4h8z"
                />
              </svg>
            </div>
            <h1 className="text-2xl font-bold text-gray-900 mb-2">
              Device Access
            </h1>
            <p className="text-gray-600">
              Viewing details for device:{" "}
              <span className="font-mono font-semibold">
                {deviceId?.toUpperCase()}
              </span>
            </p>
            <p className="text-sm text-gray-500 mt-2">
              Please authenticate to access device information
            </p>
          </div>

          <form onSubmit={handleLogin} className="space-y-6">
            <div>
              <label
                htmlFor="username"
                className="block text-sm font-medium text-gray-700 mb-2"
              >
                Username
              </label>
              <input
                type="text"
                id="username"
                value={username}
                onChange={(e) => setUsername(e.target.value)}
                className="w-full px-4 py-3 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent transition duration-200"
                placeholder="Enter username"
                required
              />
            </div>

            <div>
              <label
                htmlFor="password"
                className="block text-sm font-medium text-gray-700 mb-2"
              >
                Password
              </label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className="w-full px-4 py-3 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent transition duration-200"
                placeholder="Enter password"
                required
              />
            </div>

            {error && (
              <div className="bg-red-50 border border-red-200 rounded-lg p-3">
                <p className="text-red-700 text-sm">{error}</p>
              </div>
            )}

            <button
              type="submit"
              className="w-full bg-blue-600 hover:bg-blue-700 text-white font-bold py-3 px-4 rounded-lg transition duration-300 transform hover:scale-105"
            >
              Access Device Data
            </button>
          </form>

          <div className="mt-6 text-center">
            <p className="text-xs text-gray-500">
              This page provides offline access to device information when
              scanned from the device QR code.
            </p>
          </div>
        </div>
      </div>
    );
  }

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
                <p className="text-xs text-gray-400">
                  {new Date().toLocaleString()}
                </p>
              </div>
              <button
                onClick={() => setIsAuthenticated(false)}
                className="bg-red-600 hover:bg-red-700 text-white px-4 py-2 rounded-lg transition-colors"
              >
                Logout
              </button>
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
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Device Name</p>
                  <p className="font-semibold text-gray-900">
                    {deviceData.details.name || "Not Set"}
                  </p>
                </div>
                <div className="bg-gray-50 p-4 rounded-lg">
                  <p className="text-sm text-gray-600">Set Location</p>
                  <p className="font-semibold text-gray-900">
                    {deviceData.details.setLocation || "Not Set"}
                  </p>
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
                    {deviceData.sensorData.currentLocation}
                  </p>
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
