"use client";

import { useState, useEffect, useRef } from "react";
import { db } from "./firebase";
import { ref, onValue, update, set } from "firebase/database";
import { Toaster, toast } from "react-hot-toast";
import QRLinkPage from "./qr-link/page";
import EditInfoModal from "@/components/EditInfoModal";
import TrackingBoxModal from "@/components/TrackingBoxModal";

interface TrackingBoxDetails {
  name: string;
  setLocation: string; // coordinates
  setLocationLabel?: string; // human-friendly address
  description?: string;
  referenceCode?: string; // Unique 10-character reference code
}

interface SensorData {
  temp: number;
  humidity: number;
  accelerometer:
    | {
        x: number;
        y: number;
        z: number;
        tiltDetected: boolean;
      }
    | string;
  currentLocation: string;
  batteryVoltage?: number;
  wakeUpReason?: string; // standardized key
  timestamp?: number;
  bootCount?: number;
  altitude?: number;
  limitSwitchPressed?: boolean;
  locationBreach?: boolean;
  securityBreachActive?: boolean;
  buzzerIsActive?: boolean;
  buzzerDismissed?: boolean;
}

interface TrackingBox {
  details: TrackingBoxDetails;
  sensorData: SensorData;
}

interface TrackingData {
  [key: string]: TrackingBox;
}


export default function Home() {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [trackingData, setTrackingData] = useState<TrackingData>({});
  const [currentDate, setCurrentDate] = useState(new Date());
  const [isOnline, setIsOnline] = useState(true);
  const [isEditModalOpen, setIsEditModalOpen] = useState(false);
  const [selectedEditBoxId, setSelectedEditBoxId] = useState<string | null>(
    null
  );
  const [isViewModalOpen, setIsViewModalOpen] = useState(false);
  const [selectedViewBoxId, setSelectedViewBoxId] = useState<string | null>(
    null
  );
  const [isLoading, setIsLoading] = useState(true);
  const [dataError, setDataError] = useState<string | null>(null);
  const [isDismissing, setIsDismissing] = useState<string | null>(null);
  const prevTrackingDataRef = useRef<TrackingData>({});
  const [isActivatingLock, setIsActivatingLock] = useState<string | null>(null);

  // ------------------------------------------------------------------
  // Reverse-geocoding (lat,lon → human readable)
  // ------------------------------------------------------------------
  const geocodeCache = useRef<Record<string, string>>({});

  // --------------------------------------------------------------
  // Helper: convert coordinate strings (decimal OR DMS) to [lat,lon]
  // --------------------------------------------------------------
  const dmsToDecimal = (deg: number, min: number, sec: number, dir = "N") => {
    let dec = deg + min / 60 + sec / 3600;
    if (/[SW]/i.test(dir)) dec = -dec;
    return dec;
  };

  const parseCoordinates = (raw: string): [number, number] | null => {
    if (!raw) return null;

    // 1) Simple decimal "lat, lon"
    const decMatch = raw.match(/-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?/);
    if (decMatch) {
      const [latStr, lonStr] = decMatch[0].split(/\s*,\s*/);
      return [parseFloat(latStr), parseFloat(lonStr)];
    }

    // 2) Degrees-minutes-seconds   14°33'43.1"N 121°06'43.3"E
    const dmsRegex =
      /(\d{1,3})[^0-9]+(\d{1,2})[^0-9]+(\d{1,2}(?:\.\d+)?)\s*["'′″]?\s*([NSEW])/gi;
    const parts: { deg: number; min: number; sec: number; dir: string }[] = [];
    let m;
    // eslint-disable-next-line no-cond-assign
    while ((m = dmsRegex.exec(raw))) {
      parts.push({ deg: +m[1], min: +m[2], sec: +m[3], dir: m[4] });
    }
    if (parts.length >= 2) {
      return [
        dmsToDecimal(parts[0].deg, parts[0].min, parts[0].sec, parts[0].dir),
        dmsToDecimal(parts[1].deg, parts[1].min, parts[1].sec, parts[1].dir),
      ];
    }

    // 3) Fallback to first two numbers (old behaviour)
    const nums = raw.match(/-?\d+(?:\.\d+)?/g);
    if (nums && nums.length >= 2) {
      let lat = parseFloat(nums[0]);
      let lon = parseFloat(nums[1]);
      if (/S/i.test(raw)) lat = -Math.abs(lat);
      if (/W/i.test(raw)) lon = -Math.abs(lon);
      return [lat, lon];
    }
    return null;
  };

  const [prettyLocations, setPrettyLocations] = useState<
    Record<string, string>
  >({});

  // Whenever trackingData changes, resolve any new coordinate strings
  useEffect(() => {
    const fetchReverseGeocode = async (lat: number, lon: number) => {
      const key = `${lat.toFixed(4)},${lon.toFixed(4)}`;
      if (geocodeCache.current[key]) return geocodeCache.current[key];

      try {
        const url = `https://nominatim.openstreetmap.org/reverse?format=jsonv2&lat=${lat}&lon=${lon}&zoom=14&addressdetails=1`;
        const res = await fetch(url, {
          headers: { "User-Agent": "tracking-box-dashboard" },
        });
        const json = await res.json();
        // Build a concise address using returned address fields when possible
        let human = json.display_name || key;
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
        return key;
      }
    };

    (async () => {
      const updates: Record<string, string> = {};
      await Promise.all(
        Object.entries(trackingData).map(async ([boxId, box]) => {
          const rawLoc = box.details.setLocation;
          const coords = parseCoordinates(rawLoc);
          if (!coords) return;
          const [lat, lon] = coords;

          const human = await fetchReverseGeocode(lat, lon);
          updates[boxId] = human;
        })
      );
      if (Object.keys(updates).length) {
        setPrettyLocations((prev) => ({ ...prev, ...updates }));
      }
    })();
  }, [trackingData]);

  useEffect(() => {
    prevTrackingDataRef.current = trackingData;
  });


  useEffect(() => {
    const trackingBoxRef = ref(db, "tracking_box");

    const unsubscribe = onValue(
      trackingBoxRef,
      (snapshot) => {
        try {
          setIsLoading(false);
          setDataError(null);
          const rawData = snapshot.val();

          if (rawData) {
            // Validate data structure
            const validatedData: TrackingData = {};
            Object.keys(rawData).forEach((boxId) => {
              const box = rawData[boxId];
              if (box && typeof box === "object") {
                validatedData[boxId] = {
                  details: {
                    name: box.details?.name || "",
                    setLocation: box.details?.setLocation || "",
                    setLocationLabel: box.details?.setLocationLabel || "",
                    description: box.details?.description || "",
                    referenceCode: box.details?.referenceCode || "",
                  },
                  sensorData: {
                    temp: box.sensorData?.temp || 0,
                    humidity: box.sensorData?.humidity || 0,
                    accelerometer: box.sensorData?.accelerometer || "NORMAL",
                    currentLocation:
                      box.sensorData?.currentLocation || "No GPS Fix",
                    batteryVoltage: box.sensorData?.batteryVoltage || 0,
                    wakeUpReason:
                      box.sensorData?.wakeUpReason ||
                      box.sensorData?.wakeReason ||
                      "",
                    timestamp: box.sensorData?.timestamp || 0,
                    bootCount: box.sensorData?.bootCount || 0,
                    altitude: box.sensorData?.altitude || 0,
                    limitSwitchPressed:
                      box.sensorData?.limitSwitchPressed ?? true, // Default to true (secure)
                    locationBreach: box.sensorData?.locationBreach || false,
                    securityBreachActive:
                      box.sensorData?.securityBreachActive || false,
                    buzzerIsActive: box.sensorData?.buzzerIsActive || false,
                    buzzerDismissed: box.sensorData?.buzzerDismissed || false,
                  },
                };
              }
            });

            // Compare with previous data (stored in ref) for real-time toasts
            const prevData = prevTrackingDataRef.current;

            Object.keys(validatedData).forEach((boxId) => {
              const currentBox = validatedData[boxId];
              const prevBox = prevData ? prevData[boxId] : null;

              // Check for motion detection wake up
              if (currentBox && prevBox) {
                const currentWakeReason = currentBox.sensorData.wakeUpReason || "";
                const prevWakeReason = prevBox.sensorData.wakeUpReason || "";
                
                // Check if wake reason changed to motion detection
                if (currentWakeReason.toLowerCase().includes("motion") && 
                    !prevWakeReason.toLowerCase().includes("motion")) {
                  toast(
                    `🏃 Motion detected on ${currentBox.details.name || boxId}`,
                    {
                      id: `motion-toast-${boxId}-${Date.now()}`,
                      duration: 15000, // Changed from 5000ms (5 seconds) to 15000ms (15 seconds)
                      position: "top-right",
                      style: {
                        background: "#FEF3C7",
                        color: "#92400E",
                        border: "1px solid #F59E0B",
                      },
                      iconTheme: {
                        primary: "#F59E0B",
                        secondary: "#FFFFFF",
                      },
                    }
                  );
                }
              }

              const isNowCritical =
                currentBox &&
                currentBox.sensorData.securityBreachActive &&
                currentBox.sensorData.buzzerIsActive &&
                !currentBox.sensorData.buzzerDismissed;

              const wasPreviouslyCritical =
                prevBox &&
                prevBox.sensorData.securityBreachActive &&
                prevBox.sensorData.buzzerIsActive &&
                !prevBox.sensorData.buzzerDismissed;

              if (isNowCritical && !wasPreviouslyCritical) {
                toast.error(
                  (t) => {
                    if (!currentBox) return null;
                    return (
                      <div className="flex flex-col gap-2">
                        <span className="font-bold">
                          CRITICAL SECURITY BREACH!
                        </span>
                        <span>
                          {currentBox.details.name || boxId} has been moved
                          outside the safe zone.
                        </span>
                        <div className="flex gap-2 mt-2">
                          <button
                            onClick={() => {
                              dismissCriticalAlert(boxId);
                              toast.dismiss(t.id);
                            }}
                            disabled={isDismissing === boxId}
                            className={`flex-1 font-semibold py-1 px-2 rounded-md transition-colors ${
                              isDismissing === boxId
                                ? "bg-gray-400 cursor-not-allowed"
                                : "bg-red-700 hover:bg-red-800"
                            } text-white`}
                          >
                            {isDismissing === boxId
                              ? "Dismissing..."
                              : "DISMISS ALARM"}
                          </button>
                          <button
                            onClick={() => toast.dismiss(t.id)}
                            className="flex-1 bg-gray-500 text-white font-semibold py-1 px-2 rounded-md hover:bg-gray-600 transition-colors"
                          >
                            Close
                          </button>
                        </div>
                      </div>
                    );
                  },
                  {
                    id: `critical-toast-${boxId}`, // Prevent duplicate toasts for the same box
                    duration: Infinity, // Keep toast open until manually dismissed
                    position: "top-right",
                    style: {
                      border: "2px solid #B91C1C",
                      padding: "12px",
                      color: "#B91C1C",
                      backgroundColor: "#FEE2E2",
                    },
                    iconTheme: {
                      primary: "#B91C1C",
                      secondary: "#FFFFFF",
                    },
                  }
                );
              }
            });

            // Finally update React state with the new snapshot
            setTrackingData(validatedData);
          } else {
            // No data available
            setTrackingData({});
          }
        } catch (error) {
          console.error("Error processing Firebase data:", error);
          setDataError("Failed to load tracking data");
          setIsLoading(false);
        }
      },
      (error) => {
        console.error("Firebase connection error:", error);
        setDataError(
          "Connection error. Please check your internet connection."
        );
        setIsLoading(false);
      }
    );

    return () => unsubscribe();
  }, []);

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentDate(new Date());
    }, 1000);

    return () => {
      clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    if (typeof window !== "undefined") {
      if ("serviceWorker" in navigator) {
        window.addEventListener("load", () => {
          navigator.serviceWorker
            .register("/sw.js")
            .then((registration) => {
              console.log(
                "ServiceWorker registration successful with scope: ",
                registration.scope
              );
            })
            .catch((error) => {
              console.log("ServiceWorker registration failed: ", error);
            });
        });
      }
    }
  }, []);

  useEffect(() => {
    const handleOnline = () => setIsOnline(true);
    const handleOffline = () => setIsOnline(false);

    if (typeof window !== "undefined") {
      setIsOnline(navigator.onLine);
      window.addEventListener("online", handleOnline);
      window.addEventListener("offline", handleOffline);
    }

    return () => {
      if (typeof window !== "undefined") {
        window.removeEventListener("online", handleOnline);
        window.removeEventListener("offline", handleOffline);
      }
    };
  }, []);

  // Function to dismiss critical security breach alert
  const dismissCriticalAlert = async (boxId: string) => {
    setIsDismissing(boxId);

    try {
      // Update BOTH paths to support WiFi and SMS modes
      // 1. For WiFi mode: Update dismissAlert path
      const dismissAlertRef = ref(db, `tracking_box/${boxId}/dismissAlert`);
      await set(dismissAlertRef, {
        dismissed: true,
        timestamp: Date.now()
      });

      // 2. For SMS mode: Update sensorData flags
      const sensorDataRef = ref(db, `tracking_box/${boxId}/sensorData`);
      await update(sensorDataRef, {
        buzzerIsActive: false,
        buzzerDismissed: true, // Mark as dismissed for SMS mode
      });

      toast.success(`Alarm for ${boxId} dismissed successfully.`, {
        position: "top-right",
      });
      console.log(`Critical alert dismissed for ${boxId}`);
    } catch (error) {
      console.error("Error dismissing alert:", error);
      toast.error(`An error occurred while dismissing the alarm for ${boxId}.`);
    } finally {
      setIsDismissing(null);
    }
  };

  // Convert human address → "lat, lon" string using Nominatim search
  const forwardGeocode = async (query: string): Promise<string | null> => {
    try {
      const res = await fetch(
        `https://nominatim.openstreetmap.org/search?format=jsonv2&limit=1&countrycodes=ph&q=${encodeURIComponent(
          query
        )}`,
        {
          headers: {
            "User-Agent": "tracking-box-dashboard",
            "Accept-Language": "en",
          },
        }
      );
      const json = await res.json();
      if (json && json.length > 0) {
        const { lat, lon } = json[0];
        return `${parseFloat(lat).toFixed(5)}, ${parseFloat(lon).toFixed(5)}`;
      }
    } catch (e) {
      console.warn("forward geocode failed", e);
    }
    return null;
  };

  const handleLogin = (e: React.FormEvent) => {
    e.preventDefault();
    if (username === "Admin123" && password === "123123123a") {
      setIsLoggedIn(true);
      setError("");
    } else {
      setError("Invalid username or password");
    }
  };

  const handleLogout = () => {
    setIsLoggedIn(false);
    setUsername("");
    setPassword("");
  };

  const handleEditClick = (boxId: string) => {
    setSelectedEditBoxId(boxId);
    setIsEditModalOpen(true);
  };

  const handleCloseEditModal = () => {
    setIsEditModalOpen(false);
    setSelectedEditBoxId(null);
  };

  const handleViewClick = (boxId: string) => {
    setSelectedViewBoxId(boxId);
    setIsViewModalOpen(true);
  };

  const handleCloseViewModal = () => {
    setIsViewModalOpen(false);
    setSelectedViewBoxId(null);
  };

  const handleSaveInfo = async (
    boxId: string,
    coords: string,
    name: string,
    label: string,
    description: string
  ) => {
    try {
      const coordString = coords || (await forwardGeocode(label)) || label;

      const boxRef = ref(db, `tracking_box/${boxId}/details`);
      await update(boxRef, {
        setLocation: coordString,
        setLocationLabel: label,
        name: name,
        description: description,
      });

      // Update local state as well
      setTrackingData((prev) => ({
        ...prev,
        [boxId]: {
          ...prev[boxId],
          details: {
            ...prev[boxId]?.details,
            setLocation: coordString,
            setLocationLabel: label,
            name: name,
            description: description,
          },
        },
      }));

      // cache human-readable for immediate UI feedback
      setPrettyLocations((prev) => ({ ...prev, [boxId]: label }));
    } catch (error) {
      console.error("Error updating tracking box info:", error);
    }
  };

  // Trigger solenoid activation in Firebase
  const handleActivateLock = async (boxId: string) => {
    setIsActivatingLock(boxId);
    try {
      const lockRef = ref(db, `tracking_box/${boxId}/solenoid`);
      await set(lockRef, true);
      toast.success(`Lock activated for ${boxId}`, { position: "top-right" });
    } catch (error) {
      console.error("Error activating lock:", error);
      toast.error(`Failed to activate lock for ${boxId}`);
    } finally {
      setIsActivatingLock(null);
    }
  };

  if (!isOnline) {
    return <QRLinkPage />;
  }

  if (isLoggedIn) {
    return (
      <main className="flex min-h-screen flex-col items-center justify-start bg-gray-100 p-10">
        <Toaster />
        <div className="w-full max-w-6xl bg-white rounded-md shadow-md p-8">
          <div className="flex justify-between items-center w-full mb-8">
            <h1 className="text-3xl font-bold text-gray-900">
              Admin Dashboard
            </h1>
            <div className="flex items-center gap-4">
              <p className="text-md font-semibold text-gray-600">
                {currentDate.toLocaleDateString("en-US", {
                  weekday: "long",
                  year: "numeric",
                  month: "long",
                  day: "numeric",
                })}
              </p>
              <p className="text-2xl font-bold text-gray-800">
                {currentDate.toLocaleTimeString("en-US")}
              </p>
              <div className="flex items-center gap-2">
                <span
                  className={`h-3 w-3 rounded-full ${
                    isOnline ? "bg-green-500" : "bg-red-500"
                  }`}
                ></span>
                <p className="text-md font-semibold text-gray-600">
                  {isOnline ? "Online" : "Offline"}
                </p>
              </div>
            </div>
            <button
              onClick={handleLogout}
              className="bg-red-600 hover:bg-red-700 text-white cursor-pointer font-bold py-2 px-4 rounded-md transition duration-300"
            >
              Logout
            </button>
          </div>

          <div className="mt-8 overflow-x-auto">
            <table className="min-w-full divide-y divide-gray-200 border">
              <thead className="bg-gray-50">
                <tr>
                  <th
                    scope="col"
                    className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider border border-gray-300"
                  >
                    Tracking Box No.
                  </th>
                  <th
                    scope="col"
                    className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider border border-gray-300"
                  >
                    Address
                  </th>
                  <th
                    scope="col"
                    className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider border border-gray-300"
                  >
                    Owner
                  </th>
                  <th
                    scope="col"
                    className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider border border-gray-300"
                  >
                    Full Description
                  </th>
                  <th
                    scope="col"
                    className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider border border-gray-300"
                  ></th>
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {isLoading ? (
                  <tr>
                    <td
                      colSpan={5}
                      className="px-6 py-10 text-center text-sm text-gray-500"
                    >
                      <div className="flex items-center justify-center gap-2">
                        <div className="animate-spin h-5 w-5 border-2 border-blue-600 border-t-transparent rounded-full"></div>
                        Loading tracking data...
                      </div>
                    </td>
                  </tr>
                ) : dataError ? (
                  <tr>
                    <td
                      colSpan={5}
                      className="px-6 py-10 text-center text-sm text-red-600"
                    >
                      <div className="flex flex-col items-center gap-2">
                        <svg
                          className="h-8 w-8 text-red-500"
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
                        <p className="font-medium">{dataError}</p>
                        <button
                          onClick={() => window.location.reload()}
                          className="mt-2 px-4 py-2 bg-red-600 text-white rounded-md hover:bg-red-700 transition-colors"
                        >
                          Retry
                        </button>
                      </div>
                    </td>
                  </tr>
                ) : Object.keys(trackingData).length === 0 ? (
                  <tr>
                    <td
                      colSpan={5}
                      className="px-6 py-10 text-center text-sm text-gray-500"
                    >
                      <div className="flex flex-col items-center gap-2">
                        <svg
                          className="h-8 w-8 text-gray-400"
                          fill="none"
                          viewBox="0 0 24 24"
                          stroke="currentColor"
                        >
                          <path
                            strokeLinecap="round"
                            strokeLinejoin="round"
                            strokeWidth={2}
                            d="M20 13V6a2 2 0 00-2-2H6a2 2 0 00-2 2v7m16 0v5a2 2 0 01-2 2H6a2 2 0 01-2-2v-5m16 0h-2M4 13h2m13-4h.01M6 9h.01"
                          />
                        </svg>
                        <p className="font-medium">No tracking boxes found</p>
                        <p className="text-xs text-gray-400">
                          Devices will appear here once they start transmitting
                          data
                        </p>
                      </div>
                    </td>
                  </tr>
                ) : (
                  Object.keys(trackingData).map((boxId) => {
                    const item = trackingData[boxId];
                    return (
                      <tr key={boxId} className="hover:bg-gray-100">
                        <td className="px-6 py-2 whitespace-nowrap text-sm">
                          <div className="flex flex-col">
                            <span className="font-medium text-gray-900">{boxId.toUpperCase()}</span>
                            {item.details.referenceCode && (
                              <span className="text-xs text-gray-500 font-mono">[{item.details.referenceCode}]</span>
                            )}
                          </div>
                        </td>
                        <td className="px-6 py-2 text-sm text-gray-500 max-w-xs">
                          <div className="whitespace-normal break-words">
                            {prettyLocations[boxId] || "Location not set"}
                          </div>
                          <div className="text-xs text-gray-400 break-words">
                            {item.details.setLocation}
                          </div>
                        </td>
                        <td className="px-6 py-2 whitespace-nowrap text-sm text-gray-500">
                          {item.details.name || "Name not set"}
                        </td>
                        <td className="px-6 py-2 text-sm text-gray-500">
                          <button
                            onClick={() => handleViewClick(boxId)}
                            className="bg-blue-600 hover:bg-blue-700 text-white font-bold py-1 px-3 rounded-md border-2 border-blue-700 transition duration-300 text-sm"
                          >
                            View
                          </button>
                        </td>
                        <td className="px-6 py-2 text-sm text-gray-500">
                          <div className="flex gap-2">
                            <button
                              onClick={() => handleEditClick(boxId)}
                              className="bg-green-600 hover:bg-green-700 text-white font-bold py-1 px-3 rounded-md border-2 border-green-700 transition duration-300 text-sm"
                            >
                              Edit Info
                            </button>
                            <button
                              onClick={() => handleActivateLock(boxId)}
                              disabled={isActivatingLock === boxId}
                              className={`bg-red-600 hover:bg-red-700 text-white font-bold py-1 px-3 rounded-md border-2 border-red-700 transition duration-300 text-sm ${
                                isActivatingLock === boxId
                                  ? "opacity-50 cursor-not-allowed"
                                  : ""
                              }`}
                            >
                              {isActivatingLock === boxId
                                ? "Activating..."
                                : "Activate Lock"}
                            </button>
                          </div>
                        </td>
                      </tr>
                    );
                  })
                )}
              </tbody>
            </table>
          </div>

          {/* Modal for detailed tracking box view */}
          <TrackingBoxModal
            isOpen={isViewModalOpen}
            onClose={handleCloseViewModal}
            boxId={selectedViewBoxId}
            trackingData={
              selectedViewBoxId ? trackingData[selectedViewBoxId] : null
            }
          />

          {/* Modal for editing tracking box info */}
          <EditInfoModal
            isOpen={isEditModalOpen}
            onClose={handleCloseEditModal}
            boxId={selectedEditBoxId}
            currentAddress={
              selectedEditBoxId
                ? trackingData[selectedEditBoxId]?.details?.setLocationLabel ||
                  trackingData[selectedEditBoxId]?.details?.setLocation ||
                  ""
                : ""
            }
            currentOwner={
              selectedEditBoxId
                ? trackingData[selectedEditBoxId]?.details?.name || ""
                : ""
            }
            currentDescription={
              selectedEditBoxId
                ? trackingData[selectedEditBoxId]?.details?.description || ""
                : ""
            }
            referenceCode={
              selectedEditBoxId
                ? trackingData[selectedEditBoxId]?.details?.referenceCode || ""
                : ""
            }
            onSave={handleSaveInfo}
          />
        </div>
      </main>
    );
  }

  return (
    <main className="flex min-h-screen flex-col items-center justify-center bg-gray-100 p-24">
      <div className="w-full max-w-md bg-white rounded-lg shadow-md border border-gray-300 p-8">
        <h1 className="text-3xl font-bold text-center text-gray-900 mb-6">
          Tracking Box Login
        </h1>
        <form onSubmit={handleLogin}>
          {error && (
            <p className="text-red-500 text-sm text-center mb-4">{error}</p>
          )}
          <div className="mb-4">
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
              className="w-full px-3 py-2 bg-white border border-gray-300 rounded-md text-gray-900 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="Enter your username"
            />
          </div>
          <div className="mb-6">
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
              className="w-full px-3 py-2 bg-white border border-gray-300 rounded-md text-gray-900 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="Enter your password"
            />
          </div>
          <button
            type="submit"
            className="w-full bg-blue-600 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded-md transition duration-300 cursor-pointer"
          >
            Login
          </button>
        </form>
      </div>
    </main>
  );
}
