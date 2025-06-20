"use client";

import {useState, useEffect} from "react";
import {db} from "./firebase";
import {ref, onValue, update} from "firebase/database";
import QRLinkPage from "./qr-link/page";
import EditInfoModal from "@/components/EditInfoModal";
import TrackingBoxModal from "@/components/TrackingBoxModal";

interface TrackingBoxDetails {
  name: string;
  setLocation: string;
}

interface SensorData {
  temp: number;
  humidity: number;
  accelerometer: string;
  currentLocation: string;
  // Additional monitoring data
  batteryVoltage?: number;
  wakeReason?: string;
  timestamp?: number;
  bootCount?: number;
  altitude?: number;
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

  useEffect(() => {
    const trackingBoxRef = ref(db, "tracking_box");

    const unsubscribe = onValue(
      trackingBoxRef,
      (snapshot) => {
        try {
          setIsLoading(false);
          setDataError(null);
          const data = snapshot.val();

          // Debug: Log raw Firebase data
          console.log("🔥 Raw Firebase data:", data);
          console.log("🔑 Data keys:", data ? Object.keys(data) : "No data");

          if (data) {
            // Validate data structure for new format
            const validatedData: TrackingData = {};
            Object.keys(data).forEach((boxId) => {
              const box = data[boxId];
              console.log(`📦 Processing box ${boxId}:`, box);

              if (box && typeof box === "object") {
                validatedData[boxId] = {
                  details: {
                    name: box.details?.name || "",
                    setLocation: box.details?.setLocation || "",
                  },
                  sensorData: {
                    temp: box.sensorData?.temp || 0,
                    humidity: box.sensorData?.humidity || 0,
                    accelerometer: box.sensorData?.accelerometer || "NORMAL",
                    currentLocation:
                      box.sensorData?.currentLocation || "No GPS Fix",
                    batteryVoltage: box.sensorData?.batteryVoltage || 0,
                    wakeReason: box.sensorData?.wakeReason || "",
                    timestamp: box.sensorData?.timestamp || 0,
                    bootCount: box.sensorData?.bootCount || 0,
                    altitude: box.sensorData?.altitude || 0,
                  },
                };
                console.log(`✅ Validated box ${boxId}:`, validatedData[boxId]);
              }
            });
            console.log("🎯 Final validated data:", validatedData);
            setTrackingData(validatedData);
          } else {
            // No data available - set empty state
            console.log("❌ No data received from Firebase");
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
    setLocation: string,
    name: string
  ) => {
    try {
      const boxRef = ref(db, `tracking_box/${boxId}/details`);
      await update(boxRef, {
        setLocation: setLocation,
        name: name,
      });

      // Update local state as well
      setTrackingData((prev) => ({
        ...prev,
        [boxId]: {
          ...prev[boxId],
          details: {
            ...prev[boxId]?.details,
            setLocation: setLocation,
            name: name,
          },
        },
      }));
    } catch (error) {
      console.error("Error updating tracking box info:", error);
    }
  };

  if (!isOnline) {
    return <QRLinkPage />;
  }

  if (isLoggedIn) {
    return (
      <main className="flex min-h-screen flex-col items-center justify-start bg-gray-100 p-10">
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
                        <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                          {boxId.toUpperCase()}
                        </td>
                        <td className="px-6 py-4 text-sm text-gray-500">
                          {item.details.setLocation || "Location not set"}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {item.details.name || "Name not set"}
                        </td>
                        <td className="px-6 py-4 text-sm text-gray-500">
                          <button
                            onClick={() => handleViewClick(boxId)}
                            className="bg-blue-600 hover:bg-blue-700 text-white font-bold py-1 px-3 rounded-md border-2 border-blue-700 transition duration-300 text-sm"
                          >
                            View
                          </button>
                        </td>
                        <td className="px-6 py-4 text-sm text-gray-500">
                          <button
                            onClick={() => handleEditClick(boxId)}
                            className="bg-green-600 hover:bg-green-700 text-white font-bold py-1 px-3 rounded-md border-2 border-green-700 transition duration-300 text-sm"
                          >
                            Edit Info
                          </button>
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
                ? trackingData[selectedEditBoxId]?.details?.setLocation || ""
                : ""
            }
            currentOwner={
              selectedEditBoxId
                ? trackingData[selectedEditBoxId]?.details?.name || ""
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
