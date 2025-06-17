"use client";

import {useState, useEffect} from "react";
import {
  AreaChart,
  Area,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";
import {TooltipProps} from "recharts";
import {
  NameType,
  ValueType,
} from "recharts/types/component/DefaultTooltipContent";
import {db} from "./firebase";
import {ref, onValue, set} from "firebase/database";
import QRLinkPage from "./qr-link/page";

interface Reading {
  name: string;
  temp: number;
  humidity: number;
  distance: number;
  shock: number;
  battery: number;
}

interface TrackingBox {
  address: string;
  owner: string;
  description: string;
  readings: Reading[];
}

interface TrackingData {
  [key: string]: TrackingBox;
}

const CustomTooltip = ({
  active,
  payload,
  label,
}: TooltipProps<ValueType, NameType>) => {
  if (active && payload && payload.length) {
    return (
      <div className="p-3 bg-white rounded-lg shadow-lg border border-gray-200 opacity-95">
        <p className="text-sm font-bold text-gray-900">{`Time: ${label}`}</p>
        <p style={{color: payload[0].color}} className="text-sm">
          {`${payload[0].name}: ${payload[0].value}`}
        </p>
      </div>
    );
  }

  return null;
};

export default function Home() {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [selectedBoxId, setSelectedBoxId] = useState<string | null>(null);
  const [trackingData, setTrackingData] = useState<TrackingData>({});
  const [chartData, setChartData] = useState<Reading[]>([]);
  const [currentDate, setCurrentDate] = useState(new Date());
  const [isOnline, setIsOnline] = useState(true);

  useEffect(() => {
    const trackingBoxesRef = ref(db, "tracking_boxes");
    onValue(trackingBoxesRef, (snapshot) => {
      const data = snapshot.val();
      if (data) {
        setTrackingData(data);
      } else {
        const mockTrackingData: TrackingData = {
          "box-001": {
            address: "123 Mockingbird Lane, Faketown, FS 12345",
            owner: "John Doe",
            description: "Shipment of fragile electronics.",
            readings: [
              {
                name: "10:00",
                temp: 22,
                humidity: 45,
                distance: 0,
                shock: 0,
                battery: 98,
              },
              {
                name: "10:05",
                temp: 22.1,
                humidity: 45,
                distance: 5,
                shock: 0,
                battery: 98,
              },
              {
                name: "10:10",
                temp: 22.2,
                humidity: 46,
                distance: 10,
                shock: 1,
                battery: 97,
              },
              {
                name: "10:15",
                temp: 22.5,
                humidity: 46,
                distance: 15,
                shock: 0,
                battery: 97,
              },
              {
                name: "10:20",
                temp: 23,
                humidity: 47,
                distance: 20,
                shock: 0,
                battery: 96,
              },
              {
                name: "10:25",
                temp: 22.8,
                humidity: 48,
                distance: 25,
                shock: 0,
                battery: 96,
              },
              {
                name: "10:30",
                temp: 22.7,
                humidity: 48,
                distance: 30,
                shock: 2,
                battery: 95,
              },
            ],
          },
        };
        set(trackingBoxesRef, mockTrackingData);
      }
    });
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

  useEffect(() => {
    if (selectedBoxId && trackingData[selectedBoxId]) {
      setChartData(trackingData[selectedBoxId].readings || []);
    }
  }, [selectedBoxId, trackingData]);

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

  const handleViewClick = (boxId: string) => {
    setSelectedBoxId(boxId);
  };

  const handleBack = () => {
    setSelectedBoxId(null);
  };

  if (!isOnline) {
    return <QRLinkPage />;
  }

  if (isLoggedIn) {
    if (selectedBoxId) {
      return (
        <main className="flex min-h-screen flex-col items-center justify-start bg-gray-100 p-10">
          <div className="w-full max-w-6xl bg-white rounded-md shadow-md p-8">
            <div className="flex justify-between items-center w-full mb-8">
              <h1 className="text-3xl font-bold text-gray-600">
                Tracking Box Details:
                <span className="ml-2 text-gray-900">
                  {selectedBoxId.toUpperCase()}
                </span>
              </h1>
              <button
                onClick={handleBack}
                className="bg-gray-600 hover:bg-gray-700 text-white cursor-pointer font-bold py-2 px-4 rounded-md transition duration-300"
              >
                Back to Dashboard
              </button>
            </div>

            <div className="grid grid-cols-1 gap-8">
              <div className="bg-gray-50 p-4 rounded-lg shadow-inner">
                <h3 className="text-lg font-semibold mb-4 text-center text-gray-800">
                  Battery Level (%)
                </h3>
                <ResponsiveContainer width="100%" height={300}>
                  <AreaChart data={chartData}>
                    <defs>
                      <linearGradient
                        id="colorBattery"
                        x1="0"
                        y1="0"
                        x2="0"
                        y2="1"
                      >
                        <stop
                          offset="5%"
                          stopColor="#00C49F"
                          stopOpacity={0.8}
                        />
                        <stop
                          offset="95%"
                          stopColor="#00C49F"
                          stopOpacity={0}
                        />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="name" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip
                      content={<CustomTooltip />}
                      cursor={{
                        stroke: "#00C49F",
                        strokeWidth: 1,
                        strokeDasharray: "3 3",
                      }}
                    />
                    <Legend wrapperStyle={{paddingTop: "20px"}} />
                    <Area
                      type="monotone"
                      dataKey="battery"
                      stroke="#00C49F"
                      strokeWidth={2}
                      fillOpacity={1}
                      fill="url(#colorBattery)"
                      name="Battery"
                      activeDot={{r: 6, stroke: "#fff", strokeWidth: 2}}
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg shadow-inner">
                <h3 className="text-lg font-semibold mb-4 text-center text-gray-800">
                  Temperature (°C)
                </h3>
                <ResponsiveContainer width="100%" height={400}>
                  <AreaChart data={chartData}>
                    <defs>
                      <linearGradient
                        id="colorTemp"
                        x1="0"
                        y1="0"
                        x2="0"
                        y2="1"
                      >
                        <stop
                          offset="5%"
                          stopColor="#8884d8"
                          stopOpacity={0.8}
                        />
                        <stop
                          offset="95%"
                          stopColor="#8884d8"
                          stopOpacity={0}
                        />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="name" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip
                      content={<CustomTooltip />}
                      cursor={{
                        stroke: "#8884d8",
                        strokeWidth: 1,
                        strokeDasharray: "3 3",
                      }}
                    />
                    <Legend wrapperStyle={{paddingTop: "20px"}} />
                    <Area
                      type="monotone"
                      dataKey="temp"
                      stroke="#8884d8"
                      strokeWidth={2}
                      fillOpacity={1}
                      fill="url(#colorTemp)"
                      name="Temperature"
                      activeDot={{r: 6, stroke: "#fff", strokeWidth: 2}}
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg shadow-inner">
                <h3 className="text-lg font-semibold mb-4 text-center text-gray-800">
                  Humidity (%)
                </h3>
                <ResponsiveContainer width="100%" height={300}>
                  <AreaChart data={chartData}>
                    <defs>
                      <linearGradient
                        id="colorHumidity"
                        x1="0"
                        y1="0"
                        x2="0"
                        y2="1"
                      >
                        <stop
                          offset="5%"
                          stopColor="#82ca9d"
                          stopOpacity={0.8}
                        />
                        <stop
                          offset="95%"
                          stopColor="#82ca9d"
                          stopOpacity={0}
                        />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="name" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip
                      content={<CustomTooltip />}
                      cursor={{
                        stroke: "#82ca9d",
                        strokeWidth: 1,
                        strokeDasharray: "3 3",
                      }}
                    />
                    <Legend wrapperStyle={{paddingTop: "20px"}} />
                    <Area
                      type="monotone"
                      dataKey="humidity"
                      stroke="#82ca9d"
                      strokeWidth={2}
                      fillOpacity={1}
                      fill="url(#colorHumidity)"
                      name="Humidity"
                      activeDot={{r: 6, stroke: "#fff", strokeWidth: 2}}
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg shadow-inner">
                <h3 className="text-lg font-semibold mb-4 text-center text-gray-800">
                  Distance (km)
                </h3>
                <ResponsiveContainer width="100%" height={300}>
                  <AreaChart data={chartData}>
                    <defs>
                      <linearGradient
                        id="colorDistance"
                        x1="0"
                        y1="0"
                        x2="0"
                        y2="1"
                      >
                        <stop
                          offset="5%"
                          stopColor="#ffc658"
                          stopOpacity={0.8}
                        />
                        <stop
                          offset="95%"
                          stopColor="#ffc658"
                          stopOpacity={0}
                        />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="name" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip
                      content={<CustomTooltip />}
                      cursor={{
                        stroke: "#ffc658",
                        strokeWidth: 1,
                        strokeDasharray: "3 3",
                      }}
                    />
                    <Legend wrapperStyle={{paddingTop: "20px"}} />
                    <Area
                      type="monotone"
                      dataKey="distance"
                      stroke="#ffc658"
                      strokeWidth={2}
                      fillOpacity={1}
                      fill="url(#colorDistance)"
                      name="Distance"
                      activeDot={{r: 6, stroke: "#fff", strokeWidth: 2}}
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg shadow-inner">
                <h3 className="text-lg font-semibold mb-4 text-center text-gray-800">
                  Shock (G)
                </h3>
                <ResponsiveContainer width="100%" height={300}>
                  <AreaChart data={chartData}>
                    <defs>
                      <linearGradient
                        id="colorShock"
                        x1="0"
                        y1="0"
                        x2="0"
                        y2="1"
                      >
                        <stop
                          offset="5%"
                          stopColor="#ff7300"
                          stopOpacity={0.8}
                        />
                        <stop
                          offset="95%"
                          stopColor="#ff7300"
                          stopOpacity={0}
                        />
                      </linearGradient>
                    </defs>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="name" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip
                      content={<CustomTooltip />}
                      cursor={{
                        stroke: "#ff7300",
                        strokeWidth: 1,
                        strokeDasharray: "3 3",
                      }}
                    />
                    <Legend wrapperStyle={{paddingTop: "20px"}} />
                    <Area
                      type="monotone"
                      dataKey="shock"
                      stroke="#ff7300"
                      strokeWidth={2}
                      fillOpacity={1}
                      fill="url(#colorShock)"
                      name="Shock"
                      activeDot={{r: 6, stroke: "#fff", strokeWidth: 2}}
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
            </div>
          </div>
        </main>
      );
    }
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
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {Object.keys(trackingData).length === 0 ? (
                  <tr>
                    <td
                      colSpan={4}
                      className="px-6 py-10 text-center text-xs text-gray-400"
                    >
                      No Box to Trace
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
                          {item.address}
                        </td>
                        <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                          {item.owner}
                        </td>
                        <td className="px-6 py-4 text-sm text-gray-500">
                          <button
                            onClick={() => handleViewClick(boxId)}
                            className="bg-blue-600 hover:bg-blue-700 text-white font-bold py-1 px-3 rounded-md transition duration-300 text-sm"
                          >
                            View
                          </button>
                        </td>
                      </tr>
                    );
                  })
                )}
              </tbody>
            </table>
          </div>
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
