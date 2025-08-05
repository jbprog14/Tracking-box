"use client";

import { useState, useEffect } from "react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
} from "@/components/ui/dialog";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Badge } from "@/components/ui/badge";
import {
  Battery,
  Thermometer,
  Droplets,
  MapPin,
  Zap,
  Shield,
  AlertTriangle,
} from "lucide-react";
import TrackingBoxMap from "./TrackingBoxMap";
import {
  ResponsiveContainer,
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  Legend,
} from "recharts";

interface TrackingBoxDetails {
  name: string;
  setLocation: string; // coords
  setLocationLabel?: string; // human readable
  description?: string;
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
    | string; // Support both new object format and old string format for backwards compatibility
  currentLocation: string;
  batteryVoltage?: number;
  wakeUpReason?: string;
  timestamp?: number;
  bootCount?: number;
  altitude?: number;
  limitSwitchPressed?: boolean;
  locationBreach?: boolean;
}

interface TrackingBoxData {
  details: TrackingBoxDetails;
  sensorData: SensorData;
}

interface TrackingBoxModalProps {
  isOpen: boolean;
  onClose: () => void;
  boxId: string | null;
  trackingData: TrackingBoxData | null;
}

export default function TrackingBoxModal({
  isOpen,
  onClose,
  boxId,
  trackingData,
}: TrackingBoxModalProps) {
  // Realtime chart data state (keep last 30 points)
  const [chartData, setChartData] = useState<
    {
      time: string;
      temp: number;
      humidity: number;
      battery: number;
      accelX: number;
      accelY: number;
      accelZ: number;
    }[]
  >([]);

  // Push new point whenever a new timestamp arrives
  useEffect(() => {
    if (!trackingData) return;
    const sd = trackingData.sensorData;
    const ts = sd.timestamp || Date.now();
    const accel =
      typeof sd.accelerometer === "object" && sd.accelerometer
        ? sd.accelerometer
        : { x: 0, y: 0, z: 0 };
    const timeLabel = new Date(ts).toLocaleTimeString([], {
      hour: "2-digit",
      minute: "2-digit",
      hour12: false,
    });
    const point = {
      time: timeLabel,
      temp: sd.temp,
      humidity: sd.humidity,
      battery: sd.batteryVoltage || 0,
      accelX: accel.x ?? 0,
      accelY: accel.y ?? 0,
      accelZ: accel.z ?? 0,
    };
    setChartData((prev) => {
      const updated = [...prev, point].slice(-30); // keep last 30 readings
      return updated;
    });
  }, [trackingData?.sensorData.timestamp]);

  if (!boxId || !trackingData) return null;

  const currentSensorData = trackingData.sensorData;
  const deviceDetails = trackingData.details;

  const getStatus = () => {
    if (
      !currentSensorData.limitSwitchPressed &&
      currentSensorData.locationBreach
    ) {
      return {
        text: "CRITICAL BREACH",
        color: "bg-red-600",
        icon: <AlertTriangle className="h-4 w-4 mr-2" />,
      };
    }
    // Safely check for tilt detection
    const tiltDetected =
      typeof currentSensorData.accelerometer === "object" &&
      currentSensorData.accelerometer !== null &&
      currentSensorData.accelerometer.tiltDetected;

    if (tiltDetected) {
      return {
        text: "MOTION DETECTED",
        color: "bg-yellow-500",
        icon: <Zap className="h-4 w-4 mr-2" />,
      };
    }
    return {
      text: "OPERATIONAL",
      color: "bg-green-500",
      icon: <Shield className="h-4 w-4 mr-2" />,
    };
  };

  const status = getStatus();

  // Determine display label for last wake-up event
  const lastWakeupDisplay =
    currentSensorData.wakeUpReason ||
    (currentSensorData.bootCount === 0 ? "FIRST BOOT" : "TIMER SCHEDULE");

  // Normalize timestamp (handle seconds vs milliseconds) and build display string
  const lastUpdateTimestamp =
    currentSensorData.timestamp && currentSensorData.timestamp < 1e12
      ? currentSensorData.timestamp * 1000 // convert seconds → ms
      : currentSensorData.timestamp;

  const lastUpdateDisplay = lastUpdateTimestamp
    ? new Date(lastUpdateTimestamp).toLocaleTimeString()
    : "N/A";

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="max-w-7xl h-[90vh] p-0 flex flex-col">
        <ScrollArea className="flex-1 overflow-auto">
          <DialogHeader className="p-6 pb-0">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-4">
                <div className="p-3 bg-gradient-to-br from-blue-500 to-blue-600 rounded-xl">
                  <MapPin className="h-6 w-6 text-white" />
                </div>
                <div>
                  <DialogTitle className="text-2xl font-bold text-gray-900 flex items-center gap-3">
                    Tracking Box {boxId.toUpperCase()}
                    <Badge className={`${status.color} text-white border-none`}>
                      {status.icon}
                      {status.text}
                    </Badge>
                  </DialogTitle>
                  <DialogDescription className="text-base text-gray-600 mt-1">
                    Real-time monitoring and analytics dashboard
                  </DialogDescription>
                </div>
              </div>
            </div>
          </DialogHeader>

          <div className="p-6 space-y-6">
            {/* Info Cards */}
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
              {/* Temperature */}
              <div className="bg-white rounded-xl shadow p-4 flex items-center gap-4 border">
                <div className="p-3 bg-red-100 rounded-full">
                  <Thermometer className="h-6 w-6 text-red-600" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Temperature</p>
                  <p className="text-2xl font-bold text-gray-900">
                    {currentSensorData.temp.toFixed(1)}°C
                  </p>
                </div>
              </div>
              {/* Humidity */}
              <div className="bg-white rounded-xl shadow p-4 flex items-center gap-4 border">
                <div className="p-3 bg-blue-100 rounded-full">
                  <Droplets className="h-6 w-6 text-blue-600" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Humidity</p>
                  <p className="text-2xl font-bold text-gray-900">
                    {currentSensorData.humidity.toFixed(0)}%
                  </p>
                </div>
              </div>
              {/* Battery */}
              <div className="bg-white rounded-xl shadow p-4 flex items-center gap-4 border">
                <div className="p-3 bg-green-100 rounded-full">
                  <Battery className="h-6 w-6 text-green-600" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Battery Voltage</p>
                  <p className="text-2xl font-bold text-gray-900">
                    {currentSensorData.batteryVoltage?.toFixed(2) || "N/A"} V
                  </p>
                </div>
              </div>
              {/* Last Wake Reason */}
              <div className="bg-white rounded-xl shadow p-4 flex items-center gap-4 border">
                <div className="p-3 bg-purple-100 rounded-full">
                  <Zap className="h-6 w-6 text-purple-600" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Last Wakeup</p>
                  <p className="text-xl font-bold text-gray-900">
                    {lastWakeupDisplay}
                  </p>
                </div>
              </div>
            </div>

            {/* Map and Details */}
            <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
              {/* Map */}
              <div className="lg:col-span-2 bg-white rounded-xl shadow-lg border border-gray-200 p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                  <MapPin className="h-5 w-5 text-blue-600" />
                  Location Details
                </h3>
                <div className="flex items-center gap-4 mb-3 text-sm">
                  <div className="flex items-center gap-2">
                    <div className="w-4 h-4 bg-blue-500 rounded-full"></div>
                    <span className="text-gray-600">Drop-off Location</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <div className="w-4 h-4 bg-red-500 rounded-full"></div>
                    <span className="text-gray-600">
                      Current Package Location
                    </span>
                  </div>
                </div>
                <div className="h-[400px] rounded-lg overflow-hidden">
                  <TrackingBoxMap
                    setLocation={deviceDetails.setLocation}
                    currentLocation={currentSensorData.currentLocation}
                    boxId={boxId}
                    boxName={deviceDetails.name}
                  />
                </div>
              </div>

              {/* Device Details */}
              <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6 space-y-4">
                <h3 className="text-lg font-semibold text-gray-900">
                  Device Details
                </h3>
                <div className="space-y-3">
                  <div className="flex justify-between items-center">
                    <span className="text-gray-600">Owner Name</span>
                    <span className="font-semibold text-gray-900">
                      {deviceDetails.name || "Not Set"}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-600">Device ID</span>
                    <Badge variant="outline">{boxId}</Badge>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-600">Boot Count</span>
                    <span className="font-semibold text-gray-900">
                      {currentSensorData.bootCount || 0}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-600">Last Update</span>
                    <span className="font-semibold text-gray-900">
                      {lastUpdateDisplay}
                    </span>
                  </div>
                </div>
              </div>
            </div>

            {/* Realtime Sensor Charts */}
            <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6 space-y-6">
              <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                <Zap className="h-5 w-5 text-orange-600" /> Realtime Sensor
                Trends
              </h3>
              {/* Temperature Chart */}
              <div className="w-full h-52">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{ top: 5, right: 20, left: 0, bottom: 5 }}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [time] = str.split(" ");
                        return time.slice(0, 5);
                      }}
                    />
                    <YAxis domain={[0, "auto"]} />
                    <Tooltip />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="temp"
                      stroke="#ef4444"
                      name="Temp (°C)"
                      dot={{ r: 3 }}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Humidity Chart */}
              <div className="w-full h-52">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{ top: 5, right: 20, left: 0, bottom: 5 }}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [time] = str.split(" ");
                        return time.slice(0, 5);
                      }}
                    />
                    <YAxis domain={[0, "auto"]} />
                    <Tooltip />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="humidity"
                      stroke="#3b82f6"
                      name="Humidity (%)"
                      dot={{ r: 3 }}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Battery Voltage Chart */}
              <div className="w-full h-52">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{ top: 5, right: 20, left: 0, bottom: 5 }}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [time] = str.split(" ");
                        return time.slice(0, 5);
                      }}
                    />
                    <YAxis domain={[0, "auto"]} />
                    <Tooltip />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="battery"
                      stroke="#10b981"
                      name="Battery (V)"
                      dot={{ r: 3 }}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Accelerometer XYZ Chart */}
              <div className="w-full h-56">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{ top: 5, right: 20, left: 0, bottom: 5 }}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [time] = str.split(" ");
                        return time.slice(0, 5);
                      }}
                    />
                    <YAxis domain={[-2, 2]} />
                    <Tooltip />
                    <Legend />
                    <Line
                      type="monotone"
                      dataKey="accelX"
                      stroke="#f97316"
                      name="Accel X (g)"
                      dot={{ r: 3 }}
                    />
                    <Line
                      type="monotone"
                      dataKey="accelY"
                      stroke="#06b6d4"
                      name="Accel Y (g)"
                      dot={{ r: 3 }}
                    />
                    <Line
                      type="monotone"
                      dataKey="accelZ"
                      stroke="#a855f7"
                      name="Accel Z (g)"
                      dot={{ r: 3 }}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <p className="text-xs text-gray-500">
                Graph shows latest changes.
              </p>
            </div>
          </div>

          <div className="p-6 pt-0">
            <div className="flex justify-end">
              <button
                onClick={onClose}
                className="min-w-24 px-4 py-2 border border-gray-300 text-gray-700 bg-white rounded-md hover:bg-gray-50 transition-colors"
              >
                Close
              </button>
            </div>
          </div>
        </ScrollArea>
      </DialogContent>
    </Dialog>
  );
}
