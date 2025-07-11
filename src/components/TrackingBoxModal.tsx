"use client";

import {useState, useEffect} from "react";
import {ref, update} from "firebase/database";
import {db} from "@/app/firebase";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
} from "@/components/ui/dialog";
import {ScrollArea} from "@/components/ui/scroll-area";
import {Badge} from "@/components/ui/badge";
import {
  Battery,
  Thermometer,
  Droplets,
  MapPin,
  Zap,
  Edit3,
  Save,
  X,
  FileText,
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
  setLocation: string;
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
  const [isEditingDescription, setIsEditingDescription] = useState(false);
  const [description, setDescription] = useState("");
  const [originalDescription, setOriginalDescription] = useState("");

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

  useEffect(() => {
    if (isOpen && trackingData?.details) {
      const desc = trackingData.details.description || "";
      setDescription(desc);
      setOriginalDescription(desc);
    }
  }, [trackingData, isOpen]);

  // Push new point whenever a new timestamp arrives
  useEffect(() => {
    if (!trackingData) return;
    const sd = trackingData.sensorData;
    const ts = sd.timestamp || Date.now();
    const accel =
      typeof sd.accelerometer === "object" && sd.accelerometer
        ? sd.accelerometer
        : {x: 0, y: 0, z: 0};
    const point = {
      time: new Date(ts).toLocaleTimeString(),
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

  const handleSaveDescription = async () => {
    if (!boxId) return;

    try {
      const updates = {
        [`tracking_box/${boxId}/details/description`]: description,
      };
      await update(ref(db), updates);
      setOriginalDescription(description);
      setIsEditingDescription(false);
    } catch (error) {
      console.error("Error updating description:", error);
    }
  };

  const handleCancelEdit = () => {
    setDescription(originalDescription);
    setIsEditingDescription(false);
  };

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
            {/* Editable Description Section - Positioned at the top */}
            <div className="bg-gradient-to-br from-slate-50 to-slate-100 rounded-xl p-6 border border-slate-200">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold text-slate-900 flex items-center gap-2">
                  <FileText className="h-5 w-5" />
                  Item Description
                </h3>
                {!isEditingDescription && (
                  <button
                    onClick={() => setIsEditingDescription(true)}
                    className="flex items-center gap-2 px-3 py-1 text-sm bg-blue-500 text-white rounded-md hover:bg-blue-600 transition-colors"
                  >
                    <Edit3 className="h-4 w-4" />
                    Edit
                  </button>
                )}
              </div>

              {isEditingDescription ? (
                <div className="space-y-3">
                  <textarea
                    value={description}
                    onChange={(e) => setDescription(e.target.value)}
                    placeholder="Add a description for this tracking box..."
                    className="w-full min-h-[100px] p-3 border border-slate-300 rounded-lg resize-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                    maxLength={500}
                  />
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-slate-500">
                      {description.length}/500 characters
                    </span>
                    <div className="flex gap-2">
                      <button
                        onClick={handleCancelEdit}
                        className="flex items-center gap-2 px-3 py-1 text-sm bg-slate-500 text-white rounded-md hover:bg-slate-600 transition-colors"
                      >
                        <X className="h-4 w-4" />
                        Cancel
                      </button>
                      <button
                        onClick={handleSaveDescription}
                        className="flex items-center gap-2 px-3 py-1 text-sm bg-green-500 text-white rounded-md hover:bg-green-600 transition-colors"
                      >
                        <Save className="h-4 w-4" />
                        Save
                      </button>
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-slate-700">
                  {description || (
                    <span className="text-slate-400 italic">
                      No description set. Click &apos;Edit&apos; to add one.
                    </span>
                  )}
                </div>
              )}
            </div>

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
                    {currentSensorData.wakeUpReason ?? "Unknown"}
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
                      {currentSensorData.timestamp
                        ? new Date(
                            currentSensorData.timestamp
                          ).toLocaleTimeString()
                        : "N/A"}
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
                    margin={{top: 5, right: 20, left: 0, bottom: 5}}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [, time] = str.split(" ");
                        return time.replace(":", "").slice(0, 5);
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
                      dot={{r: 3}}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Humidity Chart */}
              <div className="w-full h-52">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{top: 5, right: 20, left: 0, bottom: 5}}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [, time] = str.split(" ");
                        return time.replace(":", "").slice(0, 5);
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
                      dot={{r: 3}}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Battery Voltage Chart */}
              <div className="w-full h-52">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{top: 5, right: 20, left: 0, bottom: 5}}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [, time] = str.split(" ");
                        return time.replace(":", "").slice(0, 5);
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
                      dot={{r: 3}}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Accelerometer XYZ Chart */}
              <div className="w-full h-56">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={chartData}
                    margin={{top: 5, right: 20, left: 0, bottom: 5}}
                  >
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="time"
                      minTickGap={20}
                      tickFormatter={(str) => {
                        const [, time] = str.split(" ");
                        return time.replace(":", "").slice(0, 5);
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
                      dot={{r: 3}}
                    />
                    <Line
                      type="monotone"
                      dataKey="accelY"
                      stroke="#06b6d4"
                      name="Accel Y (g)"
                      dot={{r: 3}}
                    />
                    <Line
                      type="monotone"
                      dataKey="accelZ"
                      stroke="#a855f7"
                      name="Accel Z (g)"
                      dot={{r: 3}}
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
