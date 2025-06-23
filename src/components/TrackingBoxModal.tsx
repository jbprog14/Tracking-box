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
  Calendar,
  User,
  MapPinIcon,
  TrendingUp,
  Edit3,
  Save,
  X,
  FileText,
  Shield,
  Power,
  AlertTriangle,
} from "lucide-react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
} from "recharts";
import TrackingBoxMap from "./TrackingBoxMap";

interface TrackingBoxDetails {
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

  useEffect(() => {
    if (isOpen && trackingData?.details) {
      const desc = trackingData.details.description || "";
      setDescription(desc);
      setOriginalDescription(desc);
    }
  }, [trackingData, isOpen]);

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

  // Since we no longer have historical readings, we'll show current sensor data
  const currentSensorData = trackingData.sensorData;
  const deviceDetails = trackingData.details;

  // Create mock data for charts with current values
  const chartData = Array.from({length: 24}, (_, i) => ({
    time: `${i}:00`,
    battery: currentSensorData.batteryVoltage
      ? Number(
          (
            currentSensorData.batteryVoltage +
            (Math.random() - 0.5) * 0.2
          ).toFixed(1)
        )
      : 3.7,
    temperature: currentSensorData.temp + (Math.random() - 0.5) * 2,
    humidity: currentSensorData.humidity + (Math.random() - 0.5) * 5,
    distance: Math.random() * 100,
    shock: Math.random() > 0.8 ? 1 : 0,
  }));

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="max-w-7xl h-[90vh] p-0 flex flex-col">
        <ScrollArea className="flex-1 overflow-auto">
          <DialogHeader className="p-6 pb-0">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-4">
                <div className="p-3 bg-gradient-to-br from-blue-500 to-blue-600 rounded-xl">
                  <MapPinIcon className="h-6 w-6 text-white" />
                </div>
                <div>
                  <DialogTitle className="text-2xl font-bold text-gray-900 flex items-center gap-3">
                    Tracking Box {boxId.toUpperCase()}
                    <Badge
                      variant="secondary"
                      className="bg-green-100 text-green-800 border-green-200"
                    >
                      <div className="w-2 h-2 bg-green-500 rounded-full mr-2"></div>
                      Active
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
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              <div className="bg-gradient-to-br from-blue-50 to-blue-100 rounded-xl p-4 border border-blue-200">
                <div className="flex items-center gap-3">
                  <div className="p-2 bg-blue-500 rounded-lg">
                    <User className="h-4 w-4 text-white" />
                  </div>
                  <div>
                    <p className="text-sm font-medium text-blue-700">Name</p>
                    <p className="text-lg font-semibold text-blue-900">
                      {deviceDetails.name || "Not Set"}
                    </p>
                  </div>
                </div>
              </div>

              <div className="bg-gradient-to-br from-purple-50 to-purple-100 rounded-xl p-4 border border-purple-200">
                <div className="flex items-center gap-3">
                  <div className="p-2 bg-purple-500 rounded-lg">
                    <MapPin className="h-4 w-4 text-white" />
                  </div>
                  <div>
                    <p className="text-sm font-medium text-purple-700">
                      Set Location
                    </p>
                    <p className="text-sm font-semibold text-purple-900 truncate">
                      {deviceDetails.setLocation || "Not Set"}
                    </p>
                  </div>
                </div>
              </div>

              <div className="bg-gradient-to-br from-green-50 to-green-100 rounded-xl p-4 border border-green-200">
                <div className="flex items-center gap-3">
                  <div className="p-2 bg-green-500 rounded-lg">
                    <TrendingUp className="h-4 w-4 text-white" />
                  </div>
                  <div>
                    <p className="text-sm font-medium text-green-700">
                      Boot Count
                    </p>
                    <p className="text-lg font-semibold text-green-900">
                      {currentSensorData.bootCount || 0} Times
                    </p>
                  </div>
                </div>
              </div>
            </div>

            {/* Security Status Section */}
            <div className="bg-gradient-to-r from-red-50 to-orange-50 rounded-xl p-6 border border-red-200">
              <h3 className="text-lg font-semibold text-red-900 mb-4 flex items-center gap-2">
                <Shield className="h-5 w-5 text-red-600" />
                Security Status
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div
                  className={`p-4 rounded-lg border-2 ${
                    currentSensorData.limitSwitchPressed
                      ? "bg-yellow-50 border-yellow-300"
                      : "bg-green-50 border-green-300"
                  }`}
                >
                  <div className="flex items-center gap-3">
                    <div
                      className={`p-2 rounded-full ${
                        currentSensorData.limitSwitchPressed
                          ? "bg-yellow-500"
                          : "bg-green-500"
                      }`}
                    >
                      <Power className="h-4 w-4 text-white" />
                    </div>
                    <div>
                      <p className="text-sm font-medium text-gray-700">
                        Limit Switch
                      </p>
                      <p
                        className={`text-lg font-bold ${
                          currentSensorData.limitSwitchPressed
                            ? "text-yellow-700"
                            : "text-green-700"
                        }`}
                      >
                        {currentSensorData.limitSwitchPressed
                          ? "PRESSED"
                          : "NORMAL"}
                      </p>
                    </div>
                  </div>
                </div>

                <div
                  className={`p-4 rounded-lg border-2 ${
                    currentSensorData.locationBreach
                      ? "bg-red-50 border-red-300"
                      : "bg-green-50 border-green-300"
                  }`}
                >
                  <div className="flex items-center gap-3">
                    <div
                      className={`p-2 rounded-full ${
                        currentSensorData.locationBreach
                          ? "bg-red-500"
                          : "bg-green-500"
                      }`}
                    >
                      <AlertTriangle className="h-4 w-4 text-white" />
                    </div>
                    <div>
                      <p className="text-sm font-medium text-gray-700">
                        Location Security
                      </p>
                      <p
                        className={`text-lg font-bold ${
                          currentSensorData.locationBreach
                            ? "text-red-700"
                            : "text-green-700"
                        }`}
                      >
                        {currentSensorData.locationBreach
                          ? "BREACH DETECTED"
                          : "SECURE"}
                      </p>
                    </div>
                  </div>
                </div>
              </div>

              {(currentSensorData.limitSwitchPressed ||
                currentSensorData.locationBreach) && (
                <div className="mt-4 p-4 bg-red-100 border border-red-300 rounded-lg">
                  <div className="flex items-center gap-2">
                    <AlertTriangle className="h-5 w-5 text-red-600" />
                    <p className="text-red-800 font-semibold">
                      Security Alert Active
                    </p>
                  </div>
                  <p className="text-red-700 text-sm mt-1">
                    {currentSensorData.limitSwitchPressed &&
                    currentSensorData.locationBreach
                      ? "Both limit switch activation and location breach detected!"
                      : currentSensorData.limitSwitchPressed
                      ? "Limit switch has been activated - device may have been accessed."
                      : "Location breach detected - device has moved beyond safe zone."}
                  </p>
                </div>
              )}
            </div>

            {/* Current Sensor Data Summary */}
            <div className="bg-gradient-to-r from-gray-900 to-gray-800 rounded-xl p-6 text-white">
              <h4 className="text-lg font-semibold mb-4 flex items-center gap-2">
                <Calendar className="h-5 w-5" />
                Current Sensor Data
              </h4>
              <div className="grid grid-cols-2 md:grid-cols-5 gap-4">
                <div className="text-center">
                  <div className="flex items-center justify-center mb-2">
                    <Thermometer className="h-6 w-6 text-red-400" />
                  </div>
                  <p className="text-2xl font-bold">
                    {currentSensorData.temp}°C
                  </p>
                  <p className="text-sm text-gray-300">Temperature</p>
                </div>
                <div className="text-center">
                  <div className="flex items-center justify-center mb-2">
                    <Droplets className="h-6 w-6 text-blue-400" />
                  </div>
                  <p className="text-2xl font-bold">
                    {currentSensorData.humidity}%
                  </p>
                  <p className="text-sm text-gray-300">Humidity</p>
                </div>
                <div className="text-center">
                  <div className="flex items-center justify-center mb-2">
                    <Battery className="h-6 w-6 text-green-400" />
                  </div>
                  <p className="text-2xl font-bold">
                    {currentSensorData.batteryVoltage?.toFixed(1)}V
                  </p>
                  <p className="text-sm text-gray-300">Battery</p>
                </div>
                <div className="text-center">
                  <div className="flex items-center justify-center mb-2">
                    <MapPin className="h-6 w-6 text-yellow-400" />
                  </div>
                  <p className="text-2xl font-bold truncate">
                    {currentSensorData.currentLocation}
                  </p>
                  <p className="text-sm text-gray-300">Location</p>
                </div>
                <div className="text-center">
                  <div className="flex items-center justify-center mb-2">
                    <Zap className="h-6 w-6 text-orange-400" />
                  </div>
                  <p className="text-2xl font-bold">
                    {currentSensorData.accelerometer}
                  </p>
                  <p className="text-sm text-gray-300">Accelerometer</p>
                </div>
              </div>
            </div>

            {/* Location Map Section */}
            <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
              <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                <MapPin className="h-5 w-5 text-blue-600" />
                Location Tracking
              </h3>
              <div className="mb-4 text-sm text-gray-600">
                <p className="mb-2">
                  <span className="inline-block w-3 h-3 bg-blue-500 rounded-full mr-2"></span>
                  <strong>Blue Pin:</strong> Set Location (manually configured)
                </p>
                <p>
                  <span className="inline-block w-3 h-3 bg-red-500 rounded-full mr-2"></span>
                  <strong>Red Pin:</strong> Current GPS Location (live from
                  device)
                </p>
              </div>
              <TrackingBoxMap
                setLocation={deviceDetails.setLocation}
                currentLocation={currentSensorData.currentLocation}
                boxId={boxId}
                boxName={deviceDetails.name}
              />
            </div>

            {/* Charts Section */}
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* Battery Chart */}
              <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                  <Battery className="h-5 w-5 text-blue-600" />
                  Battery Voltage
                </h3>
                <ResponsiveContainer width="100%" height={180}>
                  <LineChart data={chartData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="time" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip />
                    <Line
                      type="monotone"
                      dataKey="battery"
                      stroke="#2563eb"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Temperature Chart */}
              <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                  <Thermometer className="h-5 w-5 text-red-600" />
                  Temperature
                </h3>
                <ResponsiveContainer width="100%" height={180}>
                  <LineChart data={chartData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="time" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip />
                    <Line
                      type="monotone"
                      dataKey="temperature"
                      stroke="#dc2626"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Humidity Chart */}
              <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                  <Droplets className="h-5 w-5 text-blue-600" />
                  Humidity
                </h3>
                <ResponsiveContainer width="100%" height={180}>
                  <LineChart data={chartData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="time" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip />
                    <Line
                      type="monotone"
                      dataKey="humidity"
                      stroke="#0891b2"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              {/* Distance Chart */}
              <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
                <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                  <MapPin className="h-5 w-5 text-green-600" />
                  Distance
                </h3>
                <ResponsiveContainer width="100%" height={180}>
                  <LineChart data={chartData}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                    <XAxis dataKey="time" stroke="#666" />
                    <YAxis stroke="#666" />
                    <Tooltip />
                    <Line
                      type="monotone"
                      dataKey="distance"
                      stroke="#16a34a"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </div>

            {/* Shock Detection Chart */}
            <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
              <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center gap-2">
                <Zap className="h-5 w-5 text-orange-600" />
                Shock Detection
              </h3>
              <ResponsiveContainer width="100%" height={200}>
                <LineChart data={chartData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
                  <XAxis dataKey="time" stroke="#666" />
                  <YAxis stroke="#666" />
                  <Tooltip />
                  <Line
                    type="stepAfter"
                    dataKey="shock"
                    stroke="#ea580c"
                    strokeWidth={2}
                    dot={false}
                  />
                </LineChart>
              </ResponsiveContainer>
            </div>

            {/* Device Information Grid */}
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* System Information */}
              <div className="bg-gradient-to-br from-white to-gray-50 rounded-xl shadow-lg border border-gray-200 p-6">
                <div className="bg-gradient-to-r from-blue-500 to-blue-600 px-6 py-4 rounded-t-xl -m-6 mb-6">
                  <h3 className="text-lg font-semibold text-white flex items-center gap-2">
                    <Battery className="h-5 w-5" />
                    System Information
                  </h3>
                </div>
                <div className="space-y-4">
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Battery Voltage:
                    </span>
                    <span className="text-lg font-semibold text-gray-900">
                      {currentSensorData.batteryVoltage?.toFixed(2)}V
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Wake Reason:
                    </span>
                    <span className="text-sm font-semibold text-gray-900 text-right">
                      {currentSensorData.wakeReason || "Unknown"}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Boot Count:
                    </span>
                    <span className="text-lg font-semibold text-gray-900">
                      {currentSensorData.bootCount || 0}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Last Update:
                    </span>
                    <span className="text-sm font-semibold text-gray-900">
                      {currentSensorData.timestamp
                        ? new Date(currentSensorData.timestamp).toLocaleString()
                        : "N/A"}
                    </span>
                  </div>
                </div>
              </div>

              {/* Environmental Data */}
              <div className="bg-gradient-to-br from-white to-gray-50 rounded-xl shadow-lg border border-gray-200 p-6">
                <div className="bg-gradient-to-r from-green-500 to-green-600 px-6 py-4 rounded-t-xl -m-6 mb-6">
                  <h3 className="text-lg font-semibold text-white flex items-center gap-2">
                    <Thermometer className="h-5 w-5" />
                    Environmental Data
                  </h3>
                </div>
                <div className="space-y-4">
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Temperature:
                    </span>
                    <span className="text-lg font-semibold text-gray-900">
                      {currentSensorData.temp}°C
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Humidity:
                    </span>
                    <span className="text-lg font-semibold text-gray-900">
                      {currentSensorData.humidity}%
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Accelerometer:
                    </span>
                    <span
                      className={`text-sm font-semibold px-2 py-1 rounded-md ${
                        currentSensorData.accelerometer === "TILT_DETECTED"
                          ? "bg-red-100 text-red-800"
                          : "bg-green-100 text-green-800"
                      }`}
                    >
                      {currentSensorData.accelerometer}
                    </span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-sm font-medium text-gray-600">
                      Altitude:
                    </span>
                    <span className="text-lg font-semibold text-gray-900">
                      {currentSensorData.altitude || 0}m
                    </span>
                  </div>
                </div>
              </div>
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
