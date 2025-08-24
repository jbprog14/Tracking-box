import { NextRequest, NextResponse } from "next/server";
import { db } from "@/app/firebase";
import { ref, set, get, serverTimestamp } from "firebase/database";

interface SensorData {
  temp: number;
  humidity: number;
  currentLocation: string;
  altitude: number;
  tilt: boolean;
  fall: boolean;
  limitSwitch: boolean;
  solenoid: boolean;
  accelerometer: {
    x: number;
    y: number;
    z: number;
    tiltDetected: boolean;
  };
  batteryVoltage: number;
  wakeUpReason: string;
  timestamp: number;
  bootCount: number;
  referenceCode: string;
  securityBreachActive: boolean;
}

export async function POST(
  request: NextRequest,
  { params }: { params: Promise<{ deviceId: string }> }
) {
  try {
    const { deviceId } = await params;
    
    // Parse the JSON body
    const body: SensorData = await request.json();
    
    // Validate required fields
    if (!deviceId) {
      return NextResponse.json(
        { success: false, error: "Device ID is required" },
        { status: 400 }
      );
    }
    
    // Validate sensor data structure
    if (
      body.temp === undefined ||
      body.humidity === undefined ||
      !body.currentLocation ||
      body.batteryVoltage === undefined
    ) {
      return NextResponse.json(
        { success: false, error: "Missing required sensor data fields" },
        { status: 400 }
      );
    }
    
    // Prepare data for Firebase with server timestamp
    const firebaseData = {
      ...body,
      serverTimestamp: serverTimestamp(),
      lastUpdated: new Date().toISOString()
    };
    
    // Write sensor data to Firebase
    const sensorDataRef = ref(db, `tracking_box/${deviceId}/sensorData`);
    await set(sensorDataRef, firebaseData);
    
    // Read control flags from Firebase
    const controlFlagsRef = ref(db, `tracking_box/${deviceId}/controlFlags`);
    const controlFlagsSnapshot = await get(controlFlagsRef);
    
    let controlFlags = {
      buzzer: false,
      solenoid: false
    };
    
    if (controlFlagsSnapshot.exists()) {
      const flags = controlFlagsSnapshot.val();
      controlFlags = {
        buzzer: flags.buzzer || false,
        solenoid: flags.solenoid || false
      };
    }
    
    // Return success with control flags
    return NextResponse.json({
      success: true,
      message: "Sensor data updated successfully",
      controlFlags: controlFlags,
      timestamp: new Date().toISOString()
    });
    
  } catch (error) {
    console.error("Error processing sensor data:", error);
    
    // Handle JSON parse errors
    if (error instanceof SyntaxError) {
      return NextResponse.json(
        { success: false, error: "Invalid JSON format" },
        { status: 400 }
      );
    }
    
    // Handle other errors
    return NextResponse.json(
      { 
        success: false, 
        error: "Failed to process sensor data",
        details: error instanceof Error ? error.message : "Unknown error"
      },
      { status: 500 }
    );
  }
}

// Optional: GET endpoint to retrieve latest sensor data
export async function GET(
  request: NextRequest,
  { params }: { params: Promise<{ deviceId: string }> }
) {
  try {
    const { deviceId } = await params;
    
    if (!deviceId) {
      return NextResponse.json(
        { success: false, error: "Device ID is required" },
        { status: 400 }
      );
    }
    
    // Read sensor data from Firebase
    const sensorDataRef = ref(db, `tracking_box/${deviceId}/sensorData`);
    const snapshot = await get(sensorDataRef);
    
    if (!snapshot.exists()) {
      return NextResponse.json(
        { success: false, error: "Device not found" },
        { status: 404 }
      );
    }
    
    const sensorData = snapshot.val();
    
    // Also get control flags
    const controlFlagsRef = ref(db, `tracking_box/${deviceId}/controlFlags`);
    const controlFlagsSnapshot = await get(controlFlagsRef);
    
    let controlFlags = {
      buzzer: false,
      solenoid: false
    };
    
    if (controlFlagsSnapshot.exists()) {
      const flags = controlFlagsSnapshot.val();
      controlFlags = {
        buzzer: flags.buzzer || false,
        solenoid: flags.solenoid || false
      };
    }
    
    return NextResponse.json({
      success: true,
      sensorData: sensorData,
      controlFlags: controlFlags
    });
    
  } catch (error) {
    console.error("Error retrieving sensor data:", error);
    return NextResponse.json(
      { 
        success: false, 
        error: "Failed to retrieve sensor data",
        details: error instanceof Error ? error.message : "Unknown error"
      },
      { status: 500 }
    );
  }
}