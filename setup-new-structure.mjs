import {initializeApp} from "firebase/app";
import {getDatabase, ref, set} from "firebase/database";

// Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyBQje281bPAt7MiJdK94ru1irAU8i3luzY",
  authDomain: "tracking-box-e17a1.firebaseapp.com",
  databaseURL:
    "https://tracking-box-e17a1-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "tracking-box-e17a1",
  storageBucket: "tracking-box-e17a1.appspot.com",
  messagingSenderId: "915169912201",
  appId: "1:915169912201:web:933b9e9043b3904d0d2d58",
  measurementId: "G-TZNB7SK8S0",
};

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

async function setupNewStructure() {
  console.log("🏗️  Setting up new tracking_box structure...");

  try {
    // Create the correct structure for your ESP32 device
    const newStructure = {
      tracking_box: {
        box_001: {
          details: {
            name: "",
            setLocation: "",
          },
          sensorData: {
            temp: 0,
            humidity: 0,
            accelerometer: "NORMAL",
            currentLocation: "No GPS Fix",
            batteryVoltage: 0,
            wakeReason: "",
            timestamp: 0,
            bootCount: 0,
            altitude: 0,
          },
        },
      },
    };

    const rootRef = ref(db, "/");
    await set(rootRef, newStructure);

    console.log("✅ New structure created successfully!");
    console.log("📊 Database now contains:");
    console.log(JSON.stringify(newStructure, null, 2));
  } catch (error) {
    console.error("❌ Error setting up structure:", error);
  } finally {
    process.exit(0);
  }
}

setupNewStructure();
