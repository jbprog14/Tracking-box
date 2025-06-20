// Firebase Database Cleanup Script
// Run with: node firebase-cleanup.js

import {initializeApp} from "firebase/app";
import {getDatabase, ref, get, remove, set} from "firebase/database";

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

async function checkCurrentData() {
  console.log("🔍 Checking current Firebase data...\n");

  try {
    // Check tracking_boxes (old)
    const trackingBoxesRef = ref(db, "tracking_boxes");
    const trackingBoxesSnapshot = await get(trackingBoxesRef);

    if (trackingBoxesSnapshot.exists()) {
      console.log('📊 Found "tracking_boxes" (old structure):');
      console.log(JSON.stringify(trackingBoxesSnapshot.val(), null, 2));
    } else {
      console.log('❌ No "tracking_boxes" found');
    }

    // Check trackers (unwanted)
    const trackersRef = ref(db, "trackers");
    const trackersSnapshot = await get(trackersRef);

    if (trackersSnapshot.exists()) {
      console.log('\n📊 Found "trackers" (unwanted structure):');
      console.log(JSON.stringify(trackersSnapshot.val(), null, 2));
    } else {
      console.log('\n❌ No "trackers" found');
    }

    // Check tracking_box (new)
    const trackingBoxRef = ref(db, "tracking_box");
    const trackingBoxSnapshot = await get(trackingBoxRef);

    if (trackingBoxSnapshot.exists()) {
      console.log('\n📊 Found "tracking_box" (new structure):');
      console.log(JSON.stringify(trackingBoxSnapshot.val(), null, 2));
    } else {
      console.log('\n❌ No "tracking_box" found');
    }
  } catch (error) {
    console.error("🚨 Error checking data:", error);
  }
}

async function deleteTrackers() {
  console.log('🗑️ Deleting "trackers" table...');

  try {
    const trackersRef = ref(db, "trackers");
    await remove(trackersRef);
    console.log('✅ "trackers" table deleted successfully');
  } catch (error) {
    console.error("🚨 Error deleting trackers:", error);
  }
}

async function deleteTrackingBoxes() {
  console.log('🗑️ Deleting "tracking_boxes" table...');

  try {
    const trackingBoxesRef = ref(db, "tracking_boxes");
    await remove(trackingBoxesRef);
    console.log('✅ "tracking_boxes" table deleted successfully');
  } catch (error) {
    console.error("🚨 Error deleting tracking_boxes:", error);
  }
}

async function createSampleNewStructure() {
  console.log("📱 Creating sample data in new structure...");

  try {
    const sampleData = {
      box_001: {
        details: {
          name: "",
          setLocation: "",
        },
        sensorData: {
          temp: 25.5,
          humidity: 60,
          accelerometer: "NORMAL",
          currentLocation: "No GPS Fix",
          batteryVoltage: 3.8,
          wakeReason: "Timer (15 minutes)",
          timestamp: Date.now(),
          bootCount: 1,
          altitude: 0,
        },
      },
    };

    const trackingBoxRef = ref(db, "tracking_box");
    await set(trackingBoxRef, sampleData);
    console.log("✅ Sample data created in new structure");
    console.log("📋 Sample structure:");
    console.log(JSON.stringify(sampleData, null, 2));
  } catch (error) {
    console.error("🚨 Error creating sample data:", error);
  }
}

async function clearAll() {
  console.log("🧹 Clearing ALL tracking data...");

  try {
    await deleteTrackers();
    await deleteTrackingBoxes();

    const trackingBoxRef = ref(db, "tracking_box");
    await remove(trackingBoxRef);
    console.log("✅ All tracking data cleared");
  } catch (error) {
    console.error("🚨 Error clearing all data:", error);
  }
}

// Main execution
async function main() {
  const action = process.argv[2];

  console.log("🔥 Firebase Database Cleanup Tool");
  console.log("===================================\n");

  switch (action) {
    case "check":
      await checkCurrentData();
      break;
    case "delete-trackers":
      await deleteTrackers();
      break;
    case "delete-tracking-boxes":
      await deleteTrackingBoxes();
      break;
    case "create-sample":
      await createSampleNewStructure();
      break;
    case "clear-all":
      await clearAll();
      break;
    default:
      console.log("Usage:");
      console.log(
        "  node firebase-cleanup.js check                - Check current data"
      );
      console.log(
        '  node firebase-cleanup.js delete-trackers      - Delete "trackers" table'
      );
      console.log(
        '  node firebase-cleanup.js delete-tracking-boxes - Delete "tracking_boxes" table'
      );
      console.log(
        "  node firebase-cleanup.js create-sample        - Create sample data in new structure"
      );
      console.log(
        "  node firebase-cleanup.js clear-all            - Clear all tracking data"
      );
      console.log("");
      console.log("Recommended cleanup sequence:");
      console.log("1. node firebase-cleanup.js check");
      console.log("2. node firebase-cleanup.js delete-trackers");
      console.log("3. node firebase-cleanup.js delete-tracking-boxes");
      console.log("4. node firebase-cleanup.js create-sample");
  }

  process.exit(0);
}

main();
