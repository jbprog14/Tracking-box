import {initializeApp} from "firebase/app";
import {getDatabase, ref, remove, set, get} from "firebase/database";

// Firebase configuration (using the correct config from your app)
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

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

async function resetDatabase() {
  console.log("🔥 Starting Firebase Database Reset...\n");

  try {
    // Step 1: Check what's currently in the database
    console.log("📋 Step 1: Checking current database content...");
    const rootRef = ref(db, "/");
    const snapshot = await get(rootRef);
    const currentData = snapshot.val();

    if (currentData) {
      console.log("📊 Current database structure:");
      console.log(JSON.stringify(currentData, null, 2));
      console.log("\n🗂️  Top-level keys:", Object.keys(currentData));
    } else {
      console.log("✅ Database is already empty");
    }

    // Step 2: Delete all old data structures
    console.log("\n🗑️  Step 2: Removing old data structures...");

    const pathsToDelete = [
      "tracking_boxes", // Old structure
      "trackers", // Another old structure
      "devices", // Any other potential old structure
      "tracking-data", // Possible old path
      "sensor-data", // Possible old path
    ];

    for (const path of pathsToDelete) {
      try {
        const pathRef = ref(db, path);
        const pathSnapshot = await get(pathRef);

        if (pathSnapshot.exists()) {
          console.log(`🚮 Deleting: ${path}`);
          await remove(pathRef);
          console.log(`✅ Successfully deleted: ${path}`);
        } else {
          console.log(`ℹ️  Path doesn't exist: ${path}`);
        }
      } catch (error) {
        console.log(`❌ Error deleting ${path}:`, error.message);
      }
    }

    // Step 3: Completely clear the entire database (nuclear option)
    console.log("\n💥 Step 3: Performing complete database reset...");
    await remove(rootRef);
    console.log("✅ Database completely cleared");

    // Step 4: Set up the correct new structure
    console.log("\n🏗️  Step 4: Setting up new database structure...");

    const newStructure = {
      tracking_box: {
        box_001: {
          details: {
            name: "Sample Tracking Box",
            setLocation: "Test Location",
          },
          sensorData: {
            temp: 22.5,
            humidity: 45,
            accelerometer: "NORMAL",
            currentLocation: "No GPS Fix",
            batteryVoltage: 3.8,
            wakeReason: "TIMER_WAKE",
            timestamp: Date.now(),
            bootCount: 1,
            altitude: 0,
          },
        },
      },
    };

    await set(rootRef, newStructure);
    console.log("✅ New database structure created");

    // Step 5: Verify the new structure
    console.log("\n🔍 Step 5: Verifying new database structure...");
    const verifySnapshot = await get(rootRef);
    const newData = verifySnapshot.val();

    if (newData) {
      console.log("📊 New database structure:");
      console.log(JSON.stringify(newData, null, 2));
    }

    console.log("\n🎉 Database reset completed successfully!");
    console.log("\n📝 Summary:");
    console.log('   ✅ Old "tracking_boxes" data removed');
    console.log("   ✅ All legacy data structures cleared");
    console.log('   ✅ New "tracking_box" structure created');
    console.log("   ✅ Sample data added for testing");
  } catch (error) {
    console.error("❌ Error during database reset:", error);
  } finally {
    process.exit(0);
  }
}

// Alternative function to just clear without adding sample data
async function clearDatabaseOnly() {
  console.log("🔥 Starting Firebase Database Clear (no sample data)...\n");

  try {
    console.log("💥 Clearing entire database...");
    const rootRef = ref(db, "/");
    await remove(rootRef);
    console.log("✅ Database completely cleared");
    console.log("\n🎉 Database clear completed successfully!");
  } catch (error) {
    console.error("❌ Error during database clear:", error);
  } finally {
    process.exit(0);
  }
}

// Check command line arguments
const args = process.argv.slice(2);
if (args.includes("--clear-only")) {
  clearDatabaseOnly();
} else {
  resetDatabase();
}
