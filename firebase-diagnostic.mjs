import {initializeApp} from "firebase/app";
import {getDatabase, ref, get, remove, onValue, off} from "firebase/database";

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

console.log("🔍 FIREBASE DIAGNOSTIC TOOL");
console.log("============================\n");

async function comprehensiveDiagnostic() {
  try {
    // 1. Check Firebase project details
    console.log("1. 🏗️  Firebase Project Details:");
    console.log(`   Project ID: ${firebaseConfig.projectId}`);
    console.log(`   Database URL: ${firebaseConfig.databaseURL}`);
    console.log(`   Auth Domain: ${firebaseConfig.authDomain}\n`);

    // 2. Get complete database structure
    console.log("2. 📊 Complete Database Structure:");
    const rootRef = ref(db, "/");
    const rootSnapshot = await get(rootRef);
    const allData = rootSnapshot.val();

    if (allData) {
      console.log("   Top-level keys:", Object.keys(allData));
      console.log("   Full structure:");
      console.log(JSON.stringify(allData, null, 2));
    } else {
      console.log("   Database is empty");
    }

    // 3. Check specific paths in detail
    console.log("\n3. 🎯 Specific Path Analysis:");

    const pathsToCheck = [
      "tracking_boxes",
      "tracking_box",
      "trackers",
      "devices",
      "Example",
      "Push",
    ];

    for (const path of pathsToCheck) {
      console.log(`\n   📂 Checking path: /${path}`);
      const pathRef = ref(db, path);
      const pathSnapshot = await get(pathRef);

      if (pathSnapshot.exists()) {
        const data = pathSnapshot.val();
        console.log(
          `   ✅ EXISTS - Size: ${JSON.stringify(data).length} chars`
        );
        console.log(`   📋 Keys: ${Object.keys(data)}`);

        // Show first level structure
        if (typeof data === "object" && data !== null) {
          Object.keys(data).forEach((key) => {
            const value = data[key];
            if (typeof value === "object" && value !== null) {
              console.log(`      ${key}: {${Object.keys(value).join(", ")}}`);
            } else {
              console.log(`      ${key}: ${typeof value} (${value})`);
            }
          });
        }
      } else {
        console.log(`   ❌ DOES NOT EXIST`);
      }
    }

    // 4. Monitor for real-time changes
    console.log("\n4. 👁️  Real-time Monitoring (10 seconds):");
    console.log("   Watching for any database changes...\n");

    const monitorRef = ref(db, "/");
    let changeCount = 0;

    const unsubscribe = onValue(monitorRef, (snapshot) => {
      changeCount++;
      const timestamp = new Date().toISOString();
      console.log(`   📡 Change #${changeCount} detected at ${timestamp}`);

      const data = snapshot.val();
      if (data) {
        const keys = Object.keys(data);
        console.log(`   🔑 Current top-level keys: [${keys.join(", ")}]`);

        // Check if tracking_boxes exists
        if (data.tracking_boxes) {
          console.log(
            `   🚨 tracking_boxes detected! Keys: [${Object.keys(
              data.tracking_boxes
            ).join(", ")}]`
          );
        }
      }
    });

    // Wait 10 seconds
    await new Promise((resolve) => setTimeout(resolve, 10000));
    off(monitorRef, "value", unsubscribe);

    console.log("\n5. 🧪 Deletion Test:");
    console.log(
      "   Attempting to delete tracking_boxes and monitor for recreation...\n"
    );

    // Delete tracking_boxes and immediately check if it comes back
    for (let i = 0; i < 3; i++) {
      console.log(`   🗑️  Deletion attempt #${i + 1}:`);

      const trackingBoxesRef = ref(db, "tracking_boxes");
      const beforeSnapshot = await get(trackingBoxesRef);

      console.log(
        `      Before: ${beforeSnapshot.exists() ? "EXISTS" : "DOES NOT EXIST"}`
      );

      if (beforeSnapshot.exists()) {
        await remove(trackingBoxesRef);
        console.log(`      Deleted: tracking_boxes removed`);

        // Wait 2 seconds and check again
        await new Promise((resolve) => setTimeout(resolve, 2000));

        const afterSnapshot = await get(trackingBoxesRef);
        console.log(
          `      After 2s: ${
            afterSnapshot.exists() ? "EXISTS (RECREATED!)" : "STILL DELETED"
          }`
        );

        if (afterSnapshot.exists()) {
          console.log(
            `      🚨 RECREATION DETECTED! Data:`,
            JSON.stringify(afterSnapshot.val(), null, 2)
          );
        }
      } else {
        console.log(`      Nothing to delete`);
      }

      console.log("");
    }

    // 6. Check for any Firebase Functions or Rules
    console.log("6. ⚙️  Additional Analysis:");
    console.log(
      `   Database Region: ${
        firebaseConfig.databaseURL.includes("asia-southeast1")
          ? "Asia Southeast 1"
          : "Default"
      }`
    );
    console.log(`   Using Firebase v9+ SDK: Yes`);
    console.log(`   Connection Type: Realtime Database`);
  } catch (error) {
    console.error("❌ Diagnostic Error:", error);
  } finally {
    console.log("\n🏁 Diagnostic Complete");
    process.exit(0);
  }
}

comprehensiveDiagnostic();
