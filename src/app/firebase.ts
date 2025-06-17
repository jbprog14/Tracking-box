// Import the functions you need from the SDKs you need
import {initializeApp} from "firebase/app";
import {getDatabase} from "firebase/database";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
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

export {db};
