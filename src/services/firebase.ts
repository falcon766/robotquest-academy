import { initializeApp } from 'firebase/app';
import { getAuth } from 'firebase/auth';
import { getFirestore } from 'firebase/firestore';

const firebaseConfig = {
    apiKey: "AIzaSyAu2DndU0nH1SJ5D_Jttf0mlhT1Cml6MKE",
    authDomain: "droid-academy.firebaseapp.com",
    projectId: "droid-academy",
    storageBucket: "droid-academy.firebasestorage.app",
    messagingSenderId: "952207848556",
    appId: "1:952207848556:web:1d83dff917c420dc6ed449",
    measurementId: "G-RNTBMCR064"
};

const app = initializeApp(firebaseConfig);
export const auth = getAuth(app);
export const db = getFirestore(app);
