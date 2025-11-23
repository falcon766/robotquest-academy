import { doc, getDoc, setDoc, updateDoc, increment, arrayUnion } from 'firebase/firestore';
import { db } from './firebase';
import type { UserProfile } from '../types';

export const userService = {
    async getUserProfile(uid: string): Promise<UserProfile | null> {
        const userRef = doc(db, 'users', uid);
        const userSnap = await getDoc(userRef);

        if (userSnap.exists()) {
            return userSnap.data() as UserProfile;
        } else {
            // Fallback: Create profile if it doesn't exist (self-healing)
            // We need email/displayName, but we might not have them here easily without passing them.
            // For now, return null and let the caller handle it, OR try to create a basic one.
            // Better: Let's just return null, but in Dashboard we'll handle null by retrying or showing a "Setup" state.
            return null;
        }
    },

    async createUserProfile(uid: string, email: string, displayName: string = ''): Promise<void> {
        const userRef = doc(db, 'users', uid);
        const userSnap = await getDoc(userRef);

        if (!userSnap.exists()) {
            const newProfile: UserProfile = {
                uid,
                email,
                displayName: displayName || email.split('@')[0],
                xp: 0,
                level: 1,
                completedLessons: [],
                currentModuleId: 'module_1',
                streak: 1,
                lastLogin: Date.now(),
            };
            await setDoc(userRef, newProfile);
        }
    },

    async updateUserProgress(uid: string, lessonId: string, xpReward: number): Promise<void> {
        const userRef = doc(db, 'users', uid);

        await updateDoc(userRef, {
            xp: increment(xpReward),
            completedLessons: arrayUnion(lessonId),
            // Simple level up logic: Level = 1 + floor(XP / 100)
            // We can't easily do calculated fields in a simple update, 
            // so we might rely on the frontend or a cloud function for level.
            // For now, let's just update XP.
        });
    }
};
