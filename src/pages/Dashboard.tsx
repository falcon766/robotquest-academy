import { useEffect, useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { userService } from '../services/userService';
import type { UserProfile } from '../types';

export const Dashboard = () => {
    const { currentUser } = useAuth();
    const [profile, setProfile] = useState<UserProfile | null>(null);

    useEffect(() => {
        if (currentUser) {
            const fetchProfile = async () => {
                let userProfile = await userService.getUserProfile(currentUser.uid);
                if (!userProfile) {
                    // Profile missing? Create it now.
                    await userService.createUserProfile(currentUser.uid, currentUser.email || '', currentUser.displayName || '');
                    userProfile = await userService.getUserProfile(currentUser.uid);
                }
                setProfile(userProfile);
            };
            fetchProfile();
        }
    }, [currentUser]);

    if (!profile) return <div className="p-6 text-slate-400">Loading profile...</div>;

    return (
        <div className="space-y-6">
            <div className="flex justify-between items-end">
                <h2 className="text-3xl font-bold text-white">Welcome, {profile.displayName}</h2>
                <div className="text-right">
                    <div className="text-2xl font-bold text-emerald-400">{profile.xp} XP</div>
                    <div className="text-sm text-slate-400">Level {profile.level}</div>
                </div>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <div className="p-6 bg-slate-900 border border-slate-800 rounded-xl hover:border-emerald-500/50 transition-colors group cursor-pointer">
                    <h3 className="text-xl font-bold text-emerald-400 mb-2 group-hover:text-emerald-300">Module 1: The Shell</h3>
                    <p className="text-slate-400 mb-4">Learn the basics of the Linux shell and ROS 2 setup.</p>
                    <div className="w-full bg-slate-800 h-2 rounded-full overflow-hidden">
                        {/* Mock progress for now, could calculate based on completedLessons */}
                        <div className="bg-emerald-500 h-full" style={{ width: '20%' }}></div>
                    </div>
                </div>

                <div className="p-6 bg-slate-900 border border-slate-800 rounded-xl opacity-50">
                    <h3 className="text-xl font-bold text-slate-500 mb-2">Module 2: Nodes</h3>
                    <p className="text-slate-500 mb-4">Locked. Complete Module 1 to unlock.</p>
                </div>
            </div>
        </div>
    );
};
