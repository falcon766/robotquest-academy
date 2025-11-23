import { useEffect, useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { userService } from '../services/userService';
import type { UserProfile } from '../types';

export const Dashboard = () => {
    const { currentUser } = useAuth();
    const [profile, setProfile] = useState<UserProfile | null>(null);
    const [error, setError] = useState<string | null>(null);

    const fetchProfile = async () => {
        if (!currentUser) return;
        try {
            setError(null);
            let userProfile = await userService.getUserProfile(currentUser.uid);
            if (!userProfile) {
                await userService.createUserProfile(currentUser.uid, currentUser.email || '', currentUser.displayName || '');
                userProfile = await userService.getUserProfile(currentUser.uid);
            }
            if (userProfile) {
                setProfile(userProfile);
            } else {
                setError("Could not create profile. Please try again.");
            }
        } catch (err) {
            console.error(err);
            setError("Failed to load profile.");
        }
    };

    useEffect(() => {
        fetchProfile();
    }, [currentUser]);

    if (!profile) {
        return (
            <div className="p-8 text-center">
                <div className="text-slate-400 mb-4">{error || "Initializing user profile..."}</div>
                {error && (
                    <button
                        onClick={fetchProfile}
                        className="px-4 py-2 bg-slate-800 hover:bg-slate-700 text-white rounded transition-colors"
                    >
                        Retry
                    </button>
                )}
            </div>
        );
    }

    return (
        <div className="space-y-8">
            <div className="flex justify-between items-end border-b border-slate-800 pb-6">
                <div>
                    <h2 className="text-3xl font-bold text-white tracking-tight">Dashboard</h2>
                    <p className="text-slate-400 mt-1">Welcome back, {profile.displayName}</p>
                </div>
                <div className="text-right">
                    <div className="text-3xl font-bold text-white">{profile.xp} <span className="text-sm text-slate-500 font-normal">XP</span></div>
                    <div className="text-sm text-slate-500 uppercase tracking-wider">Level {profile.level}</div>
                </div>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <div className="p-6 bg-slate-900/50 border border-slate-800 rounded-lg hover:border-slate-600 transition-all group cursor-pointer">
                    <div className="flex justify-between items-start mb-4">
                        <h3 className="text-lg font-semibold text-white group-hover:text-orange-400 transition-colors">Module 1: The Shell</h3>
                        <span className="px-2 py-1 bg-orange-500/10 text-orange-400 text-xs rounded border border-orange-500/20">Active</span>
                    </div>
                    <p className="text-slate-400 text-sm mb-6 leading-relaxed">Master the Linux command line and configure your ROS 2 environment.</p>
                    <div className="w-full bg-slate-800 h-1.5 rounded-full overflow-hidden">
                        <div className="bg-orange-500 h-full" style={{ width: `${profile.completedLessons.includes('lesson_1') ? 100 : 0}%` }}></div>
                    </div>
                </div>

                <div className="p-6 bg-slate-900/30 border border-slate-800 rounded-lg opacity-60">
                    <div className="flex justify-between items-start mb-4">
                        <h3 className="text-lg font-semibold text-slate-500">Module 2: Nodes</h3>
                        <span className="px-2 py-1 bg-slate-800 text-slate-500 text-xs rounded border border-slate-700">Locked</span>
                    </div>
                    <p className="text-slate-600 text-sm mb-6 leading-relaxed">Understand the fundamental building blocks of ROS 2 graphs.</p>
                </div>
            </div>

            {/* Account Settings Section */}
            <div className="mt-12 pt-8 border-t border-slate-800">
                <h3 className="text-xl font-bold text-white mb-6">Account Settings</h3>
                <div className="bg-slate-900/50 border border-slate-800 rounded-lg p-6 max-w-2xl">
                    <div className="grid gap-6">
                        <div>
                            <label className="block text-sm font-medium text-slate-400 mb-2">Display Name</label>
                            <div className="text-white font-medium">{profile.displayName}</div>
                        </div>
                        <div>
                            <label className="block text-sm font-medium text-slate-400 mb-2">Email</label>
                            <div className="text-white font-medium">{profile.email}</div>
                        </div>
                        <div>
                            <label className="block text-sm font-medium text-slate-400 mb-2">User ID</label>
                            <div className="text-slate-500 text-xs font-mono">{profile.uid}</div>
                        </div>
                        <div className="pt-4 border-t border-slate-800">
                            <button className="px-4 py-2 bg-red-500/10 text-red-400 border border-red-500/20 rounded hover:bg-red-500/20 transition-colors text-sm font-medium">
                                Reset Progress
                            </button>
                            <p className="text-xs text-slate-500 mt-2">This will reset all your XP and lesson progress. This action cannot be undone.</p>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
};
