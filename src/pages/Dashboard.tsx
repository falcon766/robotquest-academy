import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../contexts/AuthContext';
import { userService } from '../services/userService';
import { useLessonStore } from '../store/useLessonStore';
import { curriculum, type Module } from '../data/curriculum';
import type { UserProfile } from '../types';

export const Dashboard = () => {
    const { currentUser } = useAuth();
    const navigate = useNavigate();
    const { setCurrentLesson } = useLessonStore();
    const [profile, setProfile] = useState<UserProfile | null>(null);
    const [error, setError] = useState<string | null>(null);

    const handleModuleClick = (module: Module) => {
        if (!profile) return;

        // User Request: "take you to the last completed lesson. Or if all lessons are complete, the very first lesson."

        // 1. Find all completed lessons in this module
        const completedInModule = module.lessons.filter(l => profile.completedLessons.includes(l.id));

        let targetLesson;

        if (completedInModule.length > 0 && completedInModule.length < module.lessons.length) {
            // If we have some completed, but not all, go to the last completed one (as requested)
            // Note: Usually we'd go to the *next* one, but following specific instructions here.
            targetLesson = completedInModule[completedInModule.length - 1];
        } else {
            // If none are completed, OR all are completed, go to the very first lesson
            targetLesson = module.lessons[0];
        }

        setCurrentLesson(targetLesson);
        navigate('/learn');
    };

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

            <div className="space-y-12">
                {curriculum.map((course) => (
                    <div key={course.id}>
                        <h3 className="text-xl font-bold text-slate-300 mb-6 flex items-center gap-2">
                            <span className="w-2 h-8 bg-orange-500 rounded-full"></span>
                            {course.title}
                        </h3>
                        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                            {course.modules.map((module) => {
                                const totalLessons = module.lessons.length;
                                const completedCount = module.lessons.filter(l => profile.completedLessons.includes(l.id)).length;
                                const progress = Math.round((completedCount / totalLessons) * 100);
                                const isLocked = false; // For now, unlock everything for beta
                                const isCompleted = progress === 100;
                                const isActive = progress > 0 && progress < 100;

                                return (
                                    <div
                                        key={module.id}
                                        onClick={() => handleModuleClick(module)}
                                        className={`p-6 border rounded-lg transition-all group cursor-pointer relative overflow-hidden ${isLocked
                                            ? 'bg-slate-900/30 border-slate-800 opacity-60'
                                            : 'bg-slate-900/50 border-slate-800 hover:border-orange-500/50 hover:shadow-lg hover:shadow-orange-900/10'
                                            }`}
                                    >
                                        <div className="flex justify-between items-start mb-4 relative z-10">
                                            <h3 className={`text-lg font-semibold transition-colors ${isLocked ? 'text-slate-500' : 'text-white group-hover:text-orange-400'
                                                }`}>
                                                {module.title}
                                            </h3>
                                            {isCompleted ? (
                                                <span className="px-2 py-1 bg-green-500/10 text-green-400 text-xs rounded border border-green-500/20">Completed</span>
                                            ) : isActive ? (
                                                <span className="px-2 py-1 bg-orange-500/10 text-orange-400 text-xs rounded border border-orange-500/20">Active</span>
                                            ) : (
                                                <span className="px-2 py-1 bg-slate-800 text-slate-500 text-xs rounded border border-slate-700">Start</span>
                                            )}
                                        </div>

                                        <p className="text-slate-400 text-sm mb-6 leading-relaxed relative z-10">
                                            {module.lessons.length} Lessons • {module.lessons.reduce((acc, l) => acc + l.xpReward, 0)} XP
                                        </p>

                                        <div className="w-full bg-slate-800 h-1.5 rounded-full overflow-hidden relative z-10">
                                            <div
                                                className={`h-full transition-all duration-500 ${isCompleted ? 'bg-green-500' : 'bg-orange-500'}`}
                                                style={{ width: `${progress}%` }}
                                            ></div>
                                        </div>

                                        {/* Subtle background glow for active/hover */}
                                        {!isLocked && (
                                            <div className="absolute inset-0 bg-gradient-to-br from-orange-500/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity"></div>
                                        )}
                                    </div>
                                );
                            })}
                        </div>
                    </div>
                ))}
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
