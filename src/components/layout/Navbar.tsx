import { useEffect, useState } from 'react';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from '../../contexts/AuthContext';
import { userService } from '../../services/userService';
import type { UserProfile } from '../../types';
import { Terminal } from 'lucide-react';

export const Navbar = () => {
    const { currentUser, logout } = useAuth();
    const navigate = useNavigate();
    const [profile, setProfile] = useState<UserProfile | null>(null);

    useEffect(() => {
        if (currentUser) {
            userService.getUserProfile(currentUser.uid).then(setProfile);
        } else {
            setProfile(null);
        }
    }, [currentUser]);

    const handleLogout = async () => {
        await logout();
        navigate('/');
    };

    return (
        <nav className="bg-slate-950 border-b border-slate-800 px-6 py-4">
            <div className="container mx-auto flex justify-between items-center">
                <Link to="/" className="text-xl font-bold tracking-tight flex items-center gap-3">
                    <div className="w-8 h-8 bg-slate-900 border border-slate-800 rounded flex items-center justify-center">
                        <Terminal size={16} className="text-orange-500" />
                    </div>
                    <span className="text-slate-100">
                        Droid<span className="text-orange-500">Academy</span>
                    </span>
                </Link>
                <div className="flex gap-4 items-center">
                    {currentUser ? (
                        <>
                            {profile && (
                                <div className="flex items-center gap-2 mr-4">
                                    <span className="text-white font-bold">{profile.xp} <span className="text-slate-500 text-xs font-normal">XP</span></span>
                                    <span className="text-slate-500 text-xs">Lvl {profile.level}</span>
                                </div>
                            )}
                            <span className="text-slate-400 text-sm">
                                {currentUser.displayName || currentUser.email}
                            </span>
                            <button
                                onClick={handleLogout}
                                className="px-4 py-2 text-slate-400 hover:text-white text-sm font-medium transition-colors"
                            >
                                Logout
                            </button>
                        </>
                    ) : (
                        <Link to="/login" className="px-5 py-2 bg-orange-600 text-white text-sm font-bold rounded hover:bg-orange-500 transition-all">Get Started</Link>
                    )}
                </div>
            </div>
        </nav>
    );
};
