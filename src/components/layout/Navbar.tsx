import { useEffect, useState } from 'react';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from '../../contexts/AuthContext';
import { userService } from '../../services/userService';
import type { UserProfile } from '../../types';

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
        <nav className="bg-slate-900 border-b border-slate-800 p-4">
            <div className="container mx-auto flex justify-between items-center">
                <Link to="/" className="text-xl font-bold text-cyan-400">Droid Academy</Link>
                <div className="flex gap-4 items-center">
                    {currentUser ? (
                        <>
                            {profile && (
                                <div className="flex items-center gap-2 mr-4">
                                    <span className="text-cyan-400 font-bold">{profile.xp} XP</span>
                                    <span className="text-slate-500 text-xs">Lvl {profile.level}</span>
                                </div>
                            )}
                            <span className="text-slate-300 text-sm">
                                {currentUser.displayName || currentUser.email}
                            </span>
                            <button
                                onClick={handleLogout}
                                className="text-slate-300 hover:text-white text-sm"
                            >
                                Logout
                            </button>
                        </>
                    ) : (
                        <Link to="/login" className="text-slate-300 hover:text-white">Login</Link>
                    )}
                </div>
            </div>
        </nav>
    );
};
