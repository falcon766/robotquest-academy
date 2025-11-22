import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from '../../contexts/AuthContext';

export const Navbar = () => {
    const { currentUser, logout } = useAuth();
    const navigate = useNavigate();

    const handleLogout = async () => {
        await logout();
        navigate('/');
    };

    return (
        <nav className="bg-slate-900 border-b border-slate-800 p-4">
            <div className="container mx-auto flex justify-between items-center">
                <Link to="/" className="text-xl font-bold text-cyan-400">RoboQuest Academy</Link>
                <div className="flex gap-4 items-center">
                    {currentUser ? (
                        <>
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
