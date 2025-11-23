import { useEffect, useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useNavigate, Link } from 'react-router-dom';
import { Terminal } from 'lucide-react';

export const LoginPage = () => {
    const { loginWithGoogle, currentUser } = useAuth();
    const navigate = useNavigate();
    const [error, setError] = useState('');

    useEffect(() => {
        if (currentUser) {
            navigate('/dashboard');
        }
    }, [currentUser, navigate]);

    const handleGoogleLogin = async () => {
        try {
            setError('');
            await loginWithGoogle();
            navigate('/dashboard');
        } catch (err) {
            setError('Failed to log in with Google.');
            console.error(err);
        }
    };

    return (
        <div className="min-h-screen bg-black flex flex-col items-center justify-center p-4 relative overflow-hidden">
            {/* Background Grid */}
            <div className="absolute inset-0 bg-[linear-gradient(to_right,#80808012_1px,transparent_1px),linear-gradient(to_bottom,#80808012_1px,transparent_1px)] bg-[size:24px_24px]"></div>

            <div className="w-full max-w-md relative z-10">
                <div className="text-center mb-8">
                    <Link to="/" className="inline-flex items-center gap-3 text-2xl font-bold tracking-tight mb-2 hover:opacity-80 transition-opacity">
                        <div className="w-10 h-10 bg-slate-900 border border-slate-800 rounded flex items-center justify-center">
                            <Terminal size={20} className="text-orange-500" />
                        </div>
                        <span className="text-slate-100">
                            Droid<span className="text-orange-500">Academy</span>
                        </span>
                    </Link>
                    <p className="text-slate-400">Sign in to continue your robotics journey</p>
                </div>

                <div className="bg-slate-900/50 border border-slate-800 rounded-xl p-8 shadow-2xl backdrop-blur-sm">
                    {error && (
                        <div className="bg-red-500/10 border border-red-500/50 text-red-400 p-3 rounded mb-6 text-sm">
                            {error}
                        </div>
                    )}

                    <button
                        onClick={handleGoogleLogin}
                        className="w-full py-3.5 px-4 bg-white text-slate-900 font-bold rounded-lg hover:bg-slate-200 transition-all flex items-center justify-center gap-3 shadow-lg group"
                    >
                        <svg className="w-5 h-5 group-hover:scale-110 transition-transform" viewBox="0 0 24 24">
                            <path
                                fill="currentColor"
                                d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
                            />
                            <path
                                fill="currentColor"
                                d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
                            />
                            <path
                                fill="currentColor"
                                d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
                            />
                            <path
                                fill="currentColor"
                                d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
                            />
                        </svg>
                        Sign in with Google
                    </button>

                    <div className="mt-6 text-center">
                        <Link to="/" className="text-sm text-slate-500 hover:text-slate-400 transition-colors">
                            ← Back to Home
                        </Link>
                    </div>
                </div>
            </div>
        </div>
    );
};
