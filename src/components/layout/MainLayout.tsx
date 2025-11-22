import { Navbar } from './Navbar';
import { Sidebar } from './Sidebar';
import { Outlet, useLocation } from 'react-router-dom';
import { Terminal } from '../terminal/Terminal';

export const MainLayout = () => {
    const location = useLocation();
    const isDashboard = location.pathname === '/dashboard';

    return (
        <div className="min-h-screen bg-slate-950 text-slate-200 flex flex-col">
            <Navbar />
            <div className="flex flex-1 overflow-hidden">
                <Sidebar />
                <main className="flex-1 flex flex-col min-w-0">
                    {isDashboard ? (
                        <div className="flex-1 p-6 overflow-auto">
                            <Outlet />
                        </div>
                    ) : (
                        <div className="flex-1 flex flex-col md:flex-row overflow-hidden">
                            {/* Left Panel: Lesson Content (Placeholder for now, rendered via Outlet if we had lesson routes) */}
                            <div className="flex-1 p-6 overflow-auto border-r border-slate-800">
                                <Outlet />
                            </div>

                            {/* Right Panel: Interactive Terminal */}
                            <div className="flex-1 flex flex-col min-w-[400px] bg-slate-900">
                                <div className="bg-slate-800 px-4 py-2 text-xs font-bold text-slate-400 uppercase tracking-wider flex justify-between items-center">
                                    <span>Terminal</span>
                                    <div className="flex gap-2">
                                        <div className="w-3 h-3 rounded-full bg-red-500/20"></div>
                                        <div className="w-3 h-3 rounded-full bg-yellow-500/20"></div>
                                        <div className="w-3 h-3 rounded-full bg-green-500/20"></div>
                                    </div>
                                </div>
                                <div className="flex-1 overflow-hidden relative">
                                    <Terminal />
                                </div>
                            </div>
                        </div>
                    )}
                </main>
            </div>
        </div>
    );
};
