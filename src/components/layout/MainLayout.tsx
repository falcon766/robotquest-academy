import { Navbar } from './Navbar';
import { Sidebar } from './Sidebar';
import { Outlet, useLocation } from 'react-router-dom';

export const MainLayout = () => {
    const location = useLocation();
    const isLanding = location.pathname === '/';

    return (
        <div className="h-screen bg-black text-slate-200 flex flex-col overflow-hidden">
            <Navbar />
            <div className="flex flex-1 overflow-hidden">
                {!isLanding && <Sidebar />}
                <main className="flex-1 flex flex-col min-w-0">
                    <Outlet />
                </main>
            </div>
        </div>
    );
};
