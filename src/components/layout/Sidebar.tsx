import React from 'react';

export const Sidebar = () => {
    return (
        <aside className="w-64 bg-slate-900 border-r border-slate-800 h-screen p-4 hidden md:block">
            <div className="space-y-4">
                <div className="text-slate-400 text-sm font-semibold uppercase">Modules</div>
                <ul className="space-y-2">
                    <li className="text-slate-300 hover:text-cyan-400 cursor-pointer">1. The Shell</li>
                    <li className="text-slate-300 hover:text-cyan-400 cursor-pointer">2. Nodes</li>
                    <li className="text-slate-300 hover:text-cyan-400 cursor-pointer">3. Topics</li>
                </ul>
            </div>
        </aside>
    );
};
