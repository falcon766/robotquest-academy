import React from 'react';

export const Dashboard = () => {
    return (
        <div className="space-y-6">
            <h2 className="text-3xl font-bold text-white">Your Progress</h2>
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                {/* Placeholder for progress cards */}
                <div className="p-6 bg-slate-900 border border-slate-800 rounded-xl">
                    <h3 className="text-xl font-bold text-cyan-400 mb-2">Module 1: The Shell</h3>
                    <p className="text-slate-400 mb-4">Learn the basics of the Linux shell and ROS 2 setup.</p>
                    <div className="w-full bg-slate-800 h-2 rounded-full overflow-hidden">
                        <div className="bg-cyan-500 h-full w-0"></div>
                    </div>
                </div>
            </div>
        </div>
    );
};
