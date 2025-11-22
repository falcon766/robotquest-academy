import React from 'react';
import { Link } from 'react-router-dom';

export const LandingPage = () => {
    return (
        <div className="flex flex-col items-center justify-center h-full text-center space-y-8">
            <h1 className="text-5xl font-bold text-transparent bg-clip-text bg-gradient-to-r from-cyan-400 to-purple-500">
                Master ROS 2
            </h1>
            <p className="text-xl text-slate-400 max-w-2xl">
                Learn Robot Operating System 2 through interactive simulations in your browser. No installation required.
            </p>
            <Link
                to="/dashboard"
                className="px-8 py-3 bg-cyan-500 hover:bg-cyan-400 text-slate-900 font-bold rounded-lg transition-colors"
            >
                Start Learning
            </Link>
        </div>
    );
};
