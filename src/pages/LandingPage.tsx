import { Link } from 'react-router-dom';
import { Terminal, Cpu, Zap, ChevronRight } from 'lucide-react';

export const LandingPage = () => {
    return (
        <div className="min-h-screen bg-[#0a0a0a] text-slate-200 overflow-hidden relative font-sans selection:bg-blue-500/30">
            {/* Subtle Grid Background */}
            <div className="absolute inset-0 bg-[linear-gradient(to_right,#80808012_1px,transparent_1px),linear-gradient(to_bottom,#80808012_1px,transparent_1px)] bg-[size:24px_24px]"></div>

            {/* Navbar */}
            <nav className="container mx-auto px-6 py-6 flex justify-between items-center relative z-10">
                <div className="text-xl font-bold tracking-tight flex items-center gap-3">
                    <div className="w-8 h-8 bg-slate-900 border border-slate-800 rounded flex items-center justify-center">
                        <Terminal size={16} className="text-orange-500" />
                    </div>
                    <span className="text-slate-100">
                        Droid<span className="text-orange-500">Academy</span>
                    </span>
                </div>
                <div className="flex gap-4">
                    <Link to="/login" className="px-4 py-2 text-slate-400 hover:text-white transition-colors text-sm font-medium">Login</Link>
                    <Link to="/login" className="px-5 py-2 bg-orange-600 text-white text-sm font-bold rounded hover:bg-orange-500 transition-all">
                        Get Started
                    </Link>
                </div>
            </nav>

            {/* Hero Section */}
            <main className="container mx-auto px-6 pt-24 pb-32 text-center relative z-10">
                <div className="inline-flex items-center gap-2 px-3 py-1 rounded-full bg-slate-900/50 border border-slate-800 text-slate-400 text-xs font-medium mb-8 backdrop-blur-sm">
                    <span className="relative flex h-2 w-2">
                        <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-orange-400 opacity-75"></span>
                        <span className="relative inline-flex rounded-full h-2 w-2 bg-orange-500"></span>
                    </span>
                    v1.0 Public Beta
                </div>

                <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-8 max-w-4xl mx-auto leading-tight text-white">
                    Master Robotics.<br />
                    <span className="text-slate-500">Zero Hardware.</span>
                </h1>

                <p className="text-xl text-slate-400 mb-12 max-w-2xl mx-auto leading-relaxed font-light">
                    The modern way to learn ROS 2. Interactive terminal simulations,
                    real-time visualization, and gamified progress.
                </p>

                <div className="flex flex-col sm:flex-row gap-4 justify-center items-center mb-24">
                    <Link to="/login" className="px-8 py-4 bg-orange-600 text-white rounded-lg font-semibold hover:bg-orange-500 transition-all flex items-center gap-2 group shadow-lg shadow-orange-900/20">
                        Start Learning Free
                        <ChevronRight size={18} className="group-hover:translate-x-1 transition-transform" />
                    </Link>
                    <a href="#features" className="px-8 py-4 bg-slate-900 border border-slate-800 text-slate-300 rounded-lg font-medium hover:bg-slate-800 transition-all">
                        View Curriculum
                    </a>
                </div>

                {/* Feature Grid */}
                <div className="grid md:grid-cols-3 gap-6 max-w-6xl mx-auto text-left">
                    <div className="p-8 rounded-xl border border-slate-800 bg-slate-900/50 hover:border-slate-700 transition-colors group">
                        <div className="w-12 h-12 bg-slate-800 rounded-lg flex items-center justify-center mb-6 group-hover:bg-orange-500/10 transition-colors">
                            <Terminal className="text-orange-500" size={24} />
                        </div>
                        <h3 className="text-xl font-bold mb-3 text-white">Cloud Terminal</h3>
                        <p className="text-slate-400 leading-relaxed">Execute ROS 2 commands directly in your browser. No Linux installation or complex setup required.</p>
                    </div>
                    <div className="p-8 rounded-xl border border-slate-800 bg-slate-900/50 hover:border-slate-700 transition-colors group">
                        <div className="w-12 h-12 bg-slate-800 rounded-lg flex items-center justify-center mb-6 group-hover:bg-slate-700 transition-colors">
                            <Cpu className="text-slate-400" size={24} />
                        </div>
                        <h3 className="text-xl font-bold mb-3 text-white">Visualizer</h3>
                        <p className="text-slate-400 leading-relaxed">See your nodes and topics in action with our integrated 2D visualizer. Debug faster and learn better.</p>
                    </div>
                    <div className="p-8 rounded-xl border border-slate-800 bg-slate-900/50 hover:border-slate-700 transition-colors group">
                        <div className="w-12 h-12 bg-slate-800 rounded-lg flex items-center justify-center mb-6 group-hover:bg-slate-700 transition-colors">
                            <Zap className="text-slate-400" size={24} />
                        </div>
                        <h3 className="text-xl font-bold mb-3 text-white">Progress Tracking</h3>
                        <p className="text-slate-400 leading-relaxed">Earn XP, level up, and maintain streaks. Track your journey from novice to robotics engineer.</p>
                    </div>
                </div>
            </main>
        </div>
    );
};

