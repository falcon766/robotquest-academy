import { Link } from 'react-router-dom';
import { Terminal, Cpu, Zap, ChevronRight } from 'lucide-react';

export const LandingPage = () => {
    return (
        <div className="min-h-screen bg-[#0a0a0a] text-slate-200 overflow-hidden relative font-mono">
            {/* Background Gradients - Darker & More Subtle */}
            <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[1000px] h-[600px] bg-emerald-900/10 rounded-full blur-[120px] -z-10"></div>
            <div className="absolute bottom-0 right-0 w-[800px] h-[600px] bg-indigo-900/10 rounded-full blur-[100px] -z-10"></div>

            {/* Navbar */}
            <nav className="container mx-auto px-6 py-6 flex justify-between items-center">
                <div className="text-xl font-bold tracking-tight flex items-center gap-3">
                    <div className="w-8 h-8 bg-slate-900 border border-slate-800 rounded flex items-center justify-center shadow-lg shadow-emerald-900/20">
                        <Terminal size={16} className="text-emerald-500" />
                    </div>
                    <span className="text-slate-100">
                        Droid<span className="text-emerald-500">Academy</span>_
                    </span>
                </div>
                <div className="flex gap-4">
                    <Link to="/login" className="px-4 py-2 text-slate-400 hover:text-emerald-400 transition-colors text-sm">Login</Link>
                    <Link to="/login" className="px-5 py-2 bg-emerald-600 text-white text-sm font-medium rounded hover:bg-emerald-500 transition-all shadow-lg shadow-emerald-900/50">
                        Initialize_
                    </Link>
                </div>
            </nav>

            {/* Hero Section */}
            <main className="container mx-auto px-6 pt-20 pb-32 text-center">
                <div className="inline-flex items-center gap-2 px-3 py-1 rounded bg-slate-900 border border-slate-800 text-emerald-500 text-xs font-mono mb-8">
                    <span className="relative flex h-2 w-2">
                        <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
                        <span className="relative inline-flex rounded-full h-2 w-2 bg-emerald-500"></span>
                    </span>
                    SYSTEM_ONLINE // READY_TO_LEARN
                </div>

                <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6 max-w-4xl mx-auto leading-tight text-white">
                    Master Robotics.<br />
                    <span className="text-slate-500">No Hardware Required.</span>
                </h1>

                <p className="text-lg text-slate-400 mb-10 max-w-2xl mx-auto leading-relaxed">
                    Interactive ROS 2 terminal simulations in your browser.
                    Write code, visualize nodes, and deploy virtual robots instantly.
                </p>

                <div className="flex flex-col sm:flex-row gap-4 justify-center items-center mb-24">
                    <Link to="/login" className="px-8 py-3 bg-emerald-600 text-white rounded font-medium hover:bg-emerald-500 transition-all flex items-center gap-2 group shadow-lg shadow-emerald-900/20">
                        Start_Simulation
                        <ChevronRight size={16} className="group-hover:translate-x-1 transition-transform" />
                    </Link>
                    <a href="#features" className="px-8 py-3 bg-slate-900 border border-slate-800 text-slate-300 rounded font-medium hover:border-slate-700 transition-all">
                        View_Logs
                    </a>
                </div>

                {/* Feature Grid - Fixed Spacing */}
                <div className="grid md:grid-cols-3 gap-6 max-w-6xl mx-auto text-left">
                    <div className="p-6 rounded border border-slate-800 bg-slate-900/50 hover:border-emerald-500/50 transition-colors group">
                        <div className="w-10 h-10 bg-slate-800 rounded flex items-center justify-center mb-4 group-hover:bg-emerald-500/10 transition-colors">
                            <Terminal className="text-emerald-500" size={20} />
                        </div>
                        <h3 className="text-lg font-bold mb-2 text-slate-200">Terminal_Access</h3>
                        <p className="text-slate-400 text-sm leading-relaxed">Execute ROS 2 commands directly in your browser. Full bash simulation included.</p>
                    </div>
                    <div className="p-6 rounded border border-slate-800 bg-slate-900/50 hover:border-indigo-500/50 transition-colors group">
                        <div className="w-10 h-10 bg-slate-800 rounded flex items-center justify-center mb-4 group-hover:bg-indigo-500/10 transition-colors">
                            <Cpu className="text-indigo-500" size={20} />
                        </div>
                        <h3 className="text-lg font-bold mb-2 text-slate-200">Visualizer_Lite</h3>
                        <p className="text-slate-400 text-sm leading-relaxed">Real-time 2D visualization of nodes, topics, and robot state. Debug visually.</p>
                    </div>
                    <div className="p-6 rounded border border-slate-800 bg-slate-900/50 hover:border-amber-500/50 transition-colors group">
                        <div className="w-10 h-10 bg-slate-800 rounded flex items-center justify-center mb-4 group-hover:bg-amber-500/10 transition-colors">
                            <Zap className="text-amber-500" size={20} />
                        </div>
                        <h3 className="text-lg font-bold mb-2 text-slate-200">XP_System</h3>
                        <p className="text-slate-400 text-sm leading-relaxed">Earn experience points for every command. Level up your engineering rank.</p>
                    </div>
                </div>
            </main>
        </div>
    );
};

