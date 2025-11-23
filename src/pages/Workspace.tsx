import { Terminal } from '../components/terminal/Terminal';
import { CanvasBoard } from '../components/visualizer/CanvasBoard';
import { LessonContent } from '../components/lesson/LessonContent';

export const Workspace = () => {
    return (
        <div className="flex-1 flex flex-col md:flex-row overflow-hidden h-full">
            {/* Left Panel: Lesson Content */}
            <div className="flex-1 p-6 overflow-auto border-r border-slate-800">
                <LessonContent />
            </div>

            {/* Right Panel: Interactive Terminal & Visualizer */}
            <div className="flex-1 flex flex-col min-w-[400px] bg-slate-900 border-l border-slate-800">
                {/* Visualizer Area (Top Half) */}
                <div className="flex-1 relative border-b border-slate-800 min-h-[300px]">
                    <div className="absolute top-0 left-0 right-0 z-10 bg-slate-800/80 backdrop-blur px-4 py-2 text-xs font-bold text-slate-400 uppercase tracking-wider flex justify-between items-center">
                        <span>Visualizer (Rviz Lite)</span>
                    </div>
                    <CanvasBoard />
                </div>

                {/* Terminal Area (Bottom Half) */}
                <div className="h-1/3 flex flex-col min-h-[200px]">
                    <div className="bg-slate-800 px-4 py-2 text-xs font-bold text-slate-400 uppercase tracking-wider flex justify-between items-center border-t border-slate-700">
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
        </div>
    );
};
