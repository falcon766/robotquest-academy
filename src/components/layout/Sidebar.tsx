
import { useLessonStore } from '../../store/useLessonStore';
import { Terminal, Share2, MessageSquare } from 'lucide-react';

export const Sidebar = () => {
    const { setCurrentLesson } = useLessonStore();

    const startLesson1 = () => {
        setCurrentLesson({
            id: 'lesson_1',
            title: 'Hello World',
            contentMarkdown: '# Hello World\n\nType `echo "Hello ROS"` to complete this lesson.',
            initialCode: '',
            expectedCommand: 'echo "Hello ROS"',
            successMessage: 'Great job! You spoke to the terminal.',
            xpReward: 10,
        });
    };

    return (
        <aside className="w-64 bg-black border-r border-slate-800 h-screen p-4 hidden md:flex flex-col">
            <div className="space-y-6">
                <div className="text-slate-500 text-xs font-bold uppercase tracking-widest px-2">Curriculum</div>
                <div className="space-y-1">
                    <button
                        onClick={startLesson1}
                        className="w-full flex items-center gap-3 px-3 py-2.5 bg-slate-900 text-slate-200 rounded-lg border border-slate-800 hover:border-orange-500/50 hover:bg-slate-800 transition-all group text-left"
                    >
                        <Terminal size={18} className="text-slate-500 group-hover:text-orange-400 transition-colors" />
                        <span className="text-sm font-medium">The Shell</span>
                    </button>

                    <button className="w-full flex items-center gap-3 px-3 py-2.5 text-slate-500 rounded-lg border border-transparent hover:bg-slate-900/50 transition-all text-left cursor-not-allowed opacity-60">
                        <Share2 size={18} />
                        <span className="text-sm font-medium">Nodes</span>
                    </button>

                    <button className="w-full flex items-center gap-3 px-3 py-2.5 text-slate-500 rounded-lg border border-transparent hover:bg-slate-900/50 transition-all text-left cursor-not-allowed opacity-60">
                        <MessageSquare size={18} />
                        <span className="text-sm font-medium">Topics</span>
                    </button>
                </div>
            </div>
        </aside>
    );
};
