import { useLessonStore } from '../../store/useLessonStore';

export const LessonContent = () => {
    const { currentLesson } = useLessonStore();

    if (!currentLesson) {
        return (
            <div className="flex items-center justify-center h-full text-slate-500">
                <div className="text-center">
                    <p className="text-lg mb-2">No lesson selected</p>
                    <p className="text-sm">Click a module in the sidebar to begin</p>
                </div>
            </div>
        );
    }

    return (
        <div className="p-8 max-w-3xl">
            <div className="mb-6">
                <h1 className="text-3xl font-bold text-white mb-2">{currentLesson.title}</h1>
                <div className="flex items-center gap-3">
                    <span className="px-3 py-1 bg-orange-500/10 text-orange-400 text-xs font-medium rounded border border-orange-500/20">
                        {currentLesson.xpReward} XP
                    </span>
                </div>
            </div>

            <div className="prose prose-invert prose-slate max-w-none">
                <div className="bg-slate-900/50 border border-slate-800 rounded-lg p-6 mb-6">
                    <h2 className="text-xl font-semibold text-white mb-4">Instructions</h2>
                    <div className="text-slate-300 leading-relaxed space-y-4">
                        {currentLesson.contentMarkdown.split('\n').map((line, i) => {
                            if (line.startsWith('#')) {
                                return null; // Skip markdown headers since we have a custom title
                            }
                            if (line.includes('`')) {
                                // Simple code inline rendering
                                const parts = line.split('`');
                                return (
                                    <p key={i}>
                                        {parts.map((part, j) =>
                                            j % 2 === 0 ?
                                                part :
                                                <code key={j} className="px-2 py-1 bg-slate-800 text-orange-400 rounded text-sm font-mono">{part}</code>
                                        )}
                                    </p>
                                );
                            }
                            if (line.trim()) {
                                return <p key={i} className="text-slate-300">{line}</p>;
                            }
                            return null;
                        })}
                    </div>
                </div>

                <div className="bg-blue-500/5 border border-blue-500/20 rounded-lg p-4">
                    <p className="text-sm text-blue-400">
                        💡 <strong>Tip:</strong> Use the terminal on the right to execute commands. Your progress is saved automatically!
                    </p>
                </div>
            </div>
        </div>
    );
};
