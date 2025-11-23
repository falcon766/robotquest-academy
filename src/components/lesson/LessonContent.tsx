import { useLessonStore } from '../../store/useLessonStore';
import { useAuth } from '../../contexts/AuthContext';
import { useEffect, useState } from 'react';
import { userService } from '../../services/userService';

export const LessonContent = () => {
    const { currentLesson, setCurrentLesson } = useLessonStore();
    const { currentUser } = useAuth();
    const [isCompleted, setIsCompleted] = useState(false);

    useEffect(() => {
        const checkCompletion = async () => {
            if (currentUser && currentLesson) {
                const profile = await userService.getUserProfile(currentUser.uid);
                setIsCompleted(profile?.completedLessons.includes(currentLesson.id) || false);
            }
        };
        checkCompletion();

        // Listen for lesson completion
        const handleCompletion = () => checkCompletion();
        window.addEventListener('lessonCompleted', handleCompletion);
        return () => window.removeEventListener('lessonCompleted', handleCompletion);
    }, [currentUser, currentLesson]);

    const handleNextLesson = () => {
        // For now, just clear the current lesson (in the future, load lesson_2, etc.)
        setCurrentLesson({
            id: 'coming_soon',
            title: 'Coming Soon',
            contentMarkdown: 'More lessons are on the way! Stay tuned.',
            initialCode: '',
            expectedCommand: '',
            successMessage: '',
            xpReward: 0,
        });
    };

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
                    {isCompleted && (
                        <span className="px-3 py-1 bg-green-500/10 text-green-400 text-xs font-medium rounded border border-green-500/20">
                            ✓ Completed
                        </span>
                    )}
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

                {isCompleted && currentLesson.id !== 'coming_soon' && (
                    <div className="bg-green-500/5 border border-green-500/20 rounded-lg p-6 mb-6">
                        <p className="text-green-400 mb-4">
                            🎉 Great job! You've completed this lesson.
                        </p>
                        <button
                            onClick={handleNextLesson}
                            className="px-6 py-3 bg-orange-600 text-white rounded-lg font-semibold hover:bg-orange-500 transition-all"
                        >
                            Next Lesson →
                        </button>
                    </div>
                )}

                {!isCompleted && (
                    <div className="bg-blue-500/5 border border-blue-500/20 rounded-lg p-4">
                        <p className="text-sm text-blue-400">
                            💡 <strong>Tip:</strong> Use the terminal on the right to execute commands. Your progress is saved automatically!
                        </p>
                    </div>
                )}
            </div>
        </div>
    );
};
