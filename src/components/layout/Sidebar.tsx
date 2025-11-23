
import { useLessonStore } from '../../store/useLessonStore';

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
        <aside className="w-64 bg-slate-900 border-r border-slate-800 h-screen p-4 hidden md:block">
            <div className="space-y-4">
                <div className="text-slate-400 text-sm font-semibold uppercase">Modules</div>
                <ul className="space-y-2">
                    <li
                        onClick={startLesson1}
                        className="text-slate-300 hover:text-cyan-400 cursor-pointer hover:bg-slate-800 p-2 rounded transition-colors"
                    >
                        1. The Shell (Click to Start)
                    </li>
                    <li className="text-slate-300 hover:text-cyan-400 cursor-pointer p-2">2. Nodes</li>
                    <li className="text-slate-300 hover:text-cyan-400 cursor-pointer p-2">3. Topics</li>
                </ul>
            </div>
        </aside>
    );
};
