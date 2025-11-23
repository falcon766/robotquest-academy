import { useLessonStore } from '../../store/useLessonStore';
import { Terminal, ChevronRight, CheckCircle } from 'lucide-react';
import { curriculum } from '../../data/curriculum';
import { useAuth } from '../../contexts/AuthContext';
import { useEffect, useState } from 'react';
import { userService } from '../../services/userService';
import { useNavigate } from 'react-router-dom';

export const Sidebar = () => {
    const { setCurrentLesson, currentLesson } = useLessonStore();
    const { currentUser } = useAuth();
    const navigate = useNavigate();
    const [completedLessons, setCompletedLessons] = useState<string[]>([]);
    const [openModules, setOpenModules] = useState<string[]>(['module_1']);

    useEffect(() => {
        if (currentUser) {
            userService.getUserProfile(currentUser.uid).then(profile => {
                if (profile) setCompletedLessons(profile.completedLessons);
            });
        }

        const handleCompletion = () => {
            if (currentUser) {
                userService.getUserProfile(currentUser.uid).then(profile => {
                    if (profile) setCompletedLessons(profile.completedLessons);
                });
            }
        };
        window.addEventListener('lessonCompleted', handleCompletion);
        return () => window.removeEventListener('lessonCompleted', handleCompletion);
    }, [currentUser]);

    const toggleModule = (moduleId: string) => {
        setOpenModules(prev =>
            prev.includes(moduleId) ? prev.filter(id => id !== moduleId) : [...prev, moduleId]
        );
    };

    const handleLessonClick = (lesson: any) => {
        setCurrentLesson(lesson);
        navigate('/learn');
    };

    return (
        <aside className="w-72 bg-black border-r border-slate-800 h-full flex flex-col overflow-y-auto scrollbar-thin scrollbar-thumb-slate-800">
            <div className="p-4">
                <div className="text-slate-500 text-xs font-bold uppercase tracking-widest px-2 mb-4">Curriculum</div>

                <div className="space-y-6">
                    {curriculum.map(course => (
                        <div key={course.id}>
                            <h3 className="text-orange-500 font-bold text-sm px-2 mb-2 uppercase tracking-wider">{course.title}</h3>
                            <div className="space-y-4">
                                {course.modules.map(module => (
                                    <div key={module.id} className="space-y-1">
                                        <button
                                            onClick={() => toggleModule(module.id)}
                                            className="w-full flex items-center justify-between px-2 py-1.5 text-slate-300 hover:text-white transition-colors group"
                                        >
                                            <span className="font-medium text-sm">{module.title}</span>
                                            <ChevronRight size={14} className={`transition-transform ${openModules.includes(module.id) ? 'rotate-90' : ''}`} />
                                        </button>

                                        {openModules.includes(module.id) && (
                                            <div className="space-y-1 pl-2">
                                                {module.lessons.map(lesson => {
                                                    const isCompleted = completedLessons.includes(lesson.id);
                                                    const isActive = currentLesson?.id === lesson.id;

                                                    return (
                                                        <button
                                                            key={lesson.id}
                                                            onClick={() => handleLessonClick(lesson)}
                                                            className={`w-full flex items-center gap-3 px-3 py-2.5 rounded-lg border transition-all group text-left ${isActive
                                                                ? 'bg-slate-900 border-orange-500/50 text-white'
                                                                : 'bg-transparent border-transparent text-slate-400 hover:bg-slate-900/50 hover:text-slate-200'
                                                                }`}
                                                        >
                                                            {isCompleted ? (
                                                                <CheckCircle size={16} className="text-green-500 shrink-0" />
                                                            ) : (
                                                                <Terminal size={16} className={`${isActive ? 'text-orange-500' : 'text-slate-600 group-hover:text-slate-400'} shrink-0`} />
                                                            )}
                                                            <span className="text-sm font-medium truncate">{lesson.title}</span>
                                                        </button>
                                                    );
                                                })}
                                            </div>
                                        )}
                                    </div>
                                ))}
                            </div>
                        </div>
                    ))}
                </div>
            </div>
        </aside>
    );
};
