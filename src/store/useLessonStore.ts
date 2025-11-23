import { create } from 'zustand';
import type { Lesson, RobotState, TerminalEntry } from '../types';
import { userService } from '../services/userService';

interface LessonStore {
    currentLesson: Lesson | null;
    terminalHistory: TerminalEntry[];
    robotState: RobotState;

    // Actions
    setCurrentLesson: (lesson: Lesson) => void;
    addLog: (type: TerminalEntry['type'], content: string) => void;
    clearLogs: () => void;
    updateRobotState: (updates: Partial<RobotState>) => void;
    resetRobot: () => void;
    completeLesson: (uid: string) => Promise<void>;
}

const initialRobotState: RobotState = {
    isNodeRunning: false,
    activeNodes: [],
    activeTopics: [],
    position: { x: 0, y: 0 },
    battery: 100,
};

export const useLessonStore = create<LessonStore>((set, get) => ({
    currentLesson: null,
    terminalHistory: [
        {
            id: 'init',
            type: 'output',
            content: 'Welcome to Droid Academy Terminal v1.0.0\nType "help" for available commands.',
            timestamp: Date.now(),
        },
    ],
    robotState: initialRobotState,

    setCurrentLesson: (lesson) => set({ currentLesson: lesson }),

    addLog: (type, content) =>
        set((state) => ({
            terminalHistory: [
                ...state.terminalHistory,
                {
                    id: Math.random().toString(36).substr(2, 9),
                    type,
                    content,
                    timestamp: Date.now(),
                },
            ],
        })),

    clearLogs: () => set({ terminalHistory: [] }),

    updateRobotState: (updates) =>
        set((state) => ({
            robotState: { ...state.robotState, ...updates },
        })),

    resetRobot: () => set({ robotState: initialRobotState }),

    completeLesson: async (uid: string) => {
        const { currentLesson } = get();
        if (currentLesson) {
            // Import dynamically to avoid circular dependency if any, or just import at top
            // But we need to import userService.
            // Let's assume we imported it.
            await userService.updateUserProgress(uid, currentLesson.id, currentLesson.xpReward);
            set((state) => ({
                terminalHistory: [
                    ...state.terminalHistory,
                    {
                        id: Math.random().toString(36).substr(2, 9),
                        type: 'success',
                        content: `\n🎉 Lesson Completed! +${currentLesson.xpReward} XP\n`,
                        timestamp: Date.now(),
                    }
                ]
            }));
        }
    }
}));
