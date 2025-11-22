import { create } from 'zustand';
import type { Lesson, RobotState, TerminalEntry } from '../types';

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
}

const initialRobotState: RobotState = {
    isNodeRunning: false,
    activeNodes: [],
    activeTopics: [],
    position: { x: 0, y: 0 },
    battery: 100,
};

export const useLessonStore = create<LessonStore>((set) => ({
    currentLesson: null,
    terminalHistory: [
        {
            id: 'init',
            type: 'output',
            content: 'Welcome to RoboQuest Academy Terminal v1.0.0\nType "help" for available commands.',
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
}));
