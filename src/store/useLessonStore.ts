import { create } from 'zustand';
import type { Lesson, RobotState, TerminalEntry } from '../types';
import { userService } from '../services/userService';

// Virtual File System Types
export interface FileSystemNode {
    name: string;
    type: 'file' | 'directory';
    children?: FileSystemNode[];
    content?: string;
}

interface LessonStore {
    currentLesson: Lesson | null;
    terminalHistory: TerminalEntry[];
    robotState: RobotState;

    // File System State
    fileSystem: FileSystemNode;
    currentPath: string[]; // ['home', 'user', 'workspace']

    // Actions
    setCurrentLesson: (lesson: Lesson) => void;
    addLog: (type: TerminalEntry['type'], content: string) => void;
    clearLogs: () => void;
    updateRobotState: (updates: Partial<RobotState>) => void;
    resetRobot: () => void;
    completeLesson: (uid: string) => Promise<void>;

    // FS Actions
    changeDirectory: (path: string) => string | null; // Returns error string or null if success
    listFiles: () => string;
    createDirectory: (name: string) => string | null;
    createFile: (name: string, content?: string) => string | null;
}

const initialRobotState: RobotState = {
    isNodeRunning: false,
    activeNodes: [],
    activeTopics: [],
    position: { x: 0, y: 0 },
    battery: 100,
};

// Initial Virtual File System
const initialFileSystem: FileSystemNode = {
    name: 'root',
    type: 'directory',
    children: [
        {
            name: 'home',
            type: 'directory',
            children: [
                {
                    name: 'user',
                    type: 'directory',
                    children: [
                        {
                            name: 'workspace',
                            type: 'directory',
                            children: [
                                { name: 'src', type: 'directory', children: [] },
                                { name: 'build', type: 'directory', children: [] },
                                { name: 'install', type: 'directory', children: [] },
                                { name: 'log', type: 'directory', children: [] },
                            ]
                        },
                        { name: 'documents', type: 'directory', children: [] },
                        { name: 'downloads', type: 'directory', children: [] },
                    ]
                }
            ]
        }
    ]
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

    fileSystem: initialFileSystem,
    currentPath: ['home', 'user'], // Start in ~

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
            window.dispatchEvent(new CustomEvent('lessonCompleted'));
        }
    },

    // --- File System Actions ---

    // Helper to get node at current path
    // (Internal logic, but we implement actions directly)

    changeDirectory: (path: string) => {
        const { fileSystem, currentPath } = get();

        // Handle absolute path
        let targetPath = path.startsWith('/') ? [] : [...currentPath];
        const parts = path.split('/').filter(p => p !== '' && p !== '.');

        // Navigate
        // let current: FileSystemNode = fileSystem; // Unused

        // If absolute, start at root
        if (path.startsWith('/')) {
            // Root is implicit in our structure, children of root are top level
            // Actually our root is a node.
        }

        // Simplified navigation logic for the store
        // We need to traverse the tree.

        // Let's rebuild targetPath based on parts
        for (const part of parts) {
            if (part === '..') {
                if (targetPath.length > 0) targetPath.pop();
            } else if (part === '~') {
                targetPath = ['home', 'user'];
            } else {
                // Verify child exists
                // We need to traverse to current targetPath first to check children
                // This is getting complex for a simple store.
                // Let's implement a helper traversal.

                // For now, let's just trust the path update if it looks valid in our simple mock,
                // or better: implement a robust traversal.

                // Let's do a quick traversal to verify validity
                let node = fileSystem;
                // Traverse to current location
                for (const p of targetPath) {
                    const found = node.children?.find(c => c.name === p);
                    if (found) node = found;
                }

                // Check if 'part' exists in current node
                const nextNode = node.children?.find(c => c.name === part);
                if (nextNode && nextNode.type === 'directory') {
                    targetPath.push(part);
                } else {
                    return `cd: no such file or directory: ${path}`;
                }
            }
        }

        set({ currentPath: targetPath });
        return null;
    },

    listFiles: () => {
        const { fileSystem, currentPath } = get();
        let node = fileSystem;
        for (const p of currentPath) {
            const found = node.children?.find(c => c.name === p);
            if (found) node = found;
        }

        if (!node.children || node.children.length === 0) return '';

        return node.children.map(c =>
            c.type === 'directory' ? `\x1b[1;34m${c.name}/\x1b[0m` : c.name
        ).join('  ');
    },

    createDirectory: (name: string) => {
        const { fileSystem, currentPath } = get();
        // Deep clone to avoid mutation issues
        const newFS = JSON.parse(JSON.stringify(fileSystem));

        let node = newFS;
        for (const p of currentPath) {
            const found = node.children?.find((c: any) => c.name === p);
            if (found) node = found;
        }

        if (node.children?.find((c: any) => c.name === name)) {
            return `mkdir: cannot create directory '${name}': File exists`;
        }

        if (!node.children) node.children = [];
        node.children.push({ name, type: 'directory', children: [] });

        set({ fileSystem: newFS });
        return null;
    },

    createFile: (name: string, content: string = '') => {
        const { fileSystem, currentPath } = get();
        const newFS = JSON.parse(JSON.stringify(fileSystem));

        let node = newFS;
        for (const p of currentPath) {
            const found = node.children?.find((c: any) => c.name === p);
            if (found) node = found;
        }

        if (node.children?.find((c: any) => c.name === name)) {
            return `touch: cannot create file '${name}': File exists`;
        }

        if (!node.children) node.children = [];
        node.children.push({ name, type: 'file', content });

        set({ fileSystem: newFS });
        return null;
    }
}));
