import React, { useState, useRef, useEffect } from 'react';
import { useLessonStore } from '../../store/useLessonStore';
import { parseCommand } from '../../utils/commandParser';
import { useAuth } from '../../contexts/AuthContext';

export const Terminal = () => {
    const { terminalHistory, addLog, currentLesson, updateRobotState, clearLogs, completeLesson, robotState } = useLessonStore();
    const { currentUser } = useAuth();
    const [input, setInput] = useState('');
    const [historyIndex, setHistoryIndex] = useState(-1);
    const inputRef = useRef<HTMLInputElement>(null);
    const bottomRef = useRef<HTMLDivElement>(null);

    // Auto-scroll to bottom
    useEffect(() => {
        bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [terminalHistory]);

    // Focus input on click
    const handleContainerClick = () => {
        inputRef.current?.focus();
    };

    const handleKeyDown = (e: React.KeyboardEvent) => {
        // Check if teleop is running
        const isTeleopRunning = robotState.activeNodes.includes('turtle_teleop_key');

        if (isTeleopRunning) {
            // Handle Ctrl+C to stop
            if (e.ctrlKey && e.key === 'c') {
                e.preventDefault();
                const newActiveNodes = robotState.activeNodes.filter(n => n !== 'turtle_teleop_key');
                updateRobotState({ activeNodes: newActiveNodes });
                addLog('output', '^C\n[INFO] [teleop_turtle]: Stopping teleop node');
                return;
            }

            // Handle Arrow Keys for Movement
            if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
                e.preventDefault();

                const linearSpeed = 2.0;
                const angularSpeed = Math.PI / 8; // Exactly 22.5 degrees
                const dt = 0.5; // Discrete step size

                let linear = 0;
                let angular = 0;

                if (e.key === 'ArrowUp') linear = linearSpeed;
                if (e.key === 'ArrowDown') linear = -linearSpeed;
                if (e.key === 'ArrowLeft') angular = angularSpeed;
                if (e.key === 'ArrowRight') angular = -angularSpeed;

                const current = robotState.position;
                const newTheta = current.theta + angular * dt;
                const newX = current.x + Math.cos(newTheta) * linear * dt;
                const newY = current.y + Math.sin(newTheta) * linear * dt;

                // Update Path
                const newPath = [...robotState.path, {
                    x: newX,
                    y: newY,
                    penDown: robotState.pen.isDown,
                    color: robotState.pen.color,
                    width: robotState.pen.width
                }];

                updateRobotState({
                    position: { x: newX, y: newY, theta: newTheta },
                    path: newPath
                });

                // Feedback for the user
                addLog('output', `[INFO] [teleop]: vel(lin=${linear.toFixed(1)}, ang=${angular.toFixed(1)}) -> Pose(x=${newX.toFixed(2)}, y=${newY.toFixed(2)}, theta=${newTheta.toFixed(2)})`);
                return;
            }

            // If teleop is running, we might want to block other inputs or just let them type?
            // Real ros2 run takes over stdin. Let's block typing for now to simulate that.
            if (e.key.length === 1) {
                e.preventDefault();
                return;
            }
        }

        if (e.key === 'Enter') {
            e.preventDefault();
            if (!input.trim()) return;

            // Add input to log
            addLog('input', input);

            // Parse command
            const result = parseCommand(input, currentLesson);

            // Handle actions
            if (result.action === 'clear') {
                clearLogs();
            } else {
                if (result.output) {
                    addLog(result.success ? 'output' : 'error', result.output);
                }

                if (result.success && currentLesson && currentUser) {
                    // Check if it was a lesson completion (simple check: if parseCommand returned success for a lesson)
                    // parseCommand returns success: true if it matched the lesson expectation.
                    // We should probably have a more explicit flag in CommandResult, but this works for now.
                    completeLesson(currentUser.uid);
                }

                if (result.action === 'start_node') {
                    const currentNodes = robotState.activeNodes || [];
                    const newNode = result.payload.executable;

                    // Avoid duplicates
                    const newActiveNodes = currentNodes.includes(newNode)
                        ? currentNodes
                        : [...currentNodes, newNode];

                    updateRobotState({
                        isNodeRunning: true,
                        activeNodes: newActiveNodes
                    });
                }
                if (result.action === 'publish_topic') {
                    updateRobotState({ activeTopics: [result.payload.topic] });
                }
            }

            setInput('');
            setHistoryIndex(-1);
        } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            // Simple history navigation (could be improved)
            const inputs = terminalHistory.filter(h => h.type === 'input');
            if (inputs.length > 0) {
                const newIndex = historyIndex === -1 ? inputs.length - 1 : Math.max(0, historyIndex - 1);
                setHistoryIndex(newIndex);
                setInput(inputs[newIndex].content);
            }
        } else if (e.key === 'ArrowDown') {
            e.preventDefault();
            const inputs = terminalHistory.filter(h => h.type === 'input');
            if (historyIndex !== -1) {
                const newIndex = Math.min(inputs.length - 1, historyIndex + 1);
                setHistoryIndex(newIndex);
                setInput(inputs[newIndex].content);

                if (newIndex === inputs.length - 1) {
                    // If at end, maybe clear? For now just stay at last
                }
            }
        }
    };

    return (
        <div
            className="bg-slate-950 font-mono text-sm p-4 h-full overflow-hidden flex flex-col border-t border-slate-800"
            onClick={handleContainerClick}
        >
            <div className="flex-1 overflow-y-auto space-y-1 custom-scrollbar">
                {terminalHistory.map((entry) => (
                    <div key={entry.id} className={`${entry.type === 'error' ? 'text-red-400' : entry.type === 'input' ? 'text-slate-300' : 'text-slate-400'}`}>
                        {entry.type === 'input' && <span className="text-green-500 mr-2">$</span>}
                        <span className="whitespace-pre-wrap">{entry.content}</span>
                    </div>
                ))}
                <div ref={bottomRef} />
            </div>

            <div className="flex items-center mt-2">
                <span className="text-green-500 mr-2">$</span>
                <input
                    ref={inputRef}
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onKeyDown={handleKeyDown}
                    className="flex-1 bg-transparent border-none outline-none text-slate-200 placeholder-slate-600"
                    placeholder={robotState.activeNodes.includes('turtle_teleop_key') ? "Use arrow keys to drive..." : "Type a command..."}
                    autoComplete="off"
                    autoFocus
                />
            </div>

            {/* Teleop Overlay */}
            {robotState.activeNodes.includes('turtle_teleop_key') && (
                <div className="absolute top-4 right-4 bg-slate-800/90 border border-slate-600 rounded-lg p-3 shadow-xl backdrop-blur-sm z-20 flex flex-col gap-2 animate-in fade-in slide-in-from-top-2">
                    <div className="flex items-center gap-2 text-orange-400 font-bold text-xs uppercase tracking-wider">
                        <span className="relative flex h-2 w-2">
                            <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-orange-400 opacity-75"></span>
                            <span className="relative inline-flex rounded-full h-2 w-2 bg-orange-500"></span>
                        </span>
                        Teleop Active
                    </div>
                    <div className="text-[10px] text-slate-400">
                        Use <span className="text-slate-200 font-mono bg-slate-700 px-1 rounded">Arrow Keys</span> to move
                    </div>
                    <button
                        onClick={() => {
                            const newActiveNodes = robotState.activeNodes.filter(n => n !== 'turtle_teleop_key');
                            updateRobotState({ activeNodes: newActiveNodes });
                            addLog('output', '^C\n[INFO] [teleop_turtle]: Stopping teleop node');
                            inputRef.current?.focus();
                        }}
                        className="mt-1 bg-red-500/10 hover:bg-red-500/20 text-red-400 text-xs py-1 px-2 rounded border border-red-500/30 transition-colors flex items-center justify-center gap-1"
                    >
                        <span>Stop Node (Ctrl+C)</span>
                    </button>
                </div>
            )}
        </div>
    );
};
