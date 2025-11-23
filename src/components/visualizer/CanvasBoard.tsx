import { useEffect, useRef } from 'react';
import { useLessonStore } from '../../store/useLessonStore';

export const CanvasBoard = () => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const { robotState } = useLessonStore();

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // Handle resizing
        const resizeCanvas = () => {
            const parent = canvas.parentElement;
            if (parent) {
                canvas.width = parent.clientWidth;
                canvas.height = parent.clientHeight;
            }
            draw(ctx, canvas.width, canvas.height);
        };

        window.addEventListener('resize', resizeCanvas);
        resizeCanvas(); // Initial draw

        return () => window.removeEventListener('resize', resizeCanvas);
    }, [robotState]); // Re-draw when robotState changes

    const draw = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
        // Clear canvas
        ctx.fillStyle = '#0f172a'; // slate-950
        ctx.fillRect(0, 0, width, height);

        // Draw Grid
        ctx.strokeStyle = '#1e293b'; // slate-800
        ctx.lineWidth = 1;
        const gridSize = 40;
        const offsetX = width / 2;
        const offsetY = height / 2;

        // Vertical lines
        for (let x = offsetX % gridSize; x < width; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }

        // Horizontal lines
        for (let y = offsetY % gridSize; y < height; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Draw Origin Axes
        ctx.strokeStyle = '#334155'; // slate-700
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(offsetX, 0);
        ctx.lineTo(offsetX, height);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(0, offsetY);
        ctx.lineTo(width, offsetY);
        ctx.stroke();

        // Draw Robot Path
        if (robotState.path.length > 1) {
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';

            // We need to draw segments because color/width might change
            for (let i = 1; i < robotState.path.length; i++) {
                const p1 = robotState.path[i - 1];
                const p2 = robotState.path[i];

                if (p2.penDown) {
                    ctx.beginPath();
                    ctx.strokeStyle = p2.color;
                    ctx.lineWidth = p2.width;
                    ctx.moveTo(offsetX + p1.x * gridSize, offsetY - p1.y * gridSize);
                    ctx.lineTo(offsetX + p2.x * gridSize, offsetY - p2.y * gridSize);
                    ctx.stroke();
                }
            }
        }

        // Draw Robot (Turtle) ONLY if turtlesim_node is running
        const isTurtlesimRunning = robotState.activeNodes.includes('turtlesim_node');

        if (isTurtlesimRunning) {
            const robotX = offsetX + robotState.position.x * gridSize;
            const robotY = offsetY - robotState.position.y * gridSize; // Invert Y for standard coord system

            ctx.save();
            ctx.translate(robotX, robotY);
            ctx.rotate(-robotState.position.theta); // Rotate context (negative for standard ROS angle)

            // Draw Turtle Body
            ctx.fillStyle = '#06b6d4'; // cyan-500
            ctx.beginPath();
            // Simple turtle shape
            ctx.ellipse(0, 0, 15, 12, 0, 0, Math.PI * 2);
            ctx.fill();

            // Legs
            ctx.fillStyle = '#0891b2'; // cyan-600
            ctx.beginPath(); ctx.arc(12, 10, 4, 0, Math.PI * 2); ctx.fill();
            ctx.beginPath(); ctx.arc(12, -10, 4, 0, Math.PI * 2); ctx.fill();
            ctx.beginPath(); ctx.arc(-10, 10, 4, 0, Math.PI * 2); ctx.fill();
            ctx.beginPath(); ctx.arc(-10, -10, 4, 0, Math.PI * 2); ctx.fill();

            // Head
            ctx.fillStyle = '#ecfeff'; // cyan-50
            ctx.beginPath();
            ctx.arc(14, 0, 6, 0, Math.PI * 2);
            ctx.fill();

            // Eyes
            ctx.fillStyle = '#000';
            ctx.beginPath(); ctx.arc(16, 2, 1, 0, Math.PI * 2); ctx.fill();
            ctx.beginPath(); ctx.arc(16, -2, 1, 0, Math.PI * 2); ctx.fill();

            ctx.restore();
        } else {
            // Draw placeholder text if no simulation is running
            ctx.font = '14px sans-serif';
            ctx.fillStyle = '#475569'; // slate-600
            ctx.textAlign = 'center';
            ctx.fillText('No Simulation Running', width / 2, height / 2);
            ctx.font = '12px sans-serif';
            ctx.fillText('Type "ros2 run turtlesim turtlesim_node" to start', width / 2, height / 2 + 20);
        }

        // Draw Active Nodes (Visual representation)
        if (robotState.activeNodes.length > 0) {
            ctx.font = '12px monospace';
            ctx.fillStyle = '#94a3b8'; // slate-400
            ctx.textAlign = 'left';
            ctx.fillText(`Active Nodes: ${robotState.activeNodes.join(', ')}`, 10, 20);
        }

        // Draw Active Topics
        if (robotState.activeTopics.length > 0) {
            ctx.font = '12px monospace';
            ctx.fillStyle = '#94a3b8'; // slate-400
            ctx.textAlign = 'left';
            ctx.fillText(`Active Topics: ${robotState.activeTopics.join(', ')}`, 10, 40);
        }
    };

    return (
        <div className="w-full h-full bg-slate-950 relative overflow-hidden">
            <canvas ref={canvasRef} className="block" />
            {/* Overlay UI elements could go here */}
        </div>
    );
};
