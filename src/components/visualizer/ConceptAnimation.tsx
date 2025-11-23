import { motion } from 'framer-motion';

export type AnimationType = 'topic' | 'service' | 'node' | 'action';

interface ConceptAnimationProps {
    type: AnimationType;
}

export const ConceptAnimation = ({ type }: ConceptAnimationProps) => {
    if (type === 'topic') return <TopicAnimation />;
    if (type === 'service') return <ServiceAnimation />;
    if (type === 'node') return <NodeAnimation />;
    return null;
};

const NodeAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="absolute inset-0 grid grid-cols-8 grid-rows-4 gap-4 opacity-10">
                {Array.from({ length: 32 }).map((_, i) => (
                    <div key={i} className="bg-slate-500 rounded-full w-1 h-1" />
                ))}
            </div>

            <motion.div
                className="w-24 h-24 bg-slate-800 border-2 border-orange-500 rounded-xl flex flex-col items-center justify-center z-10 shadow-[0_0_30px_rgba(249,115,22,0.2)]"
                animate={{
                    scale: [1, 1.05, 1],
                    borderColor: ['#f97316', '#fb923c', '#f97316']
                }}
                transition={{ duration: 3, repeat: Infinity }}
            >
                <div className="w-3 h-3 bg-green-500 rounded-full mb-2 shadow-[0_0_10px_rgba(34,197,94,0.6)] animate-pulse"></div>
                <span className="text-slate-200 font-bold text-sm">Node</span>
                <span className="text-xs text-slate-500">/talker</span>
            </motion.div>

            {/* Code lines effect */}
            <div className="absolute right-10 top-10 space-y-2 opacity-30">
                <motion.div className="h-1 w-12 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 0 }} />
                <motion.div className="h-1 w-20 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 0.5 }} />
                <motion.div className="h-1 w-16 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 1 }} />
            </div>
        </div>
    );
};

const TopicAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center gap-12 bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            {/* Publisher */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Publisher</span>
                </div>
                <span className="text-xs text-slate-500">Node A</span>
            </div>

            {/* Topic Pipe */}
            <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-48 h-12 bg-slate-800/50 rounded-full border border-slate-700 flex items-center px-2">
                <span className="absolute -top-6 left-1/2 -translate-x-1/2 text-xs text-orange-400 font-mono">/topic_name</span>
            </div>

            {/* Message Packet */}
            <motion.div
                className="w-6 h-6 bg-orange-500 rounded shadow-[0_0_15px_rgba(249,115,22,0.6)] z-20 absolute left-[calc(50%-6rem)]"
                animate={{
                    x: [0, 190],
                    opacity: [0, 1, 1, 0],
                    scale: [0.5, 1, 1, 0.5]
                }}
                transition={{
                    duration: 2,
                    repeat: Infinity,
                    ease: "easeInOut"
                }}
            />

            {/* Subscriber */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Subscriber</span>
                </div>
                <span className="text-xs text-slate-500">Node B</span>
            </div>
        </div>
    );
};

const ServiceAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center gap-16 bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            {/* Client */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Client</span>
                </div>
                <span className="text-xs text-slate-500">Node A</span>
            </div>

            {/* Request */}
            <motion.div
                className="absolute top-[35%] left-[calc(50%-5rem)] text-[10px] text-blue-400 font-mono"
                animate={{ x: [0, 140], opacity: [1, 0] }}
                transition={{ duration: 1.5, repeat: Infinity, repeatDelay: 1.5 }}
            >
                REQ &gt;&gt;
            </motion.div>
            <motion.div
                className="w-3 h-3 bg-blue-500 rounded-full z-20 absolute top-[42%] left-[calc(50%-5rem)]"
                animate={{ x: [0, 160], opacity: [1, 0] }}
                transition={{ duration: 1.5, repeat: Infinity, repeatDelay: 1.5 }}
            />

            {/* Response */}
            <motion.div
                className="absolute bottom-[35%] right-[calc(50%-5rem)] text-[10px] text-green-400 font-mono"
                animate={{ x: [0, -140], opacity: [0, 1] }}
                transition={{ duration: 1.5, repeat: Infinity, delay: 1.5, repeatDelay: 1.5 }}
            >
                &lt;&lt; RES
            </motion.div>
            <motion.div
                className="w-3 h-3 bg-green-500 rounded-full z-20 absolute bottom-[42%] right-[calc(50%-5rem)]"
                animate={{ x: [0, -160], opacity: [0, 1] }}
                transition={{ duration: 1.5, repeat: Infinity, delay: 1.5, repeatDelay: 1.5 }}
            />

            {/* Server */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2 relative">
                    <span className="text-xs font-bold text-slate-300">Server</span>
                    <motion.div
                        className="absolute -top-2 -right-2 w-4 h-4 bg-yellow-500 rounded-full flex items-center justify-center text-[8px] text-black font-bold"
                        animate={{ scale: [0, 1, 0] }}
                        transition={{ duration: 3, repeat: Infinity, times: [0.4, 0.5, 0.9] }}
                    >
                        !
                    </motion.div>
                </div>
                <span className="text-xs text-slate-500">Node B</span>
            </div>
        </div>
    );
};
