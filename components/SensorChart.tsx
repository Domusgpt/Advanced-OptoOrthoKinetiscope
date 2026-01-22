import React from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid } from 'recharts';
import { CapturePoint } from '../types';

interface SensorChartProps {
  data: CapturePoint[];
}

const SensorChart: React.FC<SensorChartProps> = ({ data }) => {
  const chartData = data.map(pt => ({
    time: `${pt.relativeTime}ms`,
    x: pt.acceleration.x,
    y: pt.acceleration.y,
    z: pt.acceleration.z,
  }));

  return (
    <div className="w-full h-48 bg-slate-900/50 rounded-lg border border-slate-700 p-2">
      <h3 className="text-xs font-semibold text-slate-400 mb-2 uppercase tracking-wider">Telemetry Synchronization</h3>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={chartData}>
          <CartesianGrid strokeDasharray="3 3" stroke="#334155" />
          <XAxis dataKey="time" stroke="#64748b" fontSize={10} tickLine={false} />
          <YAxis stroke="#64748b" fontSize={10} tickLine={false} domain={[-20, 20]} hide />
          <Tooltip 
            contentStyle={{ backgroundColor: '#1e293b', border: 'none', borderRadius: '4px', color: '#f8fafc' }}
            itemStyle={{ fontSize: '12px' }}
          />
          <Line type="monotone" dataKey="x" stroke="#ef4444" strokeWidth={2} dot={{r: 4}} animationDuration={1000} />
          <Line type="monotone" dataKey="y" stroke="#22c55e" strokeWidth={2} dot={{r: 4}} animationDuration={1000} />
          <Line type="monotone" dataKey="z" stroke="#3b82f6" strokeWidth={2} dot={{r: 4}} animationDuration={1000} />
        </LineChart>
      </ResponsiveContainer>
      <div className="flex justify-center gap-4 mt-1">
        <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-red-500"></div><span className="text-[10px] text-slate-400">Acc X</span></div>
        <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-green-500"></div><span className="text-[10px] text-slate-400">Acc Y</span></div>
        <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-blue-500"></div><span className="text-[10px] text-slate-400">Acc Z</span></div>
      </div>
    </div>
  );
};

export default SensorChart;
