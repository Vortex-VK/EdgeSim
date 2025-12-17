type Coordinate = { x: number; y: number };

interface ObjectRenderProps {
  pos: Coordinate;
  isSelected: boolean;
  objectId: string;
  onSelectId: (id: string) => void;
  onMouseDown: (objectId: string, coordIndex: number, e: React.MouseEvent<SVGElement>) => void;
}

interface PathObjectRenderProps extends Omit<ObjectRenderProps, 'pos'> {
  start: Coordinate;
  end: Coordinate;
  color?: string; // Optional color override
}

interface ForkliftOptions {
  reversing_mode?: boolean;
  alarm?: boolean;
  reflective?: boolean;
  load_overhang?: boolean;
}

// Robot: red disc, radius default 0.4 m (diameter 0.8 m)
export function RobotObject({ pos, isSelected, objectId, onSelectId, onMouseDown }: ObjectRenderProps) {
  const radius = 0.4;
  return (
    <g>
      <circle
        cx={pos.x}
        cy={pos.y}
        r={radius}
        fill="#ef4444"
        stroke={isSelected ? "#3b82f6" : "#b91c1c"}
        strokeWidth={0.04}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      <circle
        cx={pos.x}
        cy={pos.y}
        r={radius + 0.1}
        fill="transparent"
        stroke="transparent"
        strokeWidth={0.3}
        style={{ cursor: 'grab' }}
        onMouseDown={(e) => onMouseDown(objectId, 0, e)}
        className="hover:stroke-blue-400"
      />
    </g>
  );
}

// Humans: vertical capsules; top-down they're circles of radius ~0.25 m, green
export function HumanObject({ start, end, isSelected, objectId, onSelectId, onMouseDown, color = "#22c55e" }: PathObjectRenderProps) {
  const radius = 0.25;
  const strokeColor = color === "#ef4444" ? "#b91c1c" : "#16a34a"; // Darker shade for stroke
  
  // Calculate arrow angle and position
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const arrowSize = 0.15;
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      {/* Invisible wide path for easier hovering */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={0.4}
        strokeLinecap="round"
        style={{ cursor: 'grab' }}
        onClick={() => onSelectId(objectId)}
      />
      {/* Path line - dashed */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke={color}
        strokeWidth={0.04}
        opacity={0.4}
        strokeLinecap="round"
        strokeDasharray="0.2,0.2"
      />
      {/* Direction arrow at end */}
      <g transform={`translate(${end.x}, ${end.y}) rotate(${angle * 180 / Math.PI})`}>
        <path
          d={`M 0 0 L ${-arrowSize} ${-arrowSize/2} L ${-arrowSize} ${arrowSize/2} Z`}
          fill={color}
          opacity={0.7}
        />
      </g>
      {/* Human at start position */}
      <circle
        cx={start.x}
        cy={start.y}
        r={radius}
        fill={color}
        stroke={isSelected ? "#3b82f6" : strokeColor}
        strokeWidth={0.03}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Control points for start and end */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Forklift ~0.9 m wide, ~3.0 m long including forks; yellow/orange with gray forks/mast
export function ForkliftObject({ start, end, isSelected, objectId, onSelectId, onMouseDown, opts }: PathObjectRenderProps & { opts?: ForkliftOptions }) {
  const width = 0.9 * 1.33;
  const length = 3.0 * 1.33;
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const renderAngle = angle + (opts?.reversing_mode ? Math.PI : 0);
  const arrowSize = 0.15;
  const reversing = Boolean(opts?.reversing_mode);
  const reflective = Boolean(opts?.reflective);
  const overhang = Boolean(opts?.load_overhang);
  const alarm = Boolean(opts?.alarm || opts?.reversing_mode);
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      {/* Invisible wide path for easier hovering */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={0.4}
        strokeLinecap="round"
        style={{ cursor: 'grab' }}
        onClick={() => onSelectId(objectId)}
      />
      {/* Path line - dashed */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="#f59e0b"
        strokeWidth={0.04}
        opacity={0.4}
        strokeLinecap="round"
        strokeDasharray="0.2,0.2"
      />
      {/* Direction arrow at end */}
      <g transform={`translate(${end.x}, ${end.y}) rotate(${angle * 180 / Math.PI})`}>
        <path
          d={`M 0 0 L ${-arrowSize} ${-arrowSize/2} L ${-arrowSize} ${arrowSize/2} Z`}
          fill="#f59e0b"
          opacity={0.7}
        />
      </g>
      {/* Forklift at start */}
      <g transform={`translate(${start.x}, ${start.y}) rotate(${renderAngle * 180 / Math.PI})`}>
        {/* Main body - yellow/orange */}
        <rect
          x={0}
          y={-width / 2}
          width={length * 0.6}
          height={width}
          fill={reflective ? "#fcd34d" : "#f59e0b"}
          stroke={isSelected ? "#3b82f6" : "#d97706"}
          strokeWidth={0.05}
          rx={0.05}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {/* Mast - gray */}
        <rect
          x={length * 0.55}
          y={-width * 0.2}
          width={0.1}
          height={width * 0.4}
          fill="#6b7280"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {/* Forks - gray */}
        <rect
          x={length * 0.6}
          y={-width * 0.35}
          width={length * 0.4}
          height={0.08}
          fill="#6b7280"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        <rect
          x={length * 0.6}
          y={width * 0.27}
          width={length * 0.4}
          height={0.08}
          fill="#6b7280"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {overhang && (
          <rect
            x={length * 0.6}
            y={-width * 0.25}
            width={length * 0.45}
            height={width * 0.5}
            fill="#c084fc"
            fillOpacity={0.4}
            stroke="#9333ea"
            strokeWidth={0.03}
            strokeDasharray="0.15 0.08"
          />
        )}
        {/* Wheels */}
        <circle
          cx={0.2}
          cy={-width * 0.4}
          r={0.1}
          fill="#1f2937"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        <circle
          cx={0.2}
          cy={width * 0.4}
          r={0.1}
          fill="#1f2937"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {reflective && (
          <>
            <line x1={length * 0.1} y1={-width * 0.25} x2={length * 0.5} y2={-width * 0.25} stroke="#22d3ee" strokeWidth={0.05} />
            <line x1={length * 0.1} y1={width * 0.25} x2={length * 0.5} y2={width * 0.25} stroke="#22d3ee" strokeWidth={0.05} />
          </>
        )}
        {/* Mast / cab outline */}
        <rect
          x={length * 0.15}
          y={-width * 0.5}
          width={length * 0.35}
          height={width}
          stroke={reversing ? "#b91c1c" : "#111827"}
          strokeWidth={0.05}
          fill="transparent"
        />
        {alarm && (
          <circle
            cx={length * 0.18}
            cy={0}
            r={0.12}
            fill="#ef4444"
            stroke="#b91c1c"
            strokeWidth={0.04}
          />
        )}
      </g>
      {/* Control points */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Pallet jack ~0.64 m x 1.0 m red-orange. Tugger train ~1.0 m x 2.4 m blue
// Using cart as pallet jack
export function CartObject({ start, end, isSelected, objectId, onSelectId, onMouseDown }: PathObjectRenderProps) {
  const width = 0.64;
  const length = 1.0;
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const arrowSize = 0.15;
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      {/* Invisible wide path for easier hovering */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={0.4}
        strokeLinecap="round"
        style={{ cursor: 'grab' }}
        onClick={() => onSelectId(objectId)}
      />
      {/* Path line - dashed */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="#f97316"
        strokeWidth={0.04}
        opacity={0.4}
        strokeLinecap="round"
        strokeDasharray="0.2,0.2"
      />
      {/* Direction arrow at end */}
      <g transform={`translate(${end.x}, ${end.y}) rotate(${angle * 180 / Math.PI})`}>
        <path
          d={`M 0 0 L ${-arrowSize} ${-arrowSize/2} L ${-arrowSize} ${arrowSize/2} Z`}
          fill="#f97316"
          opacity={0.7}
        />
      </g>
      {/* Pallet jack at start - red-orange */}
      <g transform={`translate(${start.x}, ${start.y}) rotate(${angle * 180 / Math.PI})`}>
        <rect
          x={0}
          y={-width / 2}
          width={length}
          height={width}
          fill="#f97316"
          stroke={isSelected ? "#3b82f6" : "#ea580c"}
          strokeWidth={0.04}
          rx={0.03}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {/* Handle */}
        <rect
          x={-0.1}
          y={-width * 0.15}
          width={0.1}
          height={width * 0.3}
          fill="#ea580c"
          rx={0.02}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {/* Wheels */}
        <circle
          cx={length * 0.8}
          cy={-width * 0.35}
          r={0.06}
          fill="#1f2937"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        <circle
          cx={length * 0.8}
          cy={width * 0.35}
          r={0.06}
          fill="#1f2937"
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
      </g>
      {/* Control points */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Tugger: longer blue vehicle
export function TuggerObject({ start, end, isSelected, objectId, onSelectId, onMouseDown }: PathObjectRenderProps) {
  const width = 0.9;
  const length = 2.2;
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const arrowSize = 0.15;
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;

  return (
    <g className="group">
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={0.4}
        strokeLinecap="round"
        style={{ cursor: 'grab' }}
        onClick={() => onSelectId(objectId)}
      />
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="#3b82f6"
        strokeWidth={0.04}
        opacity={0.45}
        strokeLinecap="round"
        strokeDasharray="0.18,0.18"
      />
      <g transform={`translate(${end.x}, ${end.y}) rotate(${angle * 180 / Math.PI})`}>
        <path
          d={`M 0 0 L ${-arrowSize} ${-arrowSize/2} L ${-arrowSize} ${arrowSize/2} Z`}
          fill="#2563eb"
          opacity={0.8}
        />
      </g>
      <g transform={`translate(${start.x}, ${start.y}) rotate(${angle * 180 / Math.PI})`}>
        <rect
          x={0}
          y={-width / 2}
          width={length}
          height={width}
          fill="#3b82f6"
          stroke={isSelected ? "#3b82f6" : "#1d4ed8"}
          strokeWidth={0.05}
          rx={0.08}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        <rect
          x={length * 0.1}
          y={-width * 0.4}
          width={length * 0.25}
          height={width * 0.8}
          fill="#60a5fa"
        />
        <rect
          x={length * 0.45}
          y={-width * 0.3}
          width={length * 0.4}
          height={width * 0.6}
          fill="#60a5fa"
        />
        <circle cx={length * 0.2} cy={-width * 0.45} r={0.1} fill="#0f172a" />
        <circle cx={length * 0.2} cy={width * 0.45} r={0.1} fill="#0f172a" />
        <circle cx={length * 0.8} cy={-width * 0.45} r={0.1} fill="#0f172a" />
        <circle cx={length * 0.8} cy={width * 0.45} r={0.1} fill="#0f172a" />
      </g>
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <circle cx={midX} cy={midY} r={0.35} fill="transparent" />
      </g>
    </g>
  );
}

// Walls: perimeter walls dark gray thin lines (~0.1 m thick). Custom walls 0.3 m-thick dark gray
export function WallObject({ start, end, isSelected, objectId, onSelectId, onMouseDown }: PathObjectRenderProps) {
  const thickness = 0.3;
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const length = Math.sqrt((end.x - start.x) ** 2 + (end.y - start.y) ** 2);
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      <g transform={`translate(${start.x}, ${start.y}) rotate(${angle * 180 / Math.PI})`}>
        <rect
          x={0}
          y={-thickness / 2}
          width={length}
          height={thickness}
          fill="#4b5563"
          stroke={isSelected ? "#3b82f6" : "#374151"}
          strokeWidth={0.03}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
      </g>
      {/* Control points */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Racking: standard racks brown, high-bay racks light gray/blue and marked reflective. Depth ~0.9–1.1 m
export function RackObject({ start, end, isSelected, objectId, onSelectId, onMouseDown, high = false }: PathObjectRenderProps & { high?: boolean }) {
  const thickness = 3;
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const length = Math.sqrt((end.x - start.x) ** 2 + (end.y - start.y) ** 2);
  const fill = high ? "#cbd5e1" : "#92400e";
  const stroke = high ? "#94a3b8" : "#78350f";
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      {/* Invisible wide path for easier selection */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={thickness}
        strokeLinecap="round"
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      <g transform={`translate(${start.x}, ${start.y}) rotate(${angle * 180 / Math.PI})`}>
        <rect
          x={0}
          y={-thickness / 2}
          width={length}
          height={thickness}
          fill={fill}
          fillOpacity={0.8}
          stroke={isSelected ? "#3b82f6" : stroke}
          strokeWidth={0.04}
          onClick={() => onSelectId(objectId)}
          style={{ cursor: 'grab' }}
        />
        {[0.25, 0.5, 0.75].map((ratio, i) => (
          <line
            key={i}
            x1={0}
            y1={-thickness / 2 + thickness * ratio}
            x2={length}
            y2={-thickness / 2 + thickness * ratio}
            stroke={stroke}
            strokeWidth={0.03}
          />
        ))}
      </g>
      {/* Control points */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Static obstacles: colored blocks. Pallet-like obstacles brown; cart_block dark gray
export function ObstacleObject({ pos, isSelected, objectId, onSelectId, onMouseDown }: ObjectRenderProps) {
  const size = 0.6; // Pallet-like size
  
  return (
    <g>
      {/* Pallet box - brown */}
      <rect
        x={pos.x - size / 2}
        y={pos.y - size / 2}
        width={size}
        height={size}
        fill="#92400e"
        stroke={isSelected ? "#3b82f6" : "#78350f"}
        strokeWidth={0.04}
        rx={0.03}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Pallet planks */}
      <line
        x1={pos.x - size / 2}
        y1={pos.y - size / 6}
        x2={pos.x + size / 2}
        y2={pos.y - size / 6}
        stroke="#78350f"
        strokeWidth={0.02}
      />
      <line
        x1={pos.x - size / 2}
        y1={pos.y + size / 6}
        x2={pos.x + size / 2}
        y2={pos.y + size / 6}
        stroke="#78350f"
        strokeWidth={0.02}
      />
      <circle
        cx={pos.x}
        cy={pos.y}
        r={size / 2 + 0.1}
        fill="transparent"
        stroke="transparent"
        strokeWidth={0.3}
        style={{ cursor: 'grab' }}
        onMouseDown={(e) => onMouseDown(objectId, 0, e)}
        className="hover:stroke-blue-400"
      />
    </g>
  );
}

// Cart block: gray rectangle used as obstruction
export function CartBlockObject({ pos, isSelected, objectId, onSelectId, onMouseDown }: ObjectRenderProps) {
  const w = 0.8;
  const h = 0.6;
  return (
    <g>
      <rect
        x={pos.x - w / 2}
        y={pos.y - h / 2}
        width={w}
        height={h}
        fill="#6b7280"
        stroke={isSelected ? "#3b82f6" : "#4b5563"}
        strokeWidth={0.05}
        rx={0.04}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      <line x1={pos.x - w / 2 + 0.08} y1={pos.y} x2={pos.x + w / 2 - 0.08} y2={pos.y} stroke="#111827" strokeWidth={0.05} />
      <line x1={pos.x} y1={pos.y - h / 2 + 0.08} x2={pos.x} y2={pos.y + h / 2 - 0.08} stroke="#111827" strokeWidth={0.05} />
      <rect
        x={pos.x - w / 2}
        y={pos.y - h / 2}
        width={w}
        height={h}
        fill="transparent"
        stroke="transparent"
        strokeWidth={0.3}
        style={{ cursor: 'grab' }}
        onMouseDown={(e) => onMouseDown(objectId, 0, e)}
        className="hover:stroke-blue-400"
      />
    </g>
  );
}

// Traction patches: wet/cleaning_liquid light blue, oil dark gray. Transition zones are colored patches
export function WetPatchObject({ start, end, isSelected, objectId, onSelectId, onMouseDown }: PathObjectRenderProps) {
  const width = Math.abs(end.x - start.x);
  const height = Math.abs(end.y - start.y);
  const x = Math.min(start.x, end.x);
  const y = Math.min(start.y, end.y);
  
  // Calculate midpoint for middle drag handle
  const midX = (start.x + end.x) / 2;
  const midY = (start.y + end.y) / 2;
  const xSize = 0.12;
  
  return (
    <g className="group">
      {/* Invisible wide path for easier hovering */}
      <line
        x1={start.x}
        y1={start.y}
        x2={end.x}
        y2={end.y}
        stroke="transparent"
        strokeWidth={0.4}
        strokeLinecap="round"
        style={{ cursor: 'grab' }}
        onClick={() => onSelectId(objectId)}
      />
      {/* Wet patch - light blue translucent */}
      <rect
        x={x}
        y={y}
        width={width}
        height={height}
        fill="#7dd3fc"
        fillOpacity={0.5}
        stroke={isSelected ? "#3b82f6" : "#38bdf8"}
        strokeWidth={0.03}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Water effect */}
      {Array.from({ length: 3 }).map((_, i) => {
        const dx = x + (width * (i + 1)) / 4;
        const dy = y + height / 2;
        return (
          <circle
            key={i}
            cx={dx}
            cy={dy}
            r={0.08}
            fill="#38bdf8"
            fillOpacity={0.4}
          />
        );
      })}
      {/* Control points */}
      {[start, end].map((coord, i) => (
        <g key={i}>
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.15}
            fill="transparent"
            stroke={isSelected ? "#3b82f6" : "transparent"}
            strokeWidth={0.04}
            style={{ cursor: 'grab' }}
            className="hover:stroke-blue-400"
            pointerEvents="none"
          />
          {/* Larger invisible hit area */}
          <circle
            cx={coord.x}
            cy={coord.y}
            r={0.35}
            fill="transparent"
            style={{ cursor: 'grab' }}
            onMouseDown={(e) => onMouseDown(objectId, i, e)}
          />
        </g>
      ))}
      {/* Middle drag point - X shape */}
      <g
        onMouseDown={(e) => onMouseDown(objectId, 2, e)}
        style={{ cursor: 'move' }}
        className="group-hover:opacity-100"
        opacity={isSelected ? 1 : 0}
      >
        <line
          x1={midX - xSize}
          y1={midY - xSize}
          x2={midX + xSize}
          y2={midY + xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        <line
          x1={midX - xSize}
          y1={midY + xSize}
          x2={midX + xSize}
          y2={midY - xSize}
          stroke="#3b82f6"
          strokeWidth={0.04}
          strokeLinecap="round"
        />
        {/* Invisible larger hit area for easier hover */}
        <circle
          cx={midX}
          cy={midY}
          r={0.35}
          fill="transparent"
        />
      </g>
    </g>
  );
}

// Spill - using this as oil spill (dark gray overlay)
export function SpillObject({ pos, isSelected, objectId, onSelectId, onMouseDown }: ObjectRenderProps) {
  const size = 0.5;
  
  return (
    <g>
      {/* Irregular oil spill - dark gray */}
      <ellipse
        cx={pos.x}
        cy={pos.y}
        rx={size * 0.6}
        ry={size * 0.4}
        fill="#6b7280"
        fillOpacity={0.6}
        stroke={isSelected ? "#3b82f6" : "#4b5563"}
        strokeWidth={0.03}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Additional blob */}
      <ellipse
        cx={pos.x + size * 0.2}
        cy={pos.y - size * 0.15}
        rx={size * 0.3}
        ry={size * 0.25}
        fill="#6b7280"
        fillOpacity={0.6}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Droplet effects */}
      {[0, 120, 240].map((angle, i) => {
        const rad = (angle * Math.PI) / 180;
        const dx = pos.x + Math.cos(rad) * size * 0.25;
        const dy = pos.y + Math.sin(rad) * size * 0.25;
        return (
          <circle
            key={i}
            cx={dx}
            cy={dy}
            r={0.05}
            fill="#4b5563"
            fillOpacity={0.7}
          />
        );
      })}
      <circle
        cx={pos.x}
        cy={pos.y}
        r={size / 2 + 0.1}
        fill="transparent"
        stroke="transparent"
        strokeWidth={0.3}
        style={{ cursor: 'grab' }}
        onMouseDown={(e) => onMouseDown(objectId, 0, e)}
        className="hover:stroke-blue-400"
      />
    </g>
  );
}

// Falling object injector: landing spot square (0.25m half-extent = 0.5m side)
// When triggered spawns above and shatters into wet patch (2.4m x 1.2m)
export function FallingObjectInjector({ pos, isSelected, objectId, onSelectId, onMouseDown }: ObjectRenderProps) {
  const size = 0.5; // 0.25m half-extent = 0.5m total
  
  return (
    <g>
      {/* Landing zone square - purple */}
      <rect
        x={pos.x - size / 2}
        y={pos.y - size / 2}
        width={size}
        height={size}
        fill="#a855f7"
        fillOpacity={0.3}
        stroke={isSelected ? "#3b82f6" : "#9333ea"}
        strokeWidth={0.04}
        strokeDasharray="0.1 0.05"
        rx={0.02}
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Impact marker */}
      <circle
        cx={pos.x}
        cy={pos.y}
        r={0.08}
        fill="#a855f7"
        onClick={() => onSelectId(objectId)}
        style={{ cursor: 'grab' }}
      />
      {/* Drop indicator arrows */}
      <path
        d={`M ${pos.x} ${pos.y - 0.3} L ${pos.x - 0.05} ${pos.y - 0.15} L ${pos.x + 0.05} ${pos.y - 0.15} Z`}
        fill="#a855f7"
        fillOpacity={0.6}
      />
      <circle
        cx={pos.x}
        cy={pos.y}
        r={size / 2 + 0.1}
        fill="transparent"
        stroke="transparent"
        strokeWidth={0.3}
        style={{ cursor: 'grab' }}
        onMouseDown={(e) => onMouseDown(objectId, 0, e)}
        className="hover:stroke-blue-400"
      />
    </g>
  );
}
