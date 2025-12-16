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
        <circle
          key={i}
          cx={coord.x}
          cy={coord.y}
          r={0.15}
          fill="transparent"
          stroke={isSelected ? "#3b82f6" : "transparent"}
          strokeWidth={0.04}
          style={{ cursor: 'grab' }}
          onMouseDown={(e) => onMouseDown(objectId, i, e)}
          className="hover:stroke-blue-400"
        />
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
        {/* Invisible larger hit area */}
        <circle
          cx={midX}
          cy={midY}
          r={0.25}
          fill="transparent"
        />
      </g>
    </g>
  );
}
