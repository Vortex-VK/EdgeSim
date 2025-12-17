import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "./components/ui/card";
import { Button } from "./components/ui/button";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "./components/ui/tabs";
import { Input } from "./components/ui/input";
import { Label } from "./components/ui/label";
import { Textarea } from "./components/ui/textarea";
import { ScrollArea } from "./components/ui/scroll-area";
import { Badge } from "./components/ui/badge";
import { Switch } from "./components/ui/switch";
import { Slider } from "./components/ui/slider";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "./components/ui/select";
import { Dialog, DialogContent, DialogDescription, DialogFooter, DialogHeader, DialogTitle, DialogTrigger } from "./components/ui/dialog";
import { Separator } from "./components/ui/separator";
import { Tooltip, TooltipContent, TooltipTrigger, TooltipProvider } from "./components/ui/tooltip";
import { MessageSquareText, Pencil, Play, Database, Trash2, Edit, Info, Settings } from "lucide-react";
import {
  RobotObject,
  HumanObject,
  ForkliftObject,
  CartObject,
  TuggerObject,
  WallObject,
  RackObject,
  ObstacleObject,
  WetPatchObject,
  SpillObject,
  FallingObjectInjector,
  CartBlockObject
} from "./components/WarehouseObjects";

// Types
type Coordinate = { x: number; y: number };

type ForkliftOptions = {
  reversing_mode: boolean;
  alarm: boolean;
  reflective: boolean;
  load_overhang: boolean;
};

type RackOptions = {
  high_rack: boolean;
};

type ObjectType = 
  | "robot"
  | "human" 
  | "forklift"
  | "cart"
  | "tugger"
  | "wall"
  | "rack"
  | "obstacle"
  | "cart_block"
  | "wet_patch"
  | "spill"
  | "falling_object";

type SceneObject = {
  id: string;
  type: ObjectType;
  coords: Coordinate[];
  label?: string;
  options?: {
    forklift?: ForkliftOptions;
    rack?: RackOptions;
  };
};

type AppMode = "select" | "text" | "visual";
type SimulationMode = "test" | "batch";

type SimulationRequest = {
  mode: SimulationMode;
  command: string;
  config: Record<string, unknown>;
};

type SimulationResult = {
  ok: boolean;
  jobId?: string;
  message?: string;
  error?: string;
};

const EXAMPLE_PROMPTS = [
  "Aisle from 5,5 to 15,5 with human crossing from 10,0 to 10,10",
  "Narrow aisle from 8,0 to 8,20 with forklift from 8,15 to 8,5",
  "Wet patch from 10,10 to 12,12 with worker from 9,8 to 13,12",
  "Wall from 0,10 to 5,10 and rack from 6,8 to 8,12 with reversing forklift from 3,15 to 3,5",
  "Main aisle from 5,0 to 5,20 and crosswalk from 0,10 to 20,10 with group of workers"
];

const OBJECT_CATEGORIES = {
  dynamic: [
    { type: "human" as ObjectType, label: "Human", color: "#22c55e", coords: 2 }, // Green circle
    { type: "forklift" as ObjectType, label: "Forklift", color: "#f59e0b", coords: 2 }, // Yellow/orange
    { type: "cart" as ObjectType, label: "Pallet Jack", color: "#f97316", coords: 2 }, // Red-orange
    { type: "tugger" as ObjectType, label: "Tugger", color: "#3b82f6", coords: 2 } // Blue
  ],
  static: [
    { type: "wall" as ObjectType, label: "Wall", color: "#4b5563", coords: 2 }, // Dark gray
    { type: "rack" as ObjectType, label: "Rack", color: "#92400e", coords: 2 }, // Brown
    { type: "obstacle" as ObjectType, label: "Pallet/Box", color: "#92400e", coords: 1 }, // Brown
    { type: "cart_block" as ObjectType, label: "Cart Block", color: "#6b7280", coords: 1 } // Gray
  ],
  hazards: [
    { type: "wet_patch" as ObjectType, label: "Wet Patch", color: "#7dd3fc", coords: 2 }, // Light blue
    { type: "spill" as ObjectType, label: "Oil Spill", color: "#6b7280", coords: 1 } // Dark gray
  ],
  injectors: [
    { type: "falling_object" as ObjectType, label: "Falling Object", color: "#a855f7", coords: 1 } // Purple - visualized as landing spot
  ],
  robot: { type: "robot" as ObjectType, label: "AMR Robot", color: "#ef4444", coords: 2 } // Red - always present
};

function roundToHalf(n: number): number {
  return Math.round(n * 2) / 2;
}

async function triggerSimulation(req: SimulationRequest): Promise<SimulationResult> {
  try {
    const response = await fetch("/api/simulate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(req),
    });
    const data = (await response.json().catch(() => ({}))) as { success?: boolean; job_id?: string; message?: string; error?: string };
    if (!response.ok || !data?.success) {
      return { ok: false, error: data?.error || `Request failed (${response.status})` };
    }
    return { ok: true, jobId: data.job_id, message: data.message };
  } catch (err) {
    return { ok: false, error: err instanceof Error ? err.message : String(err) };
  }
}

// Info tooltip component
function InfoTooltip({ text }: { text: string }) {
  return (
    <Tooltip>
      <TooltipTrigger
        asChild={false}
        tabIndex={-1}
        className="inline-flex items-center ml-1.5 bg-transparent border-0 p-0 cursor-default"
        onMouseDown={(e) => { e.preventDefault(); e.stopPropagation(); }}
        onClick={(e) => { e.preventDefault(); e.stopPropagation(); }}
        onPointerDown={(e) => { e.preventDefault(); e.stopPropagation(); }}
      >
        <Info className="size-3.5 text-slate-400 hover:text-slate-600" />
      </TooltipTrigger>
      <TooltipContent side="right">
        <p>{text}</p>
      </TooltipContent>
    </Tooltip>
  );
}

export default function App() {
  const [mode, setMode] = useState<AppMode>("select");
  
  return (
    <TooltipProvider delayDuration={150} skipDelayDuration={100}>
      <div className="min-h-screen h-screen bg-gradient-to-br from-slate-50 to-slate-100">
        {mode === "select" && <ModeSelection onSelectMode={setMode} />}
        {mode === "text" && <TextMode onBack={() => setMode("select")} />}
        {mode === "visual" && <VisualMode onBack={() => setMode("select")} />}
      </div>
    </TooltipProvider>
  );
}

function ModeSelection({ onSelectMode }: { onSelectMode: (mode: AppMode) => void }) {
  const [hoveredMode, setHoveredMode] = useState<"text" | "visual" | null>(null);

  return (
    <div className="min-h-screen h-screen flex flex-col bg-white overflow-hidden relative" style={{minHeight: '100vh'}}>
      {/* EdgeSim Title - Centered at Top with higher z-index */}
      <div className={`absolute top-8 left-1/2 -translate-x-1/2 z-30 text-center transition-all duration-500 ${
        hoveredMode ? "scale-90 opacity-70 -translate-y-8" : "scale-100 opacity-100 translate-y-0"
      }`}>
        <h1 className="text-7xl font-bold text-white mb-2 drop-shadow-lg">
          EdgeSim
        </h1>
        <p className="text-white text-lg drop-shadow">
          Warehouse robotics simulation and data generation
        </p>
      </div>

      <div className="flex-1 flex relative">
        {/* Text Mode - Left Half */}
        <div
          className={`relative flex items-center justify-center cursor-pointer transition-all duration-500 z-10 overflow-hidden ${
            hoveredMode === "text" ? "flex-[1.05]" : hoveredMode === "visual" ? "flex-[0.95]" : "flex-1"
          }`}
          onMouseEnter={() => setHoveredMode("text")}
          onMouseLeave={() => setHoveredMode(null)}
          onClick={() => onSelectMode("text")}
        >
          {/* Background Image for Text Mode */}
          <div 
            className="absolute inset-0 bg-cover bg-center opacity-90"
            style={{
              backgroundImage: `url('https://images.unsplash.com/photo-1484480974693-6ca0a78fb36b?crop=entropy&cs=tinysrgb&fit=max&fm=jpg&ixid=M3w3Nzg4Nzd8MHwxfHNlYXJjaHwxfHx3YXJlaG91c2UlMjBkYXJrJTIwaW5kdXN0cmlhbHxlbnwwfHx8fDE3MzQyOTE4MTZ8MA&ixlib=rb-4.1.0&q=80&w=1080')`
            }}
          />
          {/* Dark overlay */}
          <div className={`absolute inset-0 bg-black transition-opacity duration-500 ${
            hoveredMode === "visual" ? "opacity-70" : "opacity-40"
          }`} />
          
          <div className={`text-center transition-all duration-300 relative z-20 ${
            hoveredMode === "text" ? "scale-110" : "scale-100"
          } ${hoveredMode === "visual" ? "opacity-40" : "opacity-100"}`}>
            <div className="p-6 bg-blue-50 rounded-2xl inline-block mb-6">
              <MessageSquareText className="size-16 text-blue-600" />
            </div>
            <h2 className={`font-semibold text-white mb-3 transition-all duration-300 drop-shadow-lg ${
              hoveredMode === "text" ? "text-4xl" : "text-3xl"
            }`}>
              Text Mode
            </h2>
            <p className="text-white text-lg max-w-sm mx-auto drop-shadow">
              Describe scenarios using natural language prompts
            </p>
          </div>
        </div>

        {/* Visual Mode - Right Half */}
        <div
          className={`relative flex items-center justify-center cursor-pointer transition-all duration-500 z-10 overflow-hidden ${
            hoveredMode === "visual" ? "flex-[1.05]" : hoveredMode === "text" ? "flex-[0.95]" : "flex-1"
          }`}
          onMouseEnter={() => setHoveredMode("visual")}
          onMouseLeave={() => setHoveredMode(null)}
          onClick={() => onSelectMode("visual")}
        >
          {/* Background Image for Visual Mode */}
          <div 
            className="absolute inset-0 bg-cover bg-center opacity-90"
            style={{
              backgroundImage: `url('https://images.unsplash.com/photo-1721244654346-9be0c0129e36?crop=entropy&cs=tinysrgb&fit=max&fm=jpg&ixid=M3w3Nzg4Nzd8MHwxfHNlYXJjaHwxfHx3YXJlaG91c2UlMjBmbG9vciUyMHBsYW4lMjBibHVlcHJpbnR8ZW58MXx8fHwxNzY1ODE1OTQzfDA&ixlib=rb-4.1.0&q=80&w=1080')`
            }}
          />
          {/* Dark overlay */}
          <div className={`absolute inset-0 bg-black transition-opacity duration-500 ${
            hoveredMode === "text" ? "opacity-70" : "opacity-40"
          }`} />

          <div className={`text-center transition-all duration-300 relative z-20 ${
            hoveredMode === "visual" ? "scale-110" : "scale-100"
          } ${hoveredMode === "text" ? "opacity-40" : "opacity-100"}`}>
            <div className="p-6 bg-blue-50 rounded-2xl inline-block mb-6">
              <Pencil className="size-16 text-blue-600" />
            </div>
            <h2 className={`font-semibold text-white mb-3 transition-all duration-300 drop-shadow-lg ${
              hoveredMode === "visual" ? "text-4xl" : "text-3xl"
            }`}>
              Visual Mode
            </h2>
            <p className="text-white text-lg max-w-sm mx-auto drop-shadow">
              Build scenarios with precise control over object placement
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}

function TextMode({ onBack }: { onBack: () => void }) {
  const [prompt, setPrompt] = useState("");
  const [seed, setSeed] = useState(() => Math.floor(Math.random() * 1000000));
  const [showTestDialog, setShowTestDialog] = useState(false);
  const [showBatchDialog, setShowBatchDialog] = useState(false);

  // Configuration options
  const [siteProfile, setSiteProfile] = useState("default");
  const [batchProfile, setBatchProfile] = useState("minimal");
  const [lidarLogging, setLidarLogging] = useState("off");
  const [timestep, setTimestep] = useState(0.05);
  const [debugVisuals, setDebugVisuals] = useState(false);
  const [timeBudget, setTimeBudget] = useState(120);
  const [autoDegrade, setAutoDegrade] = useState(false);

  // Test run config
  const [slowmo, setSlowmo] = useState(1.0);

  // Batch run config
  const [numRuns, setNumRuns] = useState(10000);
  
  // Sensor-only injectors (not used in text mode, but kept for consistency)
  const [lidarBlackout, setLidarBlackout] = useState(false);
  const [ghostObstacle, setGhostObstacle] = useState(false);

  const handleTestRun = async () => {
    let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
    if (siteProfile !== "default") command += ` --site ${siteProfile}`;
    if (debugVisuals) command += ` --debug-visuals`;
    if (lidarLogging === "full") command += ` --lidar-logging`;
    if (lidarLogging === "events") command += ` --lidar-events-only`;
    console.log("Executing test run:", command);

    const result = await triggerSimulation({
      mode: "test",
      command,
      config: {
        scenario_type: "text",
        text_prompt: prompt,
        seed,
        timestep,
        slowmo,
        site_profile: siteProfile,
        debug_visuals: debugVisuals,
        lidar_logging: lidarLogging,
        lidar_blackout: lidarBlackout,
        ghost_obstacle: ghostObstacle,
        gui: true,
        realtime: true,
      },
    });

    if (result.ok) {
      console.log("Simulation started:", result.jobId || "");
    } else {
      console.error("Simulation failed:", result.error);
      alert(`Failed to start simulation: ${result.error}`);
    }
    setShowTestDialog(false);
  };

  const handleBatchRun = async () => {
    let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
    if (siteProfile !== "default") command += ` --site ${siteProfile}`;
    if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
    if (autoDegrade) command += ` --auto-degrade`;
    if (lidarLogging === "full") command += ` --lidar-logging`;
    if (lidarLogging === "events") command += ` --lidar-events-only`;
    console.log("Executing batch run:", command);

    const result = await triggerSimulation({
      mode: "batch",
      command,
      config: {
        scenario_type: "text",
        text_prompt: prompt,
        num_runs: numRuns,
        seed,
        batch_profile: batchProfile,
        site_profile: siteProfile,
        time_budget: timeBudget,
        auto_degrade: autoDegrade,
        lidar_logging: lidarLogging,
        lidar_blackout: lidarBlackout,
        ghost_obstacle: ghostObstacle,
        gui: false,
        headless: true,
      },
    });

    if (result.ok) {
      console.log("Batch simulation started:", result.jobId || "");
    } else {
      console.error("Batch simulation failed:", result.error);
      alert(`Failed to start batch simulation: ${result.error}`);
    }
    setShowBatchDialog(false);
  };

  return (
    <div className="size-full flex flex-col">
      <div className="bg-white border-b p-4 flex items-center justify-between">
        <div className="flex items-center gap-3">
          <Button variant="ghost" onClick={onBack}>← Back</Button>
          <Separator orientation="vertical" className="h-6" />
          <h2 className="font-semibold">Text Mode</h2>
        </div>
        <div className="flex gap-2">
          <Dialog open={showTestDialog} onOpenChange={setShowTestDialog}>
            <DialogTrigger asChild>
              <Button variant="secondary" disabled={!prompt}>
                <Play className="size-4 mr-2" />
                Test Run
              </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl">
              <DialogHeader>
                <DialogTitle>Test Run Configuration</DialogTitle>
                <DialogDescription>
                  Run a single simulation with GUI visualization
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="slowmo">Slow Motion: {slowmo}x</Label>
                    <InfoTooltip text="Playback speed for realtime/GUI runs." />
                  </div>
                  <Slider
                    id="slowmo"
                    min={0.1}
                    max={5}
                    step={0.1}
                    value={[slowmo]}
                    onValueChange={([v]) => setSlowmo(v)}
                  />
                </div>
                <div className="flex items-center justify-between">
                  <Label htmlFor="testDebugVisuals">Debug Visuals</Label>
                  <Switch id="testDebugVisuals" checked={debugVisuals} onCheckedChange={setDebugVisuals} />
                </div>
                <div className="bg-slate-50 p-3 rounded-lg">
                  <p className="text-xs font-mono text-slate-600 break-all">
                    edgesim run-one "{prompt}" --seed {seed} --gui --realtime --dt {timestep} --slowmo {slowmo}
                    {siteProfile !== "default" && ` --site ${siteProfile}`}
                    {debugVisuals && ` --debug-visuals`}
                    {lidarLogging === "full" && ` --lidar-logging`}
                    {lidarLogging === "events" && ` --lidar-events-only`}
                  </p>
                </div>
              </div>
              <DialogFooter>
                <Button variant="outline" onClick={() => setShowTestDialog(false)}>Cancel</Button>
                <Button onClick={handleTestRun}>Run Test</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>

          <Dialog open={showBatchDialog} onOpenChange={setShowBatchDialog}>
            <DialogTrigger asChild>
              <Button disabled={!prompt}>
                <Database className="size-4 mr-2" />
                Create Data
              </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl" onOpenAutoFocus={(e) => e.preventDefault()}>
              <DialogHeader>
                <DialogTitle>Batch Run Configuration</DialogTitle>
                <DialogDescription>
                  Generate multiple simulation runs for data collection
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="numRuns">Number of Runs</Label>
                    <InfoTooltip text="Number of runs to generate with different seeds." />
                  </div>
                  <Input
                    id="numRuns"
                    type="number"
                    min={1}
                    max={10000}
                    value={numRuns}
                    onChange={(e) => setNumRuns(parseInt(e.target.value) || 1)}
                    autoFocus={false}
                  />
                </div>
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="batchSeed">Base Seed</Label>
                    <InfoTooltip text="Controls randomness. Reuse to reproduce results; change to vary the scenario." />
                  </div>
                  <Input
                    id="batchSeed"
                    type="number"
                    value={seed}
                    onChange={(e) => setSeed(parseInt(e.target.value) || 0)}
                  />
                </div>
                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="timeBudget">Time Budget (seconds)</Label>
                      <InfoTooltip text="Target wall-clock limit for a batch; runs aim to complete within this window." />
                    </div>
                    <Input
                      id="timeBudget"
                      type="number"
                      min={0}
                      value={timeBudget}
                      onChange={(e) => setTimeBudget(parseInt(e.target.value) || 120)}
                    />
                  </div>
                  <div className="flex items-center justify-between pt-7">
                    <div className="flex items-center">
                      <Label htmlFor="autoDegrade">Auto Degrade</Label>
                      <InfoTooltip text="If the budget is tight, shortens runs (and may lower LiDAR rate) to stay on time." />
                    </div>
                    <Switch id="autoDegrade" checked={autoDegrade} onCheckedChange={setAutoDegrade} />
                  </div>
                </div>
                <div className="bg-slate-50 p-3 rounded-lg">
                  <p className="text-xs font-mono text-slate-600 break-all">
                    edgesim run-batch "{prompt}" --runs {numRuns} --seed {seed} --profile {batchProfile}
                    {siteProfile !== "default" && ` --site ${siteProfile}`}
                    {timeBudget && ` --time-budget-min ${timeBudget}`}
                    {autoDegrade && ` --auto-degrade`}
                    {lidarLogging === "full" && ` --lidar-logging`}
                    {lidarLogging === "events" && ` --lidar-events-only`}
                  </p>
                </div>
              </div>
              <DialogFooter>
                <Button variant="outline" onClick={() => setShowBatchDialog(false)}>Cancel</Button>
                <Button onClick={handleBatchRun}>Start Batch</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      <div className="flex-1 overflow-auto p-6">
        <div className="max-w-6xl mx-auto space-y-6">
          <div className="space-y-4">
            <div className="space-y-2">
              <Label htmlFor="prompt" className="text-lg">Scenario Description</Label>
              <Textarea
                id="prompt"
                placeholder="Describe your warehouse scenario"
                value={prompt}
                onChange={(e) => setPrompt(e.target.value)}
                rows={10}
                className="text-base"
              />
            </div>

            <div className="space-y-2">
              <Label className="text-sm text-slate-400">Example Scenarios</Label>
              <div className="flex flex-wrap gap-2">
                {EXAMPLE_PROMPTS.map((ex, i) => (
                  <Badge
                    key={i}
                    variant="outline"
                    className="cursor-pointer hover:bg-blue-50 text-xs text-slate-400 hover:text-slate-700 border-slate-200"
                    onClick={() => setPrompt(ex)}
                  >
                    {ex}
                  </Badge>
                ))}
              </div>
            </div>
          </div>

          <Card>
            <CardHeader>
              <CardTitle>Configuration</CardTitle>
            </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="seed">Random Seed</Label>
                      <InfoTooltip text="Controls randomness. Reuse to reproduce results; change to vary the scenario." />
                    </div>
                    <Input
                      id="seed"
                      type="number"
                      value={seed}
                      onChange={(e) => setSeed(parseInt(e.target.value) || 0)}
                    />
                  </div>
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="timestep">Timestep (seconds)</Label>
                      <InfoTooltip text="Physics update interval. Smaller = smoother/more accurate, slower; larger = faster, coarser." />
                    </div>
                    <Input
                      id="timestep"
                      type="number"
                      step={0.01}
                      min={0.01}
                      max={0.2}
                      value={timestep}
                      onChange={(e) => setTimestep(parseFloat(e.target.value) || 0.05)}
                    />
                  </div>
                </div>

                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="siteProfile">Site Profile</Label>
                      <InfoTooltip text="Applies a chosen set of sensor and floor realism settings for a specific site." />
                    </div>
                    <Select value={siteProfile} onValueChange={setSiteProfile}>
                      <SelectTrigger id="siteProfile">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="default">Default</SelectItem>
                        <SelectItem value="warehouse_a">Warehouse A</SelectItem>
                        <SelectItem value="warehouse_b">Warehouse B</SelectItem>
                        <SelectItem value="distribution_center">Distribution Center</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="batchProfile">Batch Profile</Label>
                      <InfoTooltip text="Selects a preset detail/randomness level for batches (e.g., minimal vs full)." />
                    </div>
                    <Select value={batchProfile} onValueChange={setBatchProfile}>
                      <SelectTrigger id="batchProfile">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="minimal">Minimal</SelectItem>
                        <SelectItem value="robot">Robot</SelectItem>
                        <SelectItem value="full">Full</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                </div>

                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="lidarLogging">LiDAR Logging Mode</Label>
                    <InfoTooltip text="Choose what LiDAR data to save: none, full logs, or event-only snippets." />
                  </div>
                  <Select value={lidarLogging} onValueChange={setLidarLogging}>
                    <SelectTrigger id="lidarLogging">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="off">Off</SelectItem>
                      <SelectItem value="full">Full Logging</SelectItem>
                      <SelectItem value="events">Event Windows Only</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}

function VisualMode({ onBack }: { onBack: () => void }) {
  // Initialize with AMR Robot at default coordinates
  const [objects, setObjects] = useState<SceneObject[]>([{
    id: 'main-aisle',
    type: 'robot',
    coords: [{ x: 2.0, y: 10.0 }, { x: 18.0, y: 10.0 }]
  }]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [seed, setSeed] = useState(() => Math.floor(Math.random() * 1000000));
  const [showTestDialog, setShowTestDialog] = useState(false);
  const [showBatchDialog, setShowBatchDialog] = useState(false);
  const [showSettingsDialog, setShowSettingsDialog] = useState(false);
  const [numRuns, setNumRuns] = useState(10000);
  const [dragState, setDragState] = useState<{
    objectId: string;
    coordIndex: number;
    startX: number;
    startY: number;
  } | null>(null);

  // Configuration settings (matching Text Mode)
  const [siteProfile, setSiteProfile] = useState("default");
  const [batchProfile, setBatchProfile] = useState("minimal");
  const [lidarLogging, setLidarLogging] = useState("off");
  const [timestep, setTimestep] = useState(0.05);
  const [timeBudget, setTimeBudget] = useState(120);
  const [autoDegrade, setAutoDegrade] = useState(false);

  // Test run config
  const [slowmo, setSlowmo] = useState(1.0);
  const [debugVisuals, setDebugVisuals] = useState(false);
  
  // Sensor-only injectors (batch/test run options)
  const [lidarBlackout, setLidarBlackout] = useState(false);
  const [ghostObstacle, setGhostObstacle] = useState(false);

  const defaultOptionsForType = (type: ObjectType) => {
    if (type === "forklift") {
      return { forklift: { reversing_mode: false, alarm: false, reflective: false, load_overhang: false } as ForkliftOptions };
    }
    if (type === "rack") {
      return { rack: { high_rack: false } as RackOptions };
    }
    return undefined;
  };

  const addObject = (type: ObjectType, coordCount: number) => {
    const newObj: SceneObject = {
      id: Date.now().toString(),
      type,
      coords: coordCount === 1 
        ? [{ x: 10, y: 10 }]
        : [{ x: 8, y: 10 }, { x: 12, y: 10 }],
      options: defaultOptionsForType(type)
    };
    setObjects([...objects, newObj]);
    setSelectedId(newObj.id);
  };

  const updateCoord = (id: string, index: number, coord: Coordinate) => {
    setObjects(objects.map(obj =>
      obj.id === id
        ? { ...obj, coords: obj.coords.map((c, i) => i === index ? coord : c) }
        : obj
    ));
  };

  const deleteObject = (id: string) => {
    // Prevent deleting the main aisle (robot)
    if (id === 'main-aisle') return;
    
    setObjects(objects.filter(obj => obj.id !== id));
    if (selectedId === id) setSelectedId(null);
  };

  const generatePrompt = (): string => {
    const parts: string[] = [];
    
    // Find robot and add it first as "main aisle"
    const robot = objects.find(obj => obj.type === 'robot');
    if (robot && robot.coords.length === 2) {
      parts.push(`main aisle from ${robot.coords[0].x},${robot.coords[0].y} to ${robot.coords[1].x},${robot.coords[1].y}`);
    }
    
    // Add other objects
    objects.forEach(obj => {
      if (obj.type === 'robot') return; // Skip robot, already added
      
      const c = obj.coords;
      switch (obj.type) {
        case "human":
          parts.push(`human crossing from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "forklift":
          const fk = obj.options?.forklift || { reversing_mode: false, alarm: false, reflective: false, load_overhang: false };
          const base = fk.reversing_mode ? "reversing forklift" : "forklift";
          const extras: string[] = [];
          if (fk.alarm) extras.push("with alarm");
          if (fk.reflective) extras.push("with reflective load");
          if (fk.load_overhang) extras.push("with load overhang");
          parts.push(`${base} moving from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}${extras.length ? " " + extras.join(" ") : ""}`);
          break;
        case "cart":
          parts.push(`cart from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "tugger":
          parts.push(`tugger from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "wall":
          parts.push(`wall from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "rack":
          parts.push(`${obj.options?.rack?.high_rack ? "high rack" : "rack"} from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "obstacle":
          parts.push(`obstacle at ${c[0].x},${c[0].y}`);
          break;
        case "cart_block":
          parts.push(`cart block at ${c[0].x},${c[0].y}`);
          break;
        case "wet_patch":
          parts.push(`wet patch from ${c[0].x},${c[0].y} to ${c[1].x},${c[1].y}`);
          break;
        case "spill":
          parts.push(`spill at ${c[0].x},${c[0].y}`);
          break;
        case "falling_object":
          parts.push(`falling object at ${c[0].x},${c[0].y}`);
          break;
      }
    });
    return parts.join(", ");
  };

  const handleMouseDown = (objectId: string, coordIndex: number, e: React.MouseEvent<SVGElement>) => {
    e.stopPropagation();
    const svg = e.currentTarget.ownerSVGElement;
    if (!svg) return;
    
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const cursorPt = pt.matrixTransform(svg.getScreenCTM()?.inverse());
    
    setDragState({
      objectId,
      coordIndex,
      startX: cursorPt.x,
      startY: cursorPt.y
    });
    setSelectedId(objectId);
  };

  const handleMouseMove = (e: React.MouseEvent<SVGSVGElement>) => {
    if (!dragState) return;
    
    const svg = e.currentTarget;
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const cursorPt = pt.matrixTransform(svg.getScreenCTM()?.inverse());
    
    const newX = roundToHalf(Math.max(0, Math.min(20, cursorPt.x)));
    const newY = roundToHalf(Math.max(0, Math.min(20, cursorPt.y)));
    
    // Handle middle dragging (coordIndex 2) - move both start and end together
    if (dragState.coordIndex === 2) {
      const deltaX = newX - dragState.startX;
      const deltaY = newY - dragState.startY;
      
      setObjects(objects.map(obj => {
        if (obj.id === dragState.objectId && obj.coords.length === 2) {
          const newStart = {
            x: roundToHalf(Math.max(0, Math.min(20, obj.coords[0].x + deltaX))),
            y: roundToHalf(Math.max(0, Math.min(20, obj.coords[0].y + deltaY)))
          };
          const newEnd = {
            x: roundToHalf(Math.max(0, Math.min(20, obj.coords[1].x + deltaX))),
            y: roundToHalf(Math.max(0, Math.min(20, obj.coords[1].y + deltaY)))
          };
          return { ...obj, coords: [newStart, newEnd] };
        }
        return obj;
      }));
      
      setDragState({
        ...dragState,
        startX: newX,
        startY: newY
      });
    } else {
      // Normal dragging of individual coordinate
      updateCoord(dragState.objectId, dragState.coordIndex, { x: newX, y: newY });
    }
  };

  const handleMouseUp = () => {
    setDragState(null);
  };

  const handleTestRun = async () => {
    const prompt = generatePrompt();
    let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
    if (siteProfile !== "default") command += ` --site ${siteProfile}`;
    if (debugVisuals) command += ` --debug-visuals`;
    if (lidarLogging === "full") command += ` --lidar-logging`;
    if (lidarLogging === "events") command += ` --lidar-events-only`;
    console.log("Executing test run:", command);

    const result = await triggerSimulation({
      mode: "test",
      command,
      config: {
        scenario_type: "visual",
        visual_layout: objects,
        text_prompt: prompt,
        seed,
        timestep,
        slowmo,
        site_profile: siteProfile,
        debug_visuals: debugVisuals,
        lidar_logging: lidarLogging,
        lidar_blackout: lidarBlackout,
        ghost_obstacle: ghostObstacle,
        gui: true,
        realtime: true,
      },
    });

    if (result.ok) {
      console.log("Simulation started:", result.jobId || "");
    } else {
      console.error("Simulation failed:", result.error);
      alert(`Failed to start simulation: ${result.error}`);
    }
    setShowTestDialog(false);
  };

  const handleBatchRun = async () => {
    const prompt = generatePrompt();
    let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
    if (siteProfile !== "default") command += ` --site ${siteProfile}`;
    if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
    if (autoDegrade) command += ` --auto-degrade`;
    if (lidarLogging === "full") command += ` --lidar-logging`;
    if (lidarLogging === "events") command += ` --lidar-events-only`;
    console.log("Executing batch run:", command);

    const result = await triggerSimulation({
      mode: "batch",
      command,
      config: {
        scenario_type: "visual",
        visual_layout: objects,
        text_prompt: prompt,
        num_runs: numRuns,
        seed,
        batch_profile: batchProfile,
        site_profile: siteProfile,
        time_budget: timeBudget,
        auto_degrade: autoDegrade,
        lidar_logging: lidarLogging,
        lidar_blackout: lidarBlackout,
        ghost_obstacle: ghostObstacle,
        gui: false,
        headless: true,
      },
    });

    if (result.ok) {
      console.log("Batch simulation started:", result.jobId || "");
    } else {
      console.error("Batch simulation failed:", result.error);
      alert(`Failed to start batch simulation: ${result.error}`);
    }
    setShowBatchDialog(false);
  };

  const selectedObj = objects.find(obj => obj.id === selectedId);

  return (
    <div className="size-full flex flex-col">
      <div className="bg-white border-b p-4 flex items-center justify-between">
        <div className="flex items-center gap-3">
          <Button variant="ghost" onClick={onBack}>← Back</Button>
          <Separator orientation="vertical" className="h-6" />
          <h2 className="font-semibold">Visual Mode</h2>
        </div>
        <div className="flex gap-2">
          <Dialog open={showSettingsDialog} onOpenChange={setShowSettingsDialog}>
            <DialogTrigger asChild>
              <Button variant="outline">
                <Settings className="size-4 mr-2" />
                Configuration
              </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl" onOpenAutoFocus={(e) => e.preventDefault()}>
              <DialogHeader>
                <DialogTitle>Configuration</DialogTitle>
                <DialogDescription>
                  Configure simulation parameters and logging options
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="visualSeed">Random Seed</Label>
                      <InfoTooltip text="Controls randomness. Reuse to reproduce results; change to vary the scenario." />
                    </div>
                    <Input
                      id="visualSeed"
                      type="number"
                      value={seed}
                      onChange={(e) => setSeed(parseInt(e.target.value) || 0)}
                      autoFocus={false}
                    />
                  </div>
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="visualTimestep">Timestep (seconds)</Label>
                      <InfoTooltip text="Physics update interval. Smaller = smoother/more accurate, slower; larger = faster, coarser." />
                    </div>
                    <Input
                      id="visualTimestep"
                      type="number"
                      step={0.01}
                      min={0.01}
                      max={0.2}
                      value={timestep}
                      onChange={(e) => setTimestep(parseFloat(e.target.value) || 0.05)}
                    />
                  </div>
                </div>
                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="visualSiteProfile">Site Profile</Label>
                      <InfoTooltip text="Applies a chosen set of sensor and floor realism settings for a specific site." />
                    </div>
                    <Select value={siteProfile} onValueChange={setSiteProfile}>
                      <SelectTrigger id="visualSiteProfile">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="default">Default</SelectItem>
                        <SelectItem value="warehouse_a">Warehouse A</SelectItem>
                        <SelectItem value="warehouse_b">Warehouse B</SelectItem>
                        <SelectItem value="distribution_center">Distribution Center</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="visualBatchProfile">Batch Profile</Label>
                      <InfoTooltip text="Selects a preset detail/randomness level for batches (e.g., minimal vs full)." />
                    </div>
                    <Select value={batchProfile} onValueChange={setBatchProfile}>
                      <SelectTrigger id="visualBatchProfile">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="minimal">Minimal</SelectItem>
                        <SelectItem value="robot">Robot</SelectItem>
                        <SelectItem value="full">Full</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                </div>
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="visualLidarLogging">LiDAR Logging Mode</Label>
                    <InfoTooltip text="Choose what LiDAR data to save: none, full logs, or event-only snippets." />
                  </div>
                  <Select value={lidarLogging} onValueChange={setLidarLogging}>
                    <SelectTrigger id="visualLidarLogging">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="off">Off</SelectItem>
                      <SelectItem value="full">Full Logging</SelectItem>
                      <SelectItem value="events">Event Windows Only</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
                <div className="flex items-center justify-between">
                  <div className="flex items-center">
                    <Label htmlFor="configLidarBlackout">LiDAR Blackout Injector</Label>
                    <InfoTooltip text="Simulated sensor fault that blanks part of the LiDAR when triggered." />
                  </div>
                  <Switch id="configLidarBlackout" checked={lidarBlackout} onCheckedChange={setLidarBlackout} />
                </div>
                <div className="flex items-center justify-between">
                  <div className="flex items-center">
                    <Label htmlFor="configGhostObstacle">Ghost Obstacle Injector</Label>
                    <InfoTooltip text="Simulated sensor fault that inserts phantom obstacles into LiDAR." />
                  </div>
                  <Switch id="configGhostObstacle" checked={ghostObstacle} onCheckedChange={setGhostObstacle} />
                </div>
              </div>
              <DialogFooter>
                <Button onClick={() => setShowSettingsDialog(false)}>Done</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>

          <Dialog open={showTestDialog} onOpenChange={setShowTestDialog}>
            <DialogTrigger asChild>
              <Button variant="secondary" disabled={objects.length === 0}>
                <Play className="size-4 mr-2" />
                Test Run
              </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl" onOpenAutoFocus={(e) => e.preventDefault()}>
              <DialogHeader>
                <DialogTitle>Test Run Configuration</DialogTitle>
                <DialogDescription>
                  Run a single simulation with visualization
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="slowmo">Slow Motion: {slowmo}x</Label>
                    <InfoTooltip text="Playback speed for realtime/GUI runs." />
                  </div>
                  <Slider
                    id="slowmo"
                    min={0.1}
                    max={5}
                    step={0.1}
                    value={[slowmo]}
                    onValueChange={([v]) => setSlowmo(v)}
                  />
                </div>
                <div className="flex items-center justify-between">
                  <Label htmlFor="testDebugVisuals">Debug Visuals</Label>
                  <Switch id="testDebugVisuals" checked={debugVisuals} onCheckedChange={setDebugVisuals} />
                </div>
                <div className="bg-slate-50 p-3 rounded-lg">
                  <p className="text-xs font-mono text-slate-600 break-all">
                    edgesim run-one "{generatePrompt()}" --seed {seed} --gui --realtime --dt {timestep} --slowmo {slowmo}
                    {siteProfile !== "default" && ` --site ${siteProfile}`}
                    {debugVisuals && ` --debug-visuals`}
                    {lidarLogging === "full" && ` --lidar-logging`}
                    {lidarLogging === "events" && ` --lidar-events-only`}
                  </p>
                </div>
              </div>
              <DialogFooter>
                <Button variant="outline" onClick={() => setShowTestDialog(false)}>Cancel</Button>
                <Button onClick={handleTestRun}>Run Test</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>

          <Dialog open={showBatchDialog} onOpenChange={setShowBatchDialog}>
            <DialogTrigger asChild>
              <Button disabled={objects.length === 0}>
                <Database className="size-4 mr-2" />
                Create Data
              </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl" onOpenAutoFocus={(e) => e.preventDefault()}>
              <DialogHeader>
                <DialogTitle>Batch Run Configuration</DialogTitle>
                <DialogDescription>
                  Generate multiple simulation runs for data collection
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4">
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="numRuns">Number of Runs</Label>
                    <InfoTooltip text="Number of runs to generate with different seeds." />
                  </div>
                  <Input
                    id="numRuns"
                    type="number"
                    min={1}
                    max={10000}
                    value={numRuns}
                    onChange={(e) => setNumRuns(parseInt(e.target.value) || 1)}
                    autoFocus={false}
                  />
                </div>
                <div className="space-y-2">
                  <div className="flex items-center">
                    <Label htmlFor="batchSeed">Base Seed</Label>
                    <InfoTooltip text="Controls randomness. Reuse to reproduce results; change to vary the scenario." />
                  </div>
                  <Input
                    id="batchSeed"
                    type="number"
                    value={seed}
                    onChange={(e) => setSeed(parseInt(e.target.value) || 0)}
                  />
                </div>
                <div className="grid sm:grid-cols-2 gap-4">
                  <div className="space-y-2">
                    <div className="flex items-center">
                      <Label htmlFor="visualTimeBudget">Time Budget (seconds)</Label>
                      <InfoTooltip text="Target wall-clock limit for a batch; runs aim to complete within this window." />
                    </div>
                    <Input
                      id="visualTimeBudget"
                      type="number"
                      min={0}
                      value={timeBudget}
                      onChange={(e) => setTimeBudget(parseInt(e.target.value) || 120)}
                    />
                  </div>
                  <div className="flex items-center justify-between pt-7">
                    <div className="flex items-center">
                      <Label htmlFor="visualAutoDegrade">Auto Degrade</Label>
                      <InfoTooltip text="If the budget is tight, shortens runs (and may lower LiDAR rate) to stay on time." />
                    </div>
                    <Switch id="visualAutoDegrade" checked={autoDegrade} onCheckedChange={setAutoDegrade} />
                  </div>
                </div>
                <div className="bg-slate-50 p-3 rounded-lg">
                  <p className="text-xs font-mono text-slate-600 break-all">
                    edgesim run-batch "{generatePrompt()}" --runs {numRuns} --seed {seed} --profile {batchProfile}
                    {siteProfile !== "default" && ` --site ${siteProfile}`}
                    {timeBudget && ` --time-budget-min ${timeBudget}`}
                    {autoDegrade && ` --auto-degrade`}
                    {lidarLogging === "full" && ` --lidar-logging`}
                    {lidarLogging === "events" && ` --lidar-events-only`}
                  </p>
                </div>
              </div>
              <DialogFooter>
                <Button variant="outline" onClick={() => setShowBatchDialog(false)}>Cancel</Button>
                <Button onClick={handleBatchRun}>Start Batch</Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
        </div>
      </div>

      <div className="flex-1 overflow-hidden">
        <div className="h-full flex gap-4 p-4">
          <div className="flex-1 flex flex-col gap-2" style={{ height: '550px' }}>
            <div className="text-sm text-slate-600 text-center">
              Click and drag objects to reposition them
            </div>
            <div className="flex-1 bg-white rounded-lg shadow-sm border overflow-hidden">
              <WarehouseCanvas
                objects={objects}
                selectedId={selectedId}
                onSelectId={setSelectedId}
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
                onMouseUp={handleMouseUp}
                isDragging={dragState !== null}
              />
            </div>
          </div>

          <div style={{ width: '320px', height: '550px', display: 'flex', flexDirection: 'column' }}>
            <Card className="h-full flex flex-col">
              <CardHeader className="flex-shrink-0">
                <CardTitle>Objects</CardTitle>
                <CardDescription>Add and configure scene objects</CardDescription>
              </CardHeader>
              <CardContent className="flex-1 overflow-hidden p-0">
                <ScrollArea className="h-full px-6 pb-6">
                  <div className="space-y-6">
                    <ObjectPanel onAddObject={addObject} />
                    
                    {selectedObj && (
                      <>
                        <Separator />
                        <div className="space-y-3">
                          <div className="flex items-center justify-between">
                            <h3 className="font-medium">Selected Object</h3>
                            {selectedObj.id !== 'main-aisle' && (
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={() => deleteObject(selectedObj.id)}
                              >
                                <Trash2 className="size-4" />
                              </Button>
                            )}
                          </div>
                          <ObjectEditor
                            object={selectedObj}
                            onUpdateCoords={(coords) => {
                              setObjects(objects.map(obj =>
                                obj.id === selectedObj.id ? { ...obj, coords } : obj
                              ));
                            }}
                            onUpdateObject={(updated) => {
                              setObjects(objects.map(obj =>
                                obj.id === selectedObj.id ? updated : obj
                              ));
                            }}
                          />
                        </div>
                      </>
                    )}

                    {objects.length > 0 && (
                      <>
                        <Separator />
                        <div className="space-y-3">
                          <h3 className="font-medium">Scene Objects ({objects.length})</h3>
                          <div className="space-y-2">
                            {objects.map(obj => {
                              const meta = Object.values(OBJECT_CATEGORIES)
                                .flat()
                                .find(m => m.type === obj.type);
                              const coordText = obj.coords.length === 1
                                ? `(${obj.coords[0].x}, ${obj.coords[0].y})`
                                : `(${obj.coords[0].x}, ${obj.coords[0].y}) → (${obj.coords[1].x}, ${obj.coords[1].y})`;
                              
                              return (
                                <div
                                  key={obj.id}
                                  className={`p-2 rounded border cursor-pointer ${
                                    selectedId === obj.id ? 'border-blue-500 bg-blue-50' : 'border-slate-200'
                                  }`}
                                  onClick={() => setSelectedId(obj.id)}
                                >
                                  <div className="flex items-center justify-between mb-1">
                                    <Badge style={{ backgroundColor: meta?.color }} className="text-xs">
                                      {meta?.label}
                                    </Badge>
                                    {obj.id !== 'main-aisle' && (
                                      <Button
                                        variant="ghost"
                                        size="sm"
                                        onClick={(e) => {
                                          e.stopPropagation();
                                          deleteObject(obj.id);
                                        }}
                                      >
                                        <Trash2 className="size-3" />
                                      </Button>
                                    )}
                                  </div>
                                  <p className="text-xs text-slate-500">{coordText}</p>
                                </div>
                              );
                            })}
                          </div>
                        </div>
                      </>
                    )}
                  </div>
                </ScrollArea>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </div>
  );
}

function ObjectPanel({ onAddObject }: { onAddObject: (type: ObjectType, coords: number) => void }) {
  return (
    <Tabs defaultValue="dynamic" className="w-full">
      <TabsList className="grid w-full grid-cols-4 text-xs">
        <TabsTrigger value="dynamic">Dynamic</TabsTrigger>
        <TabsTrigger value="static">Static</TabsTrigger>
        <TabsTrigger value="hazards">Hazards</TabsTrigger>
        <TabsTrigger value="injectors">Injectors</TabsTrigger>
      </TabsList>
      <TabsContent value="dynamic" className="space-y-2">
        {OBJECT_CATEGORIES.dynamic.map(obj => (
          <Button
            key={obj.type}
            variant="outline"
            className="w-full justify-start"
            onClick={() => onAddObject(obj.type, obj.coords)}
          >
            {obj.label}
          </Button>
        ))}
      </TabsContent>
      <TabsContent value="static" className="space-y-2">
        {OBJECT_CATEGORIES.static.map(obj => (
          <Button
            key={obj.type}
            variant="outline"
            className="w-full justify-start"
            onClick={() => onAddObject(obj.type, obj.coords)}
          >
            {obj.label}
          </Button>
        ))}
      </TabsContent>
      <TabsContent value="hazards" className="space-y-2">
        {OBJECT_CATEGORIES.hazards.map(obj => (
          <Button
            key={obj.type}
            variant="outline"
            className="w-full justify-start"
            onClick={() => onAddObject(obj.type, obj.coords)}
          >
            {obj.label}
          </Button>
        ))}
      </TabsContent>
      <TabsContent value="injectors" className="space-y-2">
        {OBJECT_CATEGORIES.injectors.map(obj => (
          <Button
            key={obj.type}
            variant="outline"
            className="w-full justify-start"
            onClick={() => onAddObject(obj.type, obj.coords)}
          >
            {obj.label}
          </Button>
        ))}
      </TabsContent>
    </Tabs>
  );
}

function ObjectEditor({
  object,
  onUpdateCoords,
  onUpdateObject
}: {
  object: SceneObject;
  onUpdateCoords: (coords: Coordinate[]) => void;
  onUpdateObject: (obj: SceneObject) => void;
}) {
  const updateCoord = (index: number, axis: 'x' | 'y', value: number) => {
    const newCoords = [...object.coords];
    newCoords[index] = { ...newCoords[index], [axis]: value };
    onUpdateCoords(newCoords);
  };

  const mergeOptions = (patch: Partial<NonNullable<SceneObject["options"]>>) => {
    onUpdateObject({
      ...object,
      options: { ...(object.options || {}), ...patch },
    });
  };

  const meta = Object.values(OBJECT_CATEGORIES)
    .flat()
    .find(m => m.type === object.type) || OBJECT_CATEGORIES.robot;

  return (
    <div className="space-y-3">
      <Badge style={{ backgroundColor: meta?.color }}>{meta?.label}</Badge>
      {object.coords.map((coord, i) => (
        <div key={i} className="space-y-2">
          <Label className="text-xs text-slate-600">
            {object.coords.length === 1 ? 'Position' : i === 0 ? 'Start' : 'End'}
          </Label>
          <div className="grid grid-cols-2 gap-2">
            <div className="space-y-1">
              <Label htmlFor={`x-${i}`} className="text-xs">X</Label>
              <Input
                id={`x-${i}`}
                type="number"
                step={0.5}
                min={0}
                max={20}
                value={coord.x}
                onChange={(e) => updateCoord(i, 'x', parseFloat(e.target.value) || 0)}
              />
            </div>
            <div className="space-y-1">
              <Label htmlFor={`y-${i}`} className="text-xs">Y</Label>
              <Input
                id={`y-${i}`}
                type="number"
                step={0.5}
                min={0}
                max={20}
                value={coord.y}
                onChange={(e) => updateCoord(i, 'y', parseFloat(e.target.value) || 0)}
              />
            </div>
          </div>
        </div>
      ))}

      {object.type === "forklift" && (
        <div className="space-y-2">
          <Label className="text-xs text-slate-600">Forklift options</Label>
          {([
            { key: "reversing_mode", label: "Reversing mode", hint: "Ping-pong / backing" },
            { key: "alarm", label: "Backup alarm", hint: "Beeping enabled" },
            { key: "reflective", label: "Reflective load", hint: "High-vis reflective surfaces" },
            { key: "load_overhang", label: "Load overhang", hint: "Extended pallet/overhang" },
          ] as const).map(opt => (
            <div key={opt.key} className="flex items-center justify-between">
              <div>
                <p className="text-sm">{opt.label}</p>
                <p className="text-xs text-slate-500">{opt.hint}</p>
              </div>
              <Switch
                checked={Boolean(object.options?.forklift?.[opt.key])}
                onCheckedChange={(checked) => {
                  const current = object.options?.forklift || { reversing_mode: false, alarm: false, reflective: false, load_overhang: false };
                  mergeOptions({ forklift: { ...current, [opt.key]: checked } as ForkliftOptions });
                }}
              />
            </div>
          ))}
        </div>
      )}

      {object.type === "rack" && (
        <div className="space-y-2">
          <Label className="text-xs text-slate-600">Rack options</Label>
          <div className="flex items-center justify-between">
            <div>
              <p className="text-sm">High rack</p>
              <p className="text-xs text-slate-500">Increase height/occlusion</p>
            </div>
            <Switch
              checked={Boolean(object.options?.rack?.high_rack)}
              onCheckedChange={(checked) => {
                const current = object.options?.rack || { high_rack: false };
                mergeOptions({ rack: { ...current, high_rack: checked } });
              }}
            />
          </div>
        </div>
      )}
    </div>
  );
}

function WarehouseCanvas({
  objects,
  selectedId,
  onSelectId,
  onMouseDown,
  onMouseMove,
  onMouseUp,
  isDragging
}: {
  objects: SceneObject[];
  selectedId: string | null;
  onSelectId: (id: string | null) => void;
  onMouseDown: (objectId: string, coordIndex: number, e: React.MouseEvent<SVGElement>) => void;
  onMouseMove: (e: React.MouseEvent<SVGSVGElement>) => void;
  onMouseUp: () => void;
  isDragging: boolean;
}) {
  const GRID_SIZE = 20;
  const [hoveredCoord, setHoveredCoord] = useState<{ x: number; y: number } | null>(null);

  const renderObject = (obj: SceneObject) => {
    const isSelected = selectedId === obj.id;
    const commonProps = {
      isSelected,
      objectId: obj.id,
      onSelectId,
      onMouseDown
    };

    switch (obj.type) {
      case "robot":
        // Render AMR Robot as simple red disc at start position with path line to goal
        const robotRadius = 0.4;
        const robotAngle = Math.atan2(obj.coords[1].y - obj.coords[0].y, obj.coords[1].x - obj.coords[0].x);
        const arrowSize = 0.15;
        const robotMidX = (obj.coords[0].x + obj.coords[1].x) / 2;
        const robotMidY = (obj.coords[0].y + obj.coords[1].y) / 2;
        const xSize = 0.12;
        
        return (
          <g key={obj.id} className="group">
            {/* Invisible wide path for easier hovering */}
            <line
              x1={obj.coords[0].x}
              y1={obj.coords[0].y}
              x2={obj.coords[1].x}
              y2={obj.coords[1].y}
              stroke="transparent"
              strokeWidth={0.4}
              strokeLinecap="round"
              style={{ cursor: 'grab' }}
              onClick={() => commonProps.onSelectId(obj.id)}
            />
            {/* Path line to goal - dashed */}
            <line
              x1={obj.coords[0].x}
              y1={obj.coords[0].y}
              x2={obj.coords[1].x}
              y2={obj.coords[1].y}
              stroke="#ef4444"
              strokeWidth={0.04}
              opacity={0.4}
              strokeLinecap="round"
              strokeDasharray="0.2,0.2"
            />
            {/* Direction arrow at end */}
            <g transform={`translate(${obj.coords[1].x}, ${obj.coords[1].y}) rotate(${robotAngle * 180 / Math.PI})`}>
              <path
                d={`M 0 0 L ${-arrowSize} ${-arrowSize/2} L ${-arrowSize} ${arrowSize/2} Z`}
                fill="#ef4444"
                opacity={0.7}
              />
            </g>
            {/* Robot at start position - red disc */}
            <circle
              cx={obj.coords[0].x}
              cy={obj.coords[0].y}
              r={robotRadius}
              fill="#ef4444"
              stroke={isSelected ? "#3b82f6" : "#b91c1c"}
              strokeWidth={0.04}
              onClick={() => commonProps.onSelectId(obj.id)}
              style={{ cursor: 'grab' }}
            />
            {/* Control points for dragging start and end */}
            {[obj.coords[0], obj.coords[1]].map((coord, i) => (
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
                  onMouseDown={(e) => commonProps.onMouseDown(obj.id, i, e)}
                />
              </g>
            ))}
            {/* Middle drag point - X shape */}
            <g
              onMouseDown={(e) => commonProps.onMouseDown(obj.id, 2, e)}
              style={{ cursor: 'move' }}
              className="group-hover:opacity-100"
              opacity={isSelected ? 1 : 0}
            >
              <line
                x1={robotMidX - xSize}
                y1={robotMidY - xSize}
                x2={robotMidX + xSize}
                y2={robotMidY + xSize}
                stroke="#3b82f6"
                strokeWidth={0.04}
                strokeLinecap="round"
              />
              <line
                x1={robotMidX - xSize}
                y1={robotMidY + xSize}
                x2={robotMidX + xSize}
                y2={robotMidY - xSize}
                stroke="#3b82f6"
                strokeWidth={0.04}
                strokeLinecap="round"
              />
              {/* Invisible larger hit area for easier hover */}
              <circle
                cx={robotMidX}
                cy={robotMidY}
                r={0.35}
                fill="transparent"
              />
            </g>
          </g>
        );
      case "human":
        return <HumanObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} {...commonProps} />;
      case "forklift":
        return <ForkliftObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} opts={obj.options?.forklift} {...commonProps} />;
      case "cart":
        return <CartObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} {...commonProps} />;
      case "tugger":
        return <TuggerObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} {...commonProps} />;
      case "wall":
        return <WallObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} {...commonProps} />;
      case "rack":
        return <RackObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} high={Boolean(obj.options?.rack?.high_rack)} {...commonProps} />;
      case "obstacle":
        return <ObstacleObject key={obj.id} pos={obj.coords[0]} {...commonProps} />;
      case "cart_block":
        return <CartBlockObject key={obj.id} pos={obj.coords[0]} {...commonProps} />;
      case "wet_patch":
        return <WetPatchObject key={obj.id} start={obj.coords[0]} end={obj.coords[1]} {...commonProps} />;
      case "spill":
        return <SpillObject key={obj.id} pos={obj.coords[0]} {...commonProps} />;
      case "falling_object":
        return <FallingObjectInjector key={obj.id} pos={obj.coords[0]} {...commonProps} />;
      default:
        return null;
    }
  };

  return (
    <svg
      viewBox={`-1 -1 ${GRID_SIZE + 1} ${GRID_SIZE + 1}`}
      className="w-full h-full"
      onMouseMove={(e) => {
        onMouseMove(e);
        // Only update hovered coordinate if not dragging
        if (!isDragging) {
          const svg = e.currentTarget;
          const pt = svg.createSVGPoint();
          pt.x = e.clientX;
          pt.y = e.clientY;
          const cursorPt = pt.matrixTransform(svg.getScreenCTM()?.inverse());
          const snappedX = roundToHalf(Math.max(0, Math.min(20, cursorPt.x)));
          const snappedY = roundToHalf(Math.max(0, Math.min(20, cursorPt.y)));
          setHoveredCoord({ x: snappedX, y: snappedY });
        }
      }}
      onMouseUp={() => {
        onMouseUp();
      }}
      onMouseLeave={() => {
        onMouseUp();
        setHoveredCoord(null);
      }}
      style={{ cursor: isDragging ? 'grabbing' : 'default' }}
    >
      {/* Background */}
      <rect width={GRID_SIZE} height={GRID_SIZE} fill="#fafafa" />

      {/* Grid lines */}
      {Array.from({ length: GRID_SIZE + 1 }).map((_, i) => (
        <g key={i}>
          <line
            x1={i}
            y1={0}
            x2={i}
            y2={GRID_SIZE}
            stroke="#e5e7eb"
            strokeWidth={0.02}
          />
          <line
            x1={0}
            y1={i}
            x2={GRID_SIZE}
            y2={i}
            stroke="#e5e7eb"
            strokeWidth={0.02}
          />
        </g>
      ))}

      {/* Coordinate labels with highlighting - only when not dragging */}
      {Array.from({ length: 21 }).map((_, i) => {
        const pos = i;
        const isHighlightedX = !isDragging && hoveredCoord && hoveredCoord.x === pos;
        const isHighlightedY = !isDragging && hoveredCoord && hoveredCoord.y === pos;
        return (
          <g key={i}>
            {/* X-axis labels */}
            {i % 5 === 0 && (
              <>
                <text
                  x={pos}
                  y={-0.4}
                  fontSize="0.4"
                  fill={isHighlightedX ? "#3b82f6" : "#94a3b8"}
                  fontWeight={isHighlightedX ? "bold" : "normal"}
                  textAnchor="middle"
                  style={{ pointerEvents: 'none', userSelect: 'none' }}
                >
                  {pos}
                </text>
                {/* Y-axis labels */}
                <text
                  x={-0.6}
                  y={pos + 0.15}
                  fontSize="0.4"
                  fill={isHighlightedY ? "#3b82f6" : "#94a3b8"}
                  fontWeight={isHighlightedY ? "bold" : "normal"}
                  textAnchor="end"
                  style={{ pointerEvents: 'none', userSelect: 'none' }}
                >
                  {pos}
                </text>
              </>
            )}
          </g>
        );
      })}

      {/* Objects */}
      {objects.map(renderObject)}
    </svg>
  );
}
