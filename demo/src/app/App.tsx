import { useEffect, useId, useMemo, useRef, useState, type ReactNode } from "react";
import {
  Check,
  ChevronDown,
  CircleAlert,
  Database,
  Download,
  FileCode2,
  FileText,
  Gauge,
  Map,
  ShieldCheck,
  Timer,
} from "lucide-react";

import { Button } from "./components/ui/button";
import { Card } from "./components/ui/card";
import { DropdownMenu, DropdownMenuContent, DropdownMenuSeparator, DropdownMenuTrigger } from "./components/ui/dropdown-menu";
import heroImage from "../assets/hero-background.png";
import demoDataRaw from "./data/demoData.json";

type DemoPayload = {
  generated_at_utc: string;
  global: {
    prompt_count: number;
    pair_count: number;
    test_runs: number;
    batch_runs: number;
    total_runs: number;
    batch_success_rate_pct: number;
    test_success_rate_pct: number;
    avg_batch_time_s: number;
    avg_sims_per_min: number;
    event_totals: Record<string, number>;
    hardest_prompt: string | null;
    easiest_prompt: string | null;
  };
  scenarios: Scenario[];
};

type Scenario = {
  id: string;
  title: string;
  summary: string;
  prompt: string;
  timestamps: {
    test: string;
    batch: string;
  };
  world: {
    map_size_m: [number, number];
    start: [number, number];
    goal: [number, number];
    aisles: Rect[];
    walls: Rect[];
    racks: Rect[];
    objects?: Array<{
      type: string;
      aabb: Rect;
      center?: [number, number];
      half_extents?: [number, number];
      yaw?: number;
    }>;
    traction: Rect[];
    humans: Array<{ waypoints: [number, number][] }>;
    vehicles: Array<{ type: string; path: [number, number][] }>;
  };
  test: {
    outcome: string;
    duration_s: number;
    avg_speed_mps: number;
    min_ttc_s: number;
    events: Record<string, number>;
    timeline: Array<{ t: number; label: string; x: number; y: number }>;
    path: Array<{ t: number; x: number; y: number }>;
    actors: Array<{
      id: string;
      type: string;
      samples: Array<{ t: number; x: number; y: number; yaw: number; phase: string }>;
    }>;
    rows: number;
    lidar: {
      mode: string;
      artifact_logged: boolean;
      sample_count: number;
      min_clearance_m: number | null;
      avg_clearance_m: number | null;
      p10_clearance_m: number | null;
      p50_clearance_m: number | null;
      p90_clearance_m: number | null;
      noise_sigma: number | null;
      dropout_pct: number | null;
      max_range_m: number | null;
      series: Array<{ t: number; clearance_m: number }>;
      playback: {
        available: boolean;
        source_frames: number;
        frame_count: number;
        beam_count: number;
        frame_stride: number;
        ray_stride: number;
        frames: Array<{
          t: number;
          pose: [number, number, number];
          laser_origin: [number, number];
          points: [number, number][];
        }>;
      };
    };
  };
  batch: {
    runs: number;
    successes: number;
    failures: number;
    avg_time_s: number;
    avg_steps: number;
    coverage_pct: {
      wet: number;
      tight_clearance: number;
      collision_human: number;
      success: number;
    };
    perf: {
      elapsed_sec: number;
      sims_per_min: number;
      auto_degraded: boolean;
      profile: string;
    };
    event_counts: Record<string, number>;
    time_distribution_s: {
      min: number;
      p10: number;
      p50: number;
      p90: number;
      max: number;
      avg: number;
    };
    step_distribution: {
      min: number;
      max: number;
      avg: number;
    };
    sample_runs: Array<{
      run_id: string;
      success: boolean;
      time_s: number;
      steps: number;
    }>;
    success_curve: Array<{
      idx: number;
      success: 0 | 1;
      time_s: number;
    }>;
  };
  validation: {
    ok: boolean;
    errors: string[];
  };
  config: {
    lidar_hz: number;
    dt: number;
    duration_s: number;
    agents: number;
    humans: number;
    vehicles: number;
    traction_patches: number;
    racks: number;
    walls: number;
  };
  artifacts: {
    test: Record<string, string>;
    batch: Record<string, string>;
  };
};

type Rect = [number, number, number, number];
type ExplorerTab = "overview" | "artifacts";
type LidarPlaybackFrame = Scenario["test"]["lidar"]["playback"]["frames"][number];

const demoData = demoDataRaw as unknown as DemoPayload;

const artifactLabels: Record<string, string> = {
  "scenario.yaml": "Scenario settings (YAML)",
  "world.json": "Map layout (JSON)",
  "run_one.csv": "Single run timeline (CSV)",
  "lidar.npz": "Distance sensor scan log (NPZ)",
  "actors.csv": "Actor movement traces (CSV)",
  "validation.json": "Validation checks (JSON)",
  "dataset_manifest.json": "Dataset manifest (JSON)",
  "report.html": "HTML summary report",
  "report.md": "Markdown summary report",
  "summary.json": "Batch summary (JSON)",
  "coverage.json": "Coverage summary (JSON)",
  "perf.json": "Performance summary (JSON)",
  "per_run_index.csv": "Per-run index (CSV)",
};

const RUN_DOT_BASELINE_RUNS = 1000;
const RUN_DOT_BASE_RADIUS_PX = 6.6;
const RUN_DOT_MIN_RADIUS_PX = 1.4;
const RUN_DOT_GAP_PX = 3;
const RUN_DOT_MOBILE_BASELINE_RUNS = 300;
const RUN_DOT_MOBILE_BASE_RADIUS_PX = 4.6;
const RUN_DOT_MOBILE_MIN_RADIUS_PX = 1.1;
const RUN_DOT_MOBILE_GAP_PX = 2;

function isCollisionCategory(outcome: string): boolean {
  return outcome === "collision_human" || outcome === "collision_static" || outcome === "collision_dominant";
}

function outcomeLabel(outcome: string): string {
  if (outcome === "mission_success") return "success";
  if (outcome === "collision_human") return "collision (human)";
  if (outcome === "collision_static") return "collision (static)";
  if (outcome === "arrival_dominant" || outcome === "success_dominant" || outcome === "low_collision") return "mostly successful runs";
  if (outcome === "contact_dominant" || outcome === "collision_dominant") return "collision-heavy runs";
  return outcome;
}

function App() {
  const [activeScenarioId, setActiveScenarioId] = useState(demoData.scenarios[0]?.id ?? "");
  const [activeTab, setActiveTab] = useState<ExplorerTab>("overview");
  const [desktopHeaderHeight, setDesktopHeaderHeight] = useState(0);
  const desktopHeaderRef = useRef<HTMLElement | null>(null);

  const activeScenario = useMemo(
    () => demoData.scenarios.find((scenario) => scenario.id === activeScenarioId) ?? demoData.scenarios[0],
    [activeScenarioId],
  );

  if (!activeScenario) {
    return <div className="min-h-screen bg-slate-950 text-slate-100 p-10">No demo scenarios found.</div>;
  }

  const repeatedRunCount = Math.max(1, activeScenario.batch.success_curve.length);
  const repeatedRadiusScale = repeatedRunCount > RUN_DOT_BASELINE_RUNS ? Math.sqrt(RUN_DOT_BASELINE_RUNS / repeatedRunCount) : 1;
  const repeatedDotRadiusPx = Math.max(RUN_DOT_MIN_RADIUS_PX, RUN_DOT_BASE_RADIUS_PX * repeatedRadiusScale);
  const repeatedDotDiameterPx = repeatedDotRadiusPx * 2;

  useEffect(() => {
    const measureDesktopHeader = () => {
      if (!desktopHeaderRef.current) return;
      setDesktopHeaderHeight(desktopHeaderRef.current.offsetHeight);
    };

    measureDesktopHeader();
    window.addEventListener("resize", measureDesktopHeader);
    return () => window.removeEventListener("resize", measureDesktopHeader);
  }, []);

  const scrollTo = (id: string) => {
    document.getElementById(id)?.scrollIntoView({ behavior: "smooth", block: "start" });
  };

  return (
    <div
      className="min-h-screen bg-slate-950 text-slate-100"
      style={{ fontFamily: '"Sora", "Avenir Next", "Segoe UI", sans-serif' }}
    >
      <MobileDemo
        activeScenario={activeScenario}
        activeTab={activeTab}
        onPickScenario={(scenarioId) => {
          setActiveScenarioId(scenarioId);
          setActiveTab("overview");
        }}
        onSetTab={setActiveTab}
        onScrollTo={scrollTo}
      />

      <div className="hidden md:block">
      <header ref={desktopHeaderRef} className="sticky top-0 z-40 border-b border-slate-800/80 bg-slate-950/85 backdrop-blur">
        <div className="mx-auto flex max-w-7xl items-center justify-between px-6 py-4">
          <button
            className="text-left"
            onClick={() => scrollTo("hero")}
            aria-label="Jump to top of page"
          >
            <div className="text-xl tracking-tight">EdgeSim</div>
            <div className="text-xs text-slate-400">Simulated warehouse edge cases</div>
          </button>
          <div className="hidden gap-3 sm:flex">
            <Button variant="outline" className="border-slate-700 bg-transparent text-slate-100" onClick={() => scrollTo("explorer")}>
              Scenario Explorer
            </Button>
            <Button variant="outline" className="border-slate-700 bg-transparent text-slate-100" onClick={() => scrollTo("workflow")}>
              Workflow
            </Button>
            <Button variant="outline" className="border-slate-700 bg-transparent text-slate-100" onClick={() => scrollTo("comparison")}>
              Compare Scenarios
            </Button>
            <ContactMenuButton />
          </div>
        </div>
      </header>

      <section
        id="hero"
        className="relative overflow-hidden border-b border-slate-800"
        style={{ minHeight: desktopHeaderHeight > 0 ? `calc(100svh - ${desktopHeaderHeight}px)` : undefined }}
      >
        <div className="absolute inset-0">
          <img src={heroImage} alt="Warehouse aisle background" className="h-full w-full object-cover opacity-30" />
          <div className="absolute inset-0 bg-gradient-to-b from-slate-900/40 via-slate-950/90 to-slate-950" />
        </div>

        <div className="relative mx-auto max-w-7xl px-6 pb-16 pt-10">
          <div className="max-w-3xl">
            <h1 className="mb-4 text-4xl leading-tight sm:text-6xl">
              Evaluate warehouse robot safety
              <span className="text-cyan-300"> before deployment</span>
            </h1>
            <p className="max-w-2xl text-lg text-slate-300">
              EdgeSim lets teams describe a warehouse situation through text and coordinates, then automatically builds and runs a simulation for it. You can
              inspect one example replay, then compare repeated runs to see how often collisions, near-misses, and delays occur. Each metric is
              backed by downloadable logs and reports.
            </p>

          </div>

          <div className="mt-16 grid gap-4 sm:grid-cols-2 lg:grid-cols-5">
            <StatCard label="Scenarios analyzed" value={String(demoData.global.prompt_count)} icon={<FileText className="h-5 w-5" />} />
            <StatCard label="Total recorded runs" value={String(demoData.global.total_runs)} icon={<Database className="h-5 w-5" />} />
            <StatCard
              label="Runs with collisions"
              value={`${Math.max(0, 100 - demoData.global.batch_success_rate_pct).toFixed(2)}%`}
              icon={<ShieldCheck className="h-5 w-5" />}
            />
            <StatCard label="Average repeated-run time" value={`${demoData.global.avg_batch_time_s.toFixed(2)}s`} icon={<Timer className="h-5 w-5" />} />
            <StatCard label="Average simulation speed" value={`${demoData.global.avg_sims_per_min.toFixed(2)} sims/min`} icon={<Gauge className="h-5 w-5" />} />
          </div>
        </div>
      </section>

      <section id="explorer" className="border-y border-slate-800 bg-slate-900/30">
        <div className="mx-auto max-w-7xl px-6 pb-16 pt-10">
          <div className="mb-8 max-w-3xl">
            <h2 className="text-3xl sm:text-4xl">Scenario Explorer</h2>
            <p className="mt-3 text-slate-300">
              Pick a scenario to inspect one replay, summary results from repeated runs, and the files used to compute each metric.
            </p>
          </div>

          <div className="grid gap-6 lg:grid-cols-[330px_1fr]">
            <Card className="self-start border-cyan-500/25 bg-cyan-950/10 p-3 shadow-[0_0_0_1px_rgba(34,211,238,0.08)]">
              <p className="mb-1 px-1 text-xs uppercase tracking-wide text-cyan-200/85">Pick a scenario</p>
              <div className="space-y-3">
                {demoData.scenarios.map((scenario) => {
                  const isActive = scenario.id === activeScenario.id;
                  const collisionRate = Math.max(0, 100 - scenario.batch.coverage_pct.success);
                  return (
                    <button
                      key={scenario.id}
                      onClick={() => {
                        setActiveScenarioId(scenario.id);
                        setActiveTab("overview");
                      }}
                      className={`w-full rounded-xl border p-4 text-left transition ${
                        isActive
                          ? "border-cyan-500 bg-cyan-500/10"
                          : "border-slate-800 bg-slate-900/50 hover:border-slate-700 hover:bg-slate-900"
                      }`}
                    >
                      <div className="mb-2 flex items-center justify-start gap-3">
                        <span className="rounded-full bg-orange-500/20 px-2 py-0.5 text-xs text-orange-200">
                          {collisionRate.toFixed(1)}% collision risk
                        </span>
                      </div>
                      <h3 className="text-lg leading-tight text-white">{scenario.title}</h3>
                    </button>
                  );
                })}
              </div>
            </Card>

            <div className="space-y-4">
              <Card className="border-slate-800 bg-slate-950/80 p-6">
                <div className="mb-4 flex flex-wrap items-center gap-4">
                  <div>
                    <h3 className="text-2xl text-white">{activeScenario.title}</h3>
                  </div>
                </div>

                <div className="rounded-lg border border-slate-800 bg-slate-900/60 p-4">
                  <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Scenario description</p>
                  <p className="font-mono text-sm text-slate-100">{activeScenario.prompt}</p>
                </div>

                <div className="mt-4 grid gap-4 md:grid-cols-2">
                  <OutcomeCard
                    title="Example Replay"
                    outcome={activeScenario.test.outcome}
                    timeValue={`${activeScenario.test.duration_s.toFixed(2)}s`}
                    detailA={`Closest near-miss (sec): ${activeScenario.test.min_ttc_s.toFixed(2)}`}
                  />
                  <OutcomeCard
                    title={`${activeScenario.batch.runs} Repeats`}
                    outcome={Math.max(0, 100 - activeScenario.batch.coverage_pct.success) >= 50 ? "collision_dominant" : "low_collision"}
                    timeValue={`${activeScenario.batch.avg_time_s.toFixed(2)}s avg`}
                    detailA={`${Math.max(0, 100 - activeScenario.batch.coverage_pct.success).toFixed(1)}% collision`}
                  />
                </div>

                <div className="mt-5">
                  <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">
                    Repeated-run outcomes (left to right: run 1 to {activeScenario.batch.runs})
                  </p>
                  <div className="mb-2 flex flex-wrap gap-2 text-xs">
                    <span className="rounded-full bg-blue-500/20 px-2 py-0.5 text-blue-200">Success</span>
                    <span className="rounded-full bg-orange-500/20 px-2 py-0.5 text-orange-200">Collision</span>
                  </div>
                  <div className="overflow-x-auto rounded-md border border-slate-800/70 bg-slate-950/40 p-2">
                    <div
                      className="grid"
                      style={{
                        width: "100%",
                        gridAutoFlow: "row",
                        gridTemplateColumns: `repeat(auto-fill, minmax(${repeatedDotDiameterPx}px, ${repeatedDotDiameterPx}px))`,
                        gap: `${RUN_DOT_GAP_PX}px`,
                      }}
                    >
                      {activeScenario.batch.success_curve.map((point) => (
                        <span
                          key={point.idx}
                          title={`Run ${point.idx}: ${point.success ? "non-collision" : "collision"} (${point.time_s.toFixed(2)}s)`}
                          className={point.success ? "bg-blue-400/80" : "bg-orange-400/80"}
                          style={{
                            width: `${repeatedDotDiameterPx}px`,
                            height: `${repeatedDotDiameterPx}px`,
                            borderRadius: "9999px",
                          }}
                        />
                      ))}
                    </div>
                  </div>
                </div>
              </Card>

              <div className="flex flex-wrap gap-2">
                <TabButton label="Overview" active={activeTab === "overview"} onClick={() => setActiveTab("overview")} />
                <TabButton label="Generated data" active={activeTab === "artifacts"} onClick={() => setActiveTab("artifacts")} />
              </div>

              {activeTab === "overview" && <OverviewTab scenario={activeScenario} />}
              {activeTab === "artifacts" && <ArtifactsTab scenario={activeScenario} />}
            </div>
          </div>
        </div>
      </section>

      <section id="workflow" className="mx-auto max-w-7xl px-6 pb-16 pt-10">
        <div className="mb-8 max-w-3xl">
          <h2 className="text-3xl sm:text-4xl">How EdgeSim works</h2>
          <p className="mt-3 text-slate-300">
            This demo shows the full workflow: define a warehouse situation, run one replay, run repeated batches, and inspect the evidence files.
            The goal is practical and clear: verify safety and reliability before live deployment.
          </p>
        </div>
        <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
          <WorkflowCard
            compact
            title="1. Define the Situation"
            detail="Describe the warehouse situation through text and coordinates. EdgeSim converts it into map layout, moving actors, floor conditions, and sensor settings."
            code="edgesim simulate <scenario_text>"
          />
          <WorkflowCard
            compact
            title="2. Run One Replay"
            detail="Run once to inspect behavior over time and confirm the setup behaves as intended."
            code="edgesim run-one <scenario_text>"
          />
          <WorkflowCard
            compact
            title="3. Reliability Batch"
            detail="Run controlled repeats to measure success rate, collision frequency, near-misses, and completion-time spread."
            code="edgesim run-batch <scenario_text> --runs <n>"
          />
          <WorkflowCard
            compact
            title="4. Evidence Review"
            detail="Use map replay, LiDAR logs, coverage and performance summaries to verify results and use them for future robot training/testing."
            code="summary.json + coverage.json + report.html"
          />
        </div>
        <div className="mt-8 rounded-xl border border-slate-800 bg-slate-900/45 p-5">
          <h3 className="text-xl text-white">Why this project matters</h3>
          <p className="mt-2 text-slate-300">
            In robotics, the hardest failures are the ones you can't reproduce. EdgeSim turns "what if?" situations into the reproducible test
            runs you can generate at insanely fast speeds, so that teams can verify fixes, catch regressions early, generate lidar and pose data
            for training warehouse AMRs, and build confidence that improvements hold up across messy real-world conditions.
          </p>
        </div>
      </section>

      <section id="comparison" className="border-y border-slate-800/80 bg-slate-900/20">
        <div className="mx-auto max-w-7xl px-6 pb-16 pt-10">
          <div className="mb-6 max-w-3xl">
            <h2 className="text-3xl sm:text-4xl">Scenario Comparison</h2>
            <p className="mt-3 text-slate-300">
              See how changes in layout and traffic affect safety outcomes and completion time.
            </p>
          </div>

          <Card className="overflow-hidden border-slate-800 bg-slate-900/50">
            <div className="overflow-x-auto">
              <table className="min-w-full text-left text-sm">
                <thead className="bg-slate-900/80 text-slate-300">
                  <tr>
                    <th className="px-4 py-3">Scenario</th>
                    <th className="px-4 py-3">Collision rate</th>
                    <th className="px-4 py-3">Contact with person</th>
                    <th className="px-4 py-3">Avg completion time</th>
                    <th className="px-4 py-3">Simulation speed</th>
                    <th className="px-4 py-3">Number of runs</th>
                  </tr>
                </thead>
                <tbody>
                  {demoData.scenarios.map((scenario) => (
                    <tr key={scenario.id} className="border-t border-slate-800 text-slate-200">
                      <td className="px-4 py-3">
                        <p className="text-white">{scenario.title}</p>
                      </td>
                      <td className="px-4 py-3">{Math.max(0, 100 - scenario.batch.coverage_pct.success).toFixed(1)}%</td>
                      <td className="px-4 py-3">{scenario.batch.coverage_pct.collision_human.toFixed(1)}%</td>
                      <td className="px-4 py-3">{scenario.batch.avg_time_s.toFixed(2)}s</td>
                      <td className="px-4 py-3">{scenario.batch.perf.sims_per_min.toFixed(2)} sims/min</td>
                      <td className="px-4 py-3">{scenario.batch.runs}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </Card>
        </div>
      </section>

      <footer className="border-t border-slate-800 bg-slate-950/80">
        <div className="mx-auto max-w-7xl px-6 pb-10 pt-6">
          <div className="mt-2 space-y-1 text-sm text-slate-500">
            <p>Built by Vlad Kalinin</p>
            <p>
              GitHub Repository:{" "}
              <a
                href="https://github.com/Vortex-VK/EdgeSim"
                target="_blank"
                rel="noreferrer"
                className="underline-offset-2 hover:underline"
              >
                https://github.com/Vortex-VK/EdgeSim
              </a>
            </p>
          </div>
          <p className="mt-6 text-sm text-slate-500">Data snapshot generated at {demoData.generated_at_utc} UTC from local run evidence files.</p>
        </div>
      </footer>
      </div>
    </div>
  );
}

function StatCard({ label, value, icon, compact = false }: { label: string; value: string; icon: ReactNode; compact?: boolean }) {
  return (
    <Card className={`border-slate-800 bg-slate-900/70 ${compact ? "p-2.5" : "p-3"}`}>
      <div className={`flex items-center justify-center rounded-lg bg-cyan-500/15 text-cyan-300 ${compact ? "mb-0.5 h-7 w-7" : "mb-1 h-8 w-8"}`}>
        {icon}
      </div>
      <p className={`text-white ${compact ? "text-xl leading-none" : "text-2xl"}`}>{value}</p>
      <p className={compact ? "text-xs text-slate-400" : "text-sm text-slate-400"}>{label}</p>
    </Card>
  );
}

function WorkflowCard({ title, detail, code, compact = false }: { title: string; detail: string; code: string; compact?: boolean }) {
  return (
    <Card className={`border-slate-800 bg-slate-900/45 ${compact ? "p-2.5" : "p-5"}`}>
      <h3 className="text-lg text-white">{title}</h3>
      <p className={`${compact ? "mt-0.5 text-sm" : "mt-2 text-sm"} text-slate-300`}>{detail}</p>
      <p className={`${compact ? "mt-1 px-2.5 py-1.5" : "mt-4 px-3 py-2"} rounded bg-slate-950/80 font-mono text-xs text-cyan-200`}>{code}</p>
    </Card>
  );
}

function OutcomeCard({
  title,
  outcome,
  timeValue,
  detailA,
  compact = false,
}: {
  title: string;
  outcome: string;
  timeValue: string;
  detailA: string;
  compact?: boolean;
}) {
  const isCollision = isCollisionCategory(outcome);
  return (
    <Card className={`border ${compact ? "p-2.5" : "p-4"} ${isCollision ? "border-orange-500/40 bg-orange-900/10" : "border-blue-500/40 bg-blue-900/10"}`}>
      <div className={`${compact ? "mb-0.5" : "mb-3"} flex items-center justify-between`}>
        <p className="text-sm uppercase tracking-wide text-slate-300">{title}</p>
        <span
          className={`rounded-full px-2 py-0.5 text-xs ${
            isCollision ? "bg-orange-500/20 text-orange-200" : "bg-blue-500/20 text-blue-200"
          }`}
        >
          {isCollision ? "collision" : "success"}
        </span>
      </div>
      <p className={`${compact ? "text-base leading-tight" : "text-lg"} text-white`}>{outcomeLabel(outcome)}</p>
      <p className={`${compact ? "text-xs leading-tight" : "text-sm"} text-slate-300`}>{timeValue}</p>
      <p className={`${compact ? "mt-0 leading-tight" : "mt-2"} text-xs text-slate-400`}>{detailA}</p>
    </Card>
  );
}

function TabButton({ label, active, onClick }: { label: string; active: boolean; onClick: () => void }) {
  return (
    <button
      onClick={onClick}
      className={`rounded-full border px-4 py-2 text-sm transition ${
        active
          ? "border-cyan-500 bg-cyan-500/20 text-cyan-200"
          : "border-slate-700 bg-slate-900/50 text-slate-300 hover:border-slate-500"
      }`}
    >
      {label}
    </button>
  );
}

function ContactMenuButton({ triggerClassName = "" }: { triggerClassName?: string }) {
  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="outline" className={`border-slate-700 bg-transparent text-slate-100 ${triggerClassName}`}>
          Contact me
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end" className="w-80 border-slate-700 bg-slate-950/95 p-2 text-slate-100 backdrop-blur">
        <p className="px-2 py-1 text-[11px] uppercase tracking-wide text-slate-400">Contacts</p>
        <DropdownMenuSeparator className="bg-slate-800" />
        <div className="space-y-1.5 px-2 pb-1 pt-2 text-xs text-slate-300">
          <p>
            Phone number:{" "}
            <a href="tel:+17043020548" className="text-cyan-300 underline-offset-2 hover:underline">
              +1 (704) 302 0548
            </a>
          </p>
          <p>
            Email:{" "}
            <a href="mailto:kalinin.vlad.2003@gmail.com" className="text-cyan-300 underline-offset-2 hover:underline">
              kalinin.vlad.2003@gmail.com
            </a>
          </p>
          <p>
            LinkedIn:{" "}
            <a href="https://www.linkedin.com/in/kalinin-vlad/" target="_blank" rel="noreferrer" className="text-cyan-300 underline-offset-2 hover:underline">
              https://www.linkedin.com/in/kalinin-vlad/
            </a>
          </p>
        </div>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

function MobileDemo({
  activeScenario,
  activeTab,
  onPickScenario,
  onSetTab,
  onScrollTo,
}: {
  activeScenario: Scenario;
  activeTab: ExplorerTab;
  onPickScenario: (scenarioId: string) => void;
  onSetTab: (tab: ExplorerTab) => void;
  onScrollTo: (id: string) => void;
}) {
  const mobileRepeatedRunCount = Math.max(1, activeScenario.batch.success_curve.length);
  const mobileRepeatedRadiusScale =
    mobileRepeatedRunCount > RUN_DOT_MOBILE_BASELINE_RUNS ? Math.sqrt(RUN_DOT_MOBILE_BASELINE_RUNS / mobileRepeatedRunCount) : 1;
  const mobileRepeatedDotRadiusPx = Math.max(RUN_DOT_MOBILE_MIN_RADIUS_PX, RUN_DOT_MOBILE_BASE_RADIUS_PX * mobileRepeatedRadiusScale);
  const mobileRepeatedDotDiameterPx = mobileRepeatedDotRadiusPx * 2;
  const [isScenarioMenuOpen, setIsScenarioMenuOpen] = useState(false);
  const scenarioMenuRef = useRef<HTMLDivElement | null>(null);
  const mobileHeaderRef = useRef<HTMLElement | null>(null);
  const [mobileHeaderHeight, setMobileHeaderHeight] = useState(0);

  useEffect(() => {
    const updateHeaderHeight = () => {
      const measuredHeight = mobileHeaderRef.current?.getBoundingClientRect().height ?? 0;
      setMobileHeaderHeight(Math.ceil(measuredHeight));
    };

    updateHeaderHeight();
    window.addEventListener("resize", updateHeaderHeight);
    return () => window.removeEventListener("resize", updateHeaderHeight);
  }, []);

  useEffect(() => {
    if (!isScenarioMenuOpen) return;

    const handleClickOutside = (event: MouseEvent) => {
      if (!scenarioMenuRef.current?.contains(event.target as Node)) {
        setIsScenarioMenuOpen(false);
      }
    };

    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, [isScenarioMenuOpen]);

  const scrollToMobileSection = (id: string) => {
    const target = document.getElementById(id);
    if (!target) return;
    const offset = mobileHeaderHeight + 8;
    const top = target.getBoundingClientRect().top + window.scrollY - offset;
    window.scrollTo({ top: Math.max(0, top), behavior: "smooth" });
  };

  return (
    <div className="md:hidden">
      <header ref={mobileHeaderRef} data-mobile-header="true" className="sticky top-0 z-40 border-b border-slate-800/80 bg-slate-950/90 backdrop-blur">
        <div className="flex items-center justify-between px-4 py-3">
          <button className="text-left text-lg tracking-tight" onClick={() => onScrollTo("hero-mobile")} aria-label="Jump to top of page">
            EdgeSim
          </button>
          <div className="flex flex-wrap justify-end gap-2">
            <Button
              variant="outline"
              className="border-slate-700 bg-transparent px-3 py-1 text-xs text-slate-100"
              onClick={() => scrollToMobileSection("explorer-mobile")}
            >
              Scenarios
            </Button>
            <Button
              variant="outline"
              className="border-slate-700 bg-transparent px-3 py-1 text-xs text-slate-100"
              onClick={() => scrollToMobileSection("workflow-mobile")}
            >
              Workflow
            </Button>
            <ContactMenuButton triggerClassName="px-3 py-1 text-xs" />
          </div>
        </div>
      </header>

      <section
        id="hero-mobile"
        className="relative overflow-hidden border-b border-slate-800"
        style={{ minHeight: mobileHeaderHeight > 0 ? `calc(100svh - ${mobileHeaderHeight}px)` : undefined }}
      >
        <div className="absolute inset-0">
          <img src={heroImage} alt="Warehouse aisle background" className="h-full w-full object-cover opacity-35" />
          <div className="absolute inset-0 bg-gradient-to-b from-slate-900/40 via-slate-950/90 to-slate-950" />
        </div>
        <div className="relative px-4 pb-10 pt-10">
          <h1 className="text-3xl leading-tight">
            Evaluate warehouse robot safety
            <span className="text-cyan-300"> before deployment</span>
          </h1>
          <p className="mt-3 text-sm text-slate-300">
            Build scenarios through text and coordinates, run simulation batches, and generate data of the edge case warehouse scenario.
          </p>
          <div className="mt-5 grid grid-cols-2 gap-3">
            <StatCard compact label="Scenarios" value={String(demoData.global.prompt_count)} icon={<FileText className="h-5 w-5" />} />
            <StatCard compact label="Total runs" value={String(demoData.global.total_runs)} icon={<Database className="h-5 w-5" />} />
            <StatCard
              compact
              label="Collision rate"
              value={`${Math.max(0, 100 - demoData.global.batch_success_rate_pct).toFixed(2)}%`}
              icon={<ShieldCheck className="h-5 w-5" />}
            />
            <StatCard compact label="Sim speed" value={`${demoData.global.avg_sims_per_min.toFixed(2)} sims/min`} icon={<Gauge className="h-5 w-5" />} />
          </div>
        </div>
      </section>

      <section id="explorer-mobile" className="px-4 pb-10 pt-8">
        <div className="mb-4">
          <h2 className="text-2xl">Scenario Explorer</h2>
          <p className="mt-2 text-sm text-slate-300">Inspect one scenario at a time and switch between overview and generated files.</p>
        </div>

        <Card className="border-cyan-500/25 bg-cyan-950/10 p-4">
          <p className="mb-1 text-xs uppercase tracking-wide text-cyan-200/85">Pick a scenario</p>
          <div ref={scenarioMenuRef} className="relative">
            <button
              className="flex w-full items-center justify-between rounded-lg border border-slate-700 bg-slate-950 px-3 py-2 text-left text-sm text-slate-100 transition-colors hover:border-cyan-500/70"
              onClick={() => setIsScenarioMenuOpen((prev) => !prev)}
              aria-expanded={isScenarioMenuOpen}
              aria-haspopup="listbox"
            >
              <span className="truncate pr-2">{activeScenario.title}</span>
              <ChevronDown className={`h-4 w-4 shrink-0 text-slate-400 transition-transform duration-200 ${isScenarioMenuOpen ? "rotate-180" : ""}`} />
            </button>
            <div
              className={`pointer-events-none absolute left-0 right-0 top-[calc(100%+0.45rem)] z-30 origin-top transition-all duration-200 ${
                isScenarioMenuOpen ? "translate-y-0 opacity-100" : "-translate-y-1 opacity-0"
              }`}
            >
              <Card
                className={`border-slate-700 bg-slate-950/95 p-1 shadow-2xl backdrop-blur ${
                  isScenarioMenuOpen ? "pointer-events-auto" : "pointer-events-none"
                }`}
              >
                <div role="listbox" aria-label="Scenario list" className="max-h-60 overflow-y-auto">
                  {demoData.scenarios.map((scenario) => {
                    const isSelected = scenario.id === activeScenario.id;
                    return (
                      <button
                        key={scenario.id}
                        role="option"
                        aria-selected={isSelected}
                        className={`flex w-full items-center justify-between rounded-md px-3 py-2 text-left text-sm transition ${
                          isSelected
                            ? "bg-cyan-500/20 text-cyan-200"
                            : "text-slate-200 hover:bg-slate-800/80"
                        }`}
                        onClick={() => {
                          onPickScenario(scenario.id);
                          setIsScenarioMenuOpen(false);
                        }}
                      >
                        <span className="pr-2">{scenario.title}</span>
                        {isSelected && <Check className="h-4 w-4 shrink-0" />}
                      </button>
                    );
                  })}
                </div>
              </Card>
            </div>
          </div>

          <div className="mt-3 rounded-lg border border-slate-800 bg-slate-900/60 p-3">
            <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Scenario description</p>
            <p className="font-mono text-sm text-slate-100">{activeScenario.prompt}</p>
          </div>

          <div className="mt-4 grid grid-cols-2 gap-3">
            <OutcomeCard
              compact
              title="Example Replay"
              outcome={activeScenario.test.outcome}
              timeValue={`${activeScenario.test.duration_s.toFixed(2)}s`}
              detailA={`Closest near-miss (sec): ${activeScenario.test.min_ttc_s.toFixed(2)}`}
            />
            <OutcomeCard
              compact
              title={`${activeScenario.batch.runs} Repeats`}
              outcome={Math.max(0, 100 - activeScenario.batch.coverage_pct.success) >= 50 ? "collision_dominant" : "low_collision"}
              timeValue={`${activeScenario.batch.avg_time_s.toFixed(2)}s avg`}
              detailA={`${Math.max(0, 100 - activeScenario.batch.coverage_pct.success).toFixed(1)}% collision`}
            />
          </div>

          <div className="mt-4">
            <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Repeated-run outcomes</p>
            <div className="max-h-32 overflow-auto rounded-md border border-slate-800/70 bg-slate-950/40 p-2">
              <div
                className="grid"
                style={{
                  width: "100%",
                  gridAutoFlow: "row",
                  gridTemplateColumns: `repeat(auto-fill, minmax(${mobileRepeatedDotDiameterPx}px, ${mobileRepeatedDotDiameterPx}px))`,
                  gap: `${RUN_DOT_MOBILE_GAP_PX}px`,
                }}
              >
                {activeScenario.batch.success_curve.map((point) => (
                  <span
                    key={point.idx}
                    title={`Run ${point.idx}: ${point.success ? "non-collision" : "collision"} (${point.time_s.toFixed(2)}s)`}
                    className={point.success ? "bg-blue-400/80" : "bg-orange-400/80"}
                    style={{
                      width: `${mobileRepeatedDotDiameterPx}px`,
                      height: `${mobileRepeatedDotDiameterPx}px`,
                      borderRadius: "9999px",
                    }}
                  />
                ))}
              </div>
            </div>
          </div>
        </Card>

        <div className="mt-4 flex flex-wrap gap-2">
          <TabButton label="Overview" active={activeTab === "overview"} onClick={() => onSetTab("overview")} />
          <TabButton label="Generated data" active={activeTab === "artifacts"} onClick={() => onSetTab("artifacts")} />
        </div>

        <div className="mt-4">{activeTab === "overview" ? <MobileOverviewTab scenario={activeScenario} /> : <ArtifactsTab scenario={activeScenario} />}</div>
      </section>

      <section id="workflow-mobile" className="border-y border-slate-800/80 bg-slate-900/20 px-4 pb-10 pt-8">
        <div className="mb-4">
          <h2 className="text-2xl">How EdgeSim works</h2>
          <p className="mt-2 text-sm text-slate-300">Define a scenario, run replay and batch tests, then inspect evidence.</p>
        </div>
        <div className="space-y-3">
          <WorkflowCard
            compact
            title="1. Define the Situation"
            detail="Describe the warehouse situation through text and coordinates. EdgeSim converts it into map layout, moving actors, floor conditions, and sensor settings."
            code="edgesim simulate <scenario_text>"
          />
          <WorkflowCard
            compact
            title="2. Run One Replay"
            detail="Run once to inspect behavior over time and confirm the setup behaves as intended."
            code="edgesim run-one <scenario_text>"
          />
          <WorkflowCard
            compact
            title="3. Reliability Batch"
            detail="Run controlled repeats to measure success rate, collision frequency, near-misses, and completion-time spread."
            code="edgesim run-batch <scenario_text> --runs <n>"
          />
          <WorkflowCard
            compact
            title="4. Evidence Review"
            detail="Use map replay, LiDAR logs, coverage and performance summaries to verify results and use them for future robot training/testing."
            code="summary.json + coverage.json + report.html"
          />
        </div>
      </section>

      <footer className="border-t border-slate-800 bg-slate-950/80">
        <div className="px-4 pb-8 pt-6">
          <div className="space-y-1 text-sm text-slate-500">
            <p>Built by Vlad Kalinin</p>
            <p>
              GitHub Repository:{" "}
              <a href="https://github.com/Vortex-VK/EdgeSim" target="_blank" rel="noreferrer" className="underline-offset-2 hover:underline">
                https://github.com/Vortex-VK/EdgeSim
              </a>
            </p>
          </div>
          <p className="mt-5 text-sm text-slate-500">Data snapshot generated at {demoData.generated_at_utc} UTC from local run evidence files.</p>
        </div>
      </footer>
    </div>
  );
}

function MobileOverviewTab({ scenario }: { scenario: Scenario }) {
  return (
    <div className="space-y-4">
      <MapTab scenario={scenario} compact />

      <div className="grid grid-cols-2 gap-3">
        <StatCard label="Sensor refresh rate" value={`${scenario.config.lidar_hz.toFixed(1)} Hz`} icon={<Gauge className="h-5 w-5" />} />
        <StatCard label="Simulation time step" value={`${scenario.config.dt.toFixed(2)} s`} icon={<Timer className="h-5 w-5" />} />
        <StatCard label="People / Vehicles" value={`${scenario.config.humans} / ${scenario.config.vehicles}`} icon={<CircleAlert className="h-5 w-5" />} />
        <StatCard label="Racks / Barriers" value={`${scenario.config.racks} / ${scenario.config.walls}`} icon={<Map className="h-5 w-5" />} />
        <div className="col-span-2">
          <StatCard label="Simulation speed" value={`${scenario.batch.perf.sims_per_min.toFixed(2)} sims/min`} icon={<Database className="h-5 w-5" />} />
        </div>
      </div>

      <div className="grid gap-4">
        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Completion time spread (seconds)</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Fastest: {scenario.batch.time_distribution_s.min.toFixed(2)}</li>
            <li>10th percentile: {scenario.batch.time_distribution_s.p10.toFixed(2)}</li>
            <li>Median: {scenario.batch.time_distribution_s.p50.toFixed(2)}</li>
            <li>90th percentile: {scenario.batch.time_distribution_s.p90.toFixed(2)}</li>
            <li>Slowest: {scenario.batch.time_distribution_s.max.toFixed(2)}</li>
          </ul>
        </Card>

        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Event counts across repeated runs</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Contacts with people: {scenario.batch.event_counts.collision_human ?? 0}</li>
            <li>Contacts with objects: {scenario.batch.event_counts.collision_static ?? 0}</li>
            <li>Slip events: {scenario.batch.event_counts.slip ?? 0}</li>
            <li>Near-miss events: {scenario.batch.event_counts.near_miss ?? 0}</li>
            <li>Potentially hidden hazards: {scenario.batch.event_counts.occluded_hazard ?? 0}</li>
          </ul>
        </Card>
      </div>

      {!scenario.validation.ok && (
        <div className="rounded-lg border border-rose-600/40 bg-rose-900/20 p-3 text-sm text-rose-200">
          <p className="font-medium">Data validation issues detected</p>
          {scenario.validation.errors.map((error) => (
            <p key={error}>{error}</p>
          ))}
        </div>
      )}
    </div>
  );
}

function OverviewTab({ scenario }: { scenario: Scenario }) {
  return (
    <Card className="border-slate-800 bg-slate-950/70 p-6">
      <MapTab scenario={scenario} />

      <div className="mt-6 grid gap-4 md:grid-cols-2 xl:grid-cols-5">
        <MetricCard label="Sensor refresh rate" value={`${scenario.config.lidar_hz.toFixed(1)} Hz`} icon={<Gauge className="h-4 w-4" />} />
        <MetricCard label="Simulation time step" value={`${scenario.config.dt.toFixed(2)} s`} icon={<Timer className="h-4 w-4" />} />
        <MetricCard label="People / Vehicles" value={`${scenario.config.humans} / ${scenario.config.vehicles}`} icon={<CircleAlert className="h-4 w-4" />} />
        <MetricCard label="Racks / Barriers" value={`${scenario.config.racks} / ${scenario.config.walls}`} icon={<Map className="h-4 w-4" />} />
        <MetricCard label="Simulation speed" value={`${scenario.batch.perf.sims_per_min.toFixed(2)} sims/min`} icon={<Database className="h-4 w-4" />} />
      </div>

      <div className="mt-6 grid gap-4 lg:grid-cols-2">
        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Completion time spread (seconds)</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Fastest: {scenario.batch.time_distribution_s.min.toFixed(2)}</li>
            <li>10th percentile: {scenario.batch.time_distribution_s.p10.toFixed(2)}</li>
            <li>Median: {scenario.batch.time_distribution_s.p50.toFixed(2)}</li>
            <li>90th percentile: {scenario.batch.time_distribution_s.p90.toFixed(2)}</li>
            <li>Slowest: {scenario.batch.time_distribution_s.max.toFixed(2)}</li>
          </ul>
        </Card>

        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Event counts across repeated runs</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Contacts with people: {scenario.batch.event_counts.collision_human ?? 0}</li>
            <li>Contacts with objects: {scenario.batch.event_counts.collision_static ?? 0}</li>
            <li>Slip events: {scenario.batch.event_counts.slip ?? 0}</li>
            <li>Near-miss events: {scenario.batch.event_counts.near_miss ?? 0}</li>
            <li>Potentially hidden hazards: {scenario.batch.event_counts.occluded_hazard ?? 0}</li>
          </ul>
        </Card>
      </div>

      {!scenario.validation.ok && (
        <div className="mt-4 rounded-lg border border-rose-600/40 bg-rose-900/20 p-3 text-sm text-rose-200">
          <p className="font-medium">Data validation issues detected</p>
          {scenario.validation.errors.map((error) => (
            <p key={error}>{error}</p>
          ))}
        </div>
      )}

    </Card>
  );
}

function sampleAtTime(
  samples: Array<{ t: number; x: number; y: number; yaw: number; phase: string }>,
  t: number,
): { t: number; x: number; y: number; yaw: number; phase: string } | null {
  if (samples.length === 0) return null;
  let chosen = samples[0];
  for (const sample of samples) {
    if (sample.t > t) break;
    chosen = sample;
  }
  return chosen;
}

function robotPoseAtTime(scenario: Scenario, activeFrame: LidarPlaybackFrame | null, t: number): { x: number; y: number; yaw: number } {
  if (activeFrame) {
    return { x: activeFrame.pose[0], y: activeFrame.pose[1], yaw: activeFrame.pose[2] };
  }
  const sample = sampleAtTime(
    scenario.test.path.map((point) => ({ t: point.t, x: point.x, y: point.y, yaw: 0, phase: "" })),
    t,
  );
  return sample
    ? { x: sample.x, y: sample.y, yaw: sample.yaw }
    : { x: scenario.world.start[0], y: scenario.world.start[1], yaw: 0 };
}

function actorKind(type: string): "amr" | "human" | "vehicle" | "other" {
  const normalized = type.toLowerCase();
  if (normalized.includes("amr") || normalized.includes("robot")) return "amr";
  if (normalized.includes("human")) return "human";
  if (normalized.includes("vehicle") || normalized.includes("forklift") || normalized.includes("tugger") || normalized.includes("cart")) {
    return "vehicle";
  }
  return "other";
}

type ObjectCategory = "storage" | "structure" | "safety" | "other";

function objectCategory(type: string): ObjectCategory {
  const normalized = String(type ?? "").toLowerCase();
  if (normalized.includes("column") || normalized.includes("bollard") || normalized.includes("cone") || normalized.includes("extinguisher")) {
    return "safety";
  }
  if (normalized.includes("door") || normalized.includes("dock") || normalized.includes("endcap") || normalized.includes("blind")) {
    return "structure";
  }
  if (normalized.includes("storage") || normalized.includes("pallet") || normalized.includes("cart") || normalized.includes("box") || normalized.includes("obstacle")) {
    return "storage";
  }
  return "other";
}

function objectCategoryStyle(category: ObjectCategory): { fill: string; stroke: string; opacity: number } {
  if (category === "storage") return { fill: "#c2410c", stroke: "#9a3412", opacity: 0.82 };
  if (category === "structure") return { fill: "#64748b", stroke: "#475569", opacity: 0.8 };
  if (category === "safety") return { fill: "#0891b2", stroke: "#0e7490", opacity: 0.85 };
  return { fill: "#4b5563", stroke: "#334155", opacity: 0.76 };
}

function orientedRectPoints(
  center: [number, number],
  halfExtents: [number, number],
  yaw: number,
  toSvgY: (y: number) => number,
): string {
  const [cx, cy] = center;
  const [hx, hy] = halfExtents;
  if (![cx, cy, hx, hy, yaw].every((val) => Number.isFinite(val))) return "";
  if (hx <= 0 || hy <= 0) return "";
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const corners: [number, number][] = [
    [-hx, -hy],
    [hx, -hy],
    [hx, hy],
    [-hx, hy],
  ].map(([lx, ly]) => [cx + lx * c - ly * s, cy + lx * s + ly * c]);
  return corners.map(([x, y]) => `${x},${toSvgY(y)}`).join(" ");
}

function MapTab({ scenario, compact = false }: { scenario: Scenario; compact?: boolean }) {
  const lidarFrames = scenario.test.lidar.playback?.frames ?? [];
  const hasFrames = lidarFrames.length > 0;
  const [frameIdx, setFrameIdx] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);

  useEffect(() => {
    setFrameIdx(0);
    setIsPlaying(false);
  }, [scenario.id]);

  useEffect(() => {
    if (!isPlaying || !hasFrames) return;
    const handle = window.setInterval(() => {
      setFrameIdx((prev) => (prev + 1) % lidarFrames.length);
    }, 240);
    return () => window.clearInterval(handle);
  }, [isPlaying, hasFrames, lidarFrames.length]);

  const clampedFrameIdx = hasFrames ? Math.min(frameIdx, lidarFrames.length - 1) : 0;
  const activeFrame = hasFrames ? lidarFrames[clampedFrameIdx] : null;
  const fallbackTime = scenario.test.path[scenario.test.path.length - 1]?.t ?? 0;
  const activeTime = activeFrame ? activeFrame.t : fallbackTime;

  return (
    <Card className={`border-slate-800 bg-slate-950/70 ${compact ? "p-4" : "p-6"}`}>
      <div className={`mb-4 rounded-lg border border-slate-800 bg-slate-900/50 ${compact ? "p-2.5" : "p-3"}`}>
        <p className={`mb-2 uppercase tracking-wide text-slate-400 ${compact ? "text-[10px]" : "text-xs"}`}>Replay controls (map + sensor view)</p>
        {!hasFrames && <p className="text-sm text-slate-400">No sensor replay frames are available for this scenario.</p>}
        {hasFrames && (
          <div className="space-y-2">
            <div className="flex w-full items-center gap-2">
              <Button className={`bg-blue-600 text-white hover:bg-blue-500 ${compact ? "h-8 px-3 text-xs" : ""}`} onClick={() => setIsPlaying((prev) => !prev)}>
                {isPlaying ? "Pause" : "Play"}
              </Button>
              <input
                type="range"
                min={0}
                max={Math.max(0, lidarFrames.length - 1)}
                value={clampedFrameIdx}
                onChange={(event) => setFrameIdx(Number(event.target.value))}
                className="h-2 min-w-0 flex-1 accent-blue-500"
              />
            </div>
            <span className={compact ? "text-xs text-slate-300" : "text-sm text-slate-300"}>
              Frame {clampedFrameIdx + 1}/{lidarFrames.length} at t={activeTime.toFixed(2)}s
            </span>
          </div>
        )}
      </div>

      <div className={compact ? "grid grid-cols-2 gap-3" : "grid gap-5 lg:grid-cols-2"}>
        <MapPlaybackPanel scenario={scenario} activeFrame={activeFrame} activeTime={activeTime} compact={compact} />
        <LidarScanPanel scenario={scenario} activeFrame={activeFrame} compact={compact} />
      </div>
    </Card>
  );
}

function MapPlaybackPanel({
  scenario,
  activeFrame,
  activeTime,
  compact = false,
}: {
  scenario: Scenario;
  activeFrame: LidarPlaybackFrame | null;
  activeTime: number;
  compact?: boolean;
}) {
  const [mapW, mapH] = scenario.world.map_size_m;
  const toSvgY = (y: number) => mapH - y;
  const gridX = Array.from({ length: Math.floor(mapW) + 1 }, (_, idx) => idx);
  const gridY = Array.from({ length: Math.floor(mapH) + 1 }, (_, idx) => idx);
  const robotPathPoints = scenario.test.path.map((point) => `${point.x},${toSvgY(point.y)}`).join(" ");
  const robotTraversedPoints = scenario.test.path.filter((point) => point.t <= activeTime);
  const robotTraversedPath = robotTraversedPoints.map((point) => `${point.x},${toSvgY(point.y)}`).join(" ");

  const robotPose = robotPoseAtTime(scenario, activeFrame, activeTime);
  const headingLength = 0.65;
  const headingEnd = {
    x: robotPose.x + headingLength * Math.cos(robotPose.yaw),
    y: robotPose.y + headingLength * Math.sin(robotPose.yaw),
  };

  const actorSnapshots = scenario.test.actors
    .map((track) => ({ track, pose: sampleAtTime(track.samples, activeTime) }))
    .filter((item) => item.pose !== null);
  const worldObjects = scenario.world.objects ?? [];
  const objectCategories = new Set(worldObjects.map((obj) => objectCategory(obj.type)));
  const hasHumans = scenario.test.actors.some((track) => actorKind(track.type) === "human");
  const hasVehicles = scenario.test.actors.some((track) => actorKind(track.type) === "vehicle");
  const hasWalls = scenario.world.walls.length > 0;
  const hasRacks = scenario.world.racks.length > 0;
  const hasTraction = scenario.world.traction.length > 0;
  const hasStorageObjects = objectCategories.has("storage");
  const hasStructureObjects = objectCategories.has("structure");
  const hasSafetyObjects = objectCategories.has("safety");
  const hasOtherObjects = objectCategories.has("other");
  const patternInstanceId = useId().replace(/:/g, "");
  const wetPatternId = `wetDots-${scenario.id}-${patternInstanceId}`;
  const rackPatternId = `rackTexture-${scenario.id}-${patternInstanceId}`;

  return (
    <div>
      <p className={`mb-2 uppercase tracking-wide text-slate-400 ${compact ? "text-[10px]" : "text-xs"}`}>
        {compact ? "Top-Down Replay" : "Top-down replay (robot, people, vehicles, and obstacles)"}
      </p>
      <Card className={`border-slate-800 bg-slate-900/60 ${compact ? "p-2" : "p-3"}`}>
        <svg viewBox={`-1 -1 ${mapW + 2} ${mapH + 2}`} className="aspect-square w-full rounded bg-slate-950">
          <defs>
            <pattern id={wetPatternId} width="0.6" height="0.6" patternUnits="userSpaceOnUse">
              <circle cx="0.15" cy="0.15" r="0.06" fill="#1d4ed8" opacity="0.45" />
              <circle cx="0.45" cy="0.45" r="0.06" fill="#1d4ed8" opacity="0.45" />
            </pattern>
            <pattern id={rackPatternId} width="0.9" height="0.9" patternUnits="userSpaceOnUse">
              <rect x="0" y="0" width="0.9" height="0.9" fill="#7c3f11" />
              <rect x="0.08" y="0.08" width="0.33" height="0.33" fill="#b7793e" fillOpacity="0.58" />
              <rect x="0.49" y="0.08" width="0.33" height="0.33" fill="#b7793e" fillOpacity="0.58" />
              <rect x="0.08" y="0.49" width="0.33" height="0.33" fill="#b7793e" fillOpacity="0.58" />
              <rect x="0.49" y="0.49" width="0.33" height="0.33" fill="#b7793e" fillOpacity="0.58" />
              <line x1="0.45" y1="0.03" x2="0.45" y2="0.87" stroke="#5f2f0d" strokeWidth="0.03" opacity="0.85" />
              <line x1="0.03" y1="0.45" x2="0.87" y2="0.45" stroke="#5f2f0d" strokeWidth="0.03" opacity="0.85" />
            </pattern>
          </defs>

          <rect x={0} y={0} width={mapW} height={mapH} fill="#020617" stroke="#334155" strokeWidth={0.04} />

          {gridX.map((x) => (
            <line key={`grid-x-${x}`} x1={x} y1={0} x2={x} y2={mapH} stroke="#1e293b" strokeWidth={0.02} />
          ))}
          {gridY.map((yVal) => (
            <line key={`grid-y-${yVal}`} x1={0} y1={toSvgY(yVal)} x2={mapW} y2={toSvgY(yVal)} stroke="#1e293b" strokeWidth={0.02} />
          ))}

          {scenario.world.traction.map((rect, idx) => (
            <rect
              key={`traction-${idx}`}
              x={rect[0]}
              y={toSvgY(rect[3])}
              width={rect[2] - rect[0]}
              height={rect[3] - rect[1]}
              fill={`url(#${wetPatternId})`}
              stroke="#3b82f6"
              strokeWidth={0.03}
              opacity={0.85}
            />
          ))}
          {scenario.world.walls.map((rect, idx) => (
            <rect
              key={`wall-${idx}`}
              x={rect[0]}
              y={toSvgY(rect[3])}
              width={rect[2] - rect[0]}
              height={rect[3] - rect[1]}
              fill="#4b5563"
              stroke="#374151"
              strokeWidth={0.03}
            />
          ))}
          {scenario.world.racks.map((rect, idx) => (
            <rect
              key={`rack-${idx}`}
              x={rect[0]}
              y={toSvgY(rect[3])}
              width={rect[2] - rect[0]}
              height={rect[3] - rect[1]}
              fill={`url(#${rackPatternId})`}
              stroke="#d6a677"
              strokeWidth={0.035}
              strokeDasharray="0.14 0.08"
              opacity={0.95}
            />
          ))}
          {worldObjects.map((obj, idx) => {
            const style = objectCategoryStyle(objectCategory(obj.type));
            const center = obj.center;
            const halfExtents = obj.half_extents;
            const yaw = obj.yaw ?? 0;
            if (center && halfExtents) {
              const points = orientedRectPoints(center, halfExtents, yaw, toSvgY);
              if (!points) return null;
              return (
                <polygon
                  key={`world-object-${idx}`}
                  points={points}
                  fill={style.fill}
                  stroke={style.stroke}
                  strokeWidth={0.03}
                  opacity={style.opacity}
                >
                  <title>{obj.type}</title>
                </polygon>
              );
            }
            const [x0, y0, x1, y1] = obj.aabb;
            return (
              <rect
                key={`world-object-${idx}`}
                x={x0}
                y={toSvgY(y1)}
                width={x1 - x0}
                height={y1 - y0}
                fill={style.fill}
                stroke={style.stroke}
                strokeWidth={0.03}
                opacity={style.opacity}
              >
                <title>{obj.type}</title>
              </rect>
            );
          })}

          {robotPathPoints && (
            <polyline
              points={robotPathPoints}
              fill="none"
              stroke="#ef4444"
              strokeWidth={0.06}
              strokeDasharray="0.25 0.2"
              opacity={0.45}
            />
          )}
          {robotTraversedPath && (
            <polyline
              points={robotTraversedPath}
              fill="none"
              stroke="#b91c1c"
              strokeWidth={0.1}
              opacity={0.9}
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          )}
          {robotTraversedPoints.length === 1 && (
            <circle
              cx={robotTraversedPoints[0].x}
              cy={toSvgY(robotTraversedPoints[0].y)}
              r={0.14}
              fill="#b91c1c"
              opacity={0.95}
            />
          )}

          {actorSnapshots.map(({ track, pose }) => {
            if (!pose) return null;
            const kind = actorKind(track.type);
            if (kind === "amr") return null;

            const fullTrackPoints = track.samples.map((sample) => `${sample.x},${toSvgY(sample.y)}`).join(" ");
            const traversedTrackPoints = track.samples
              .filter((sample) => sample.t <= activeTime)
              .map((sample) => `${sample.x},${toSvgY(sample.y)}`)
              .join(" ");
            const color = kind === "human" ? "#22c55e" : "#f59e0b";
            const stroke = kind === "human" ? "#15803d" : "#b45309";
              const headingLen = kind === "human" ? 0.38 : 0.54;
              const hx = pose.x + headingLen * Math.cos(pose.yaw);
              const hy = pose.y + headingLen * Math.sin(pose.yaw);
              const vehicleYawDeg = (-pose.yaw * 180) / Math.PI;
              const cy = toSvgY(pose.y);

              return (
                <g key={`actor-${track.id}`}>
                {fullTrackPoints && (
                  <polyline
                    points={fullTrackPoints}
                    fill="none"
                    stroke={color}
                    strokeWidth={0.04}
                    opacity={0.3}
                    strokeDasharray="0.18 0.12"
                  />
                )}
                {traversedTrackPoints && (
                  <polyline
                    points={traversedTrackPoints}
                    fill="none"
                    stroke={color}
                    strokeWidth={0.08}
                    opacity={0.85}
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                )}
                <line x1={pose.x} y1={toSvgY(pose.y)} x2={hx} y2={toSvgY(hy)} stroke={stroke} strokeWidth={0.04} opacity={0.95} />
                {kind === "vehicle" ? (
                  <rect
                    x={pose.x - 0.28}
                    y={cy - 0.28}
                    width={0.56}
                    height={0.56}
                    fill={color}
                    stroke={stroke}
                    strokeWidth={0.04}
                    rx={0.02}
                    transform={`rotate(${vehicleYawDeg} ${pose.x} ${cy})`}
                  />
                ) : (
                  <circle cx={pose.x} cy={cy} r={0.22} fill={color} stroke={stroke} strokeWidth={0.04} />
                )}
              </g>
            );
          })}

          <line
            x1={robotPose.x}
            y1={toSvgY(robotPose.y)}
            x2={headingEnd.x}
            y2={toSvgY(headingEnd.y)}
            stroke="#1d4ed8"
            strokeWidth={0.05}
            opacity={0.9}
          />
          <circle cx={robotPose.x} cy={toSvgY(robotPose.y)} r={0.34} fill="#ef4444" stroke="#b91c1c" strokeWidth={0.04} />
        </svg>
      </Card>

      <div className={compact ? "mt-2 grid max-h-28 gap-1 overflow-y-auto pr-1" : "mt-4 grid gap-3 sm:grid-cols-2 xl:grid-cols-3"}>
        <LegendItem glyph="robot" label="Robot location" compact={compact} />
        <LegendItem glyph="robot_planned_path" label="Planned robot path" compact={compact} />
        {hasHumans && <LegendItem glyph="human_track" label="Human path" compact={compact} />}
        {hasVehicles && <LegendItem glyph="vehicle_track" label="Vehicle path" compact={compact} />}
        {hasWalls && <LegendItem glyph="wall" label="Walls" compact={compact} />}
        {hasRacks && <LegendItem glyph="rack" label="Racks" compact={compact} />}
        {hasStorageObjects && <LegendItem glyph="object_storage" label="Storage obstacles" compact={compact} />}
        {hasStructureObjects && <LegendItem glyph="object_structure" label="Structural obstacles" compact={compact} />}
        {hasSafetyObjects && <LegendItem glyph="object_safety" label="Safety markers" compact={compact} />}
        {hasOtherObjects && <LegendItem glyph="object_other" label="Other static obstacles" compact={compact} />}
        {hasTraction && <LegendItem glyph="wet_patch" label="Wet or low-traction floor" compact={compact} />}
      </div>
    </div>
  );
}

function LidarScanPanel({
  scenario,
  activeFrame,
  compact = false,
}: {
  scenario: Scenario;
  activeFrame: LidarPlaybackFrame | null;
  compact?: boolean;
}) {
  const lidarArtifact = scenario.artifacts.test["lidar.npz"];
  if (!activeFrame) {
    return (
      <Card className={`border-slate-800 bg-slate-900/60 ${compact ? "p-2.5" : "p-4"}`}>
        <p className="text-sm text-slate-400">No sensor replay data is available for this run.</p>
      </Card>
    );
  }

  const origin = activeFrame.laser_origin;
  const yaw = activeFrame.pose[2];
  const cy = Math.cos(-yaw);
  const sy = Math.sin(-yaw);
  const localPoints = activeFrame.points.map((point) => {
    const dx = point[0] - origin[0];
    const dy = point[1] - origin[1];
    return {
      x: dx * cy - dy * sy,
      y: dx * sy + dy * cy,
    };
  });
  const maxObserved = localPoints.reduce((acc, point) => Math.max(acc, Math.hypot(point.x, point.y)), 0);
  const configuredRange = scenario.test.lidar.max_range_m ?? 8;
  const viewRange = Math.max(1.5, configuredRange, maxObserved + 0.5);
  const rings = [0.25, 0.5, 0.75, 1].map((f) => viewRange * f);
  const headingTipX = viewRange * 0.22;
  const headingBackX = headingTipX - Math.max(0.12, viewRange * 0.02);
  const headingHalfY = Math.max(0.06, viewRange * 0.012);

  return (
    <div>
      <div className={`mb-2 flex items-center justify-between gap-2 ${compact ? "text-[10px]" : "text-xs"}`}>
        <p className="uppercase tracking-wide text-slate-400">{compact ? "Distance sensor replay" : "Distance sensor replay (rays only)"}</p>
        {!compact && lidarArtifact && (
          <a href={lidarArtifact} target="_blank" rel="noreferrer" className={`${compact ? "text-[10px]" : "text-xs"} text-cyan-300 underline-offset-2 hover:underline`}>
            Open sensor log
          </a>
        )}
      </div>
      <Card className={`border-slate-800 bg-slate-900/60 ${compact ? "p-2" : "p-3"}`}>
        <svg viewBox={`${-viewRange} ${-viewRange} ${viewRange * 2} ${viewRange * 2}`} className="aspect-square w-full rounded bg-slate-950">
          <g transform="scale(1,-1)">
            {rings.map((radius, idx) => (
              <circle key={`ring-${idx}`} cx={0} cy={0} r={radius} fill="none" stroke="#334155" strokeWidth={0.05} opacity={0.7} />
            ))}
            <line x1={-viewRange} y1={0} x2={viewRange} y2={0} stroke="#334155" strokeWidth={0.04} opacity={0.7} />
            <line x1={0} y1={-viewRange} x2={0} y2={viewRange} stroke="#334155" strokeWidth={0.04} opacity={0.7} />

            {localPoints.map((point, idx) => (
              <g key={`scan-${idx}`}>
                <line x1={0} y1={0} x2={point.x} y2={point.y} stroke="#f97316" strokeWidth={0.04} opacity={0.6} />
                <circle cx={point.x} cy={point.y} r={0.08} fill="#ef4444" opacity={0.95} />
              </g>
            ))}

            <line x1={0} y1={0} x2={headingTipX} y2={0} stroke="#60a5fa" strokeWidth={0.06} opacity={0.95} />
            <polygon
              points={`${headingTipX},0 ${headingBackX},${headingHalfY} ${headingBackX},${-headingHalfY}`}
              fill="#60a5fa"
              opacity={0.95}
            />
            <circle cx={0} cy={0} r={0.2} fill="#60a5fa" />
          </g>
        </svg>
      </Card>
      <p className={`mt-2 text-slate-500 ${compact ? "text-[10px]" : "text-xs"}`}>
        This panel shows what the robot "sees" using its LiDAR. The center is the sensors location.
      </p>
    </div>
  );
}

function ArtifactsTab({ scenario }: { scenario: Scenario }) {
  const testEntries = Object.entries(scenario.artifacts.test);
  const batchEntries = Object.entries(scenario.artifacts.batch);

  return (
    <Card className="border-slate-800 bg-slate-950/70 p-6">
      <div className="grid gap-6 lg:grid-cols-2">
        <div>
          <p className="mb-3 text-xs uppercase tracking-wide text-slate-400">Files from example replay</p>
          <div className="space-y-2">
            {testEntries.map(([key, href]) => (
              <ArtifactLink key={key} label={artifactLabels[key] ?? key} href={href} />
            ))}
          </div>
        </div>

        <div>
          <p className="mb-3 text-xs uppercase tracking-wide text-slate-400">Files from repeated-run batch</p>
          <div className="space-y-2">
            {batchEntries.map(([key, href]) => (
              <ArtifactLink key={key} label={artifactLabels[key] ?? key} href={href} />
            ))}
          </div>
        </div>
      </div>
    </Card>
  );
}

function ArtifactLink({ label, href }: { label: string; href: string }) {
  const icon = href.endsWith(".csv") ? <FileCode2 className="h-4 w-4" /> : <FileText className="h-4 w-4" />;
  return (
    <a
      href={href}
      target="_blank"
      rel="noreferrer"
      className="flex items-center justify-between rounded-lg border border-slate-800 bg-slate-900/55 px-3 py-2 text-sm text-slate-100 transition hover:border-cyan-500/50 hover:bg-slate-900"
    >
      <span className="flex items-center gap-2">
        {icon}
        {label}
      </span>
      <span className="flex items-center gap-1 text-cyan-300">
        <Download className="h-4 w-4" />
        View
      </span>
    </a>
  );
}

function MetricCard({ label, value, icon }: { label: string; value: string; icon: ReactNode }) {
  return (
    <Card className="border-slate-800 bg-slate-900/60 p-4">
      <div className="mb-2 flex items-center gap-2 text-cyan-300">
        {icon}
        <span className="text-xs uppercase tracking-wide">{label}</span>
      </div>
      <p className="text-lg text-white">{value}</p>
    </Card>
  );
}

type LegendGlyph =
  | "robot"
  | "robot_planned_path"
  | "human_track"
  | "vehicle_track"
  | "object_storage"
  | "object_structure"
  | "object_safety"
  | "object_other"
  | "wet_patch"
  | "wall"
  | "rack";

function LegendItem({ glyph, label, compact = false }: { glyph: LegendGlyph; label: string; compact?: boolean }) {
  const icon = (() => {
    if (glyph === "robot") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <circle cx="8" cy="8" r="3.8" fill="#ef4444" stroke="#b91c1c" strokeWidth="1.3" />
        </svg>
      );
    }
    if (glyph === "robot_planned_path") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <line x1="2" y1="8" x2="26" y2="8" stroke="#ef4444" strokeWidth="1.2" strokeDasharray="2 1.4" opacity="0.65" />
          <circle cx="21" cy="8" r="2.8" fill="#ef4444" stroke="#b91c1c" strokeWidth="1.1" />
        </svg>
      );
    }
    if (glyph === "human_track") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <line x1="2" y1="8" x2="26" y2="8" stroke="#22c55e" strokeWidth="1.2" strokeDasharray="2 1.4" opacity="0.65" />
          <circle cx="21" cy="8" r="2.2" fill="#22c55e" stroke="#15803d" strokeWidth="1.1" />
        </svg>
      );
    }
    if (glyph === "vehicle_track") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <line x1="2" y1="8" x2="26" y2="8" stroke="#f59e0b" strokeWidth="1.2" strokeDasharray="2 1.4" opacity="0.65" />
          <rect x="18.5" y="5.3" width="6" height="5.4" rx="1" fill="#f59e0b" stroke="#b45309" strokeWidth="1.1" />
        </svg>
      );
    }
    if (glyph === "wet_patch") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <rect x="3" y="3" width="22" height="10" rx="1.5" fill="#bfdbfe" opacity="0.7" stroke="#3b82f6" strokeWidth="1" />
          <circle cx="9" cy="8" r="1" fill="#1d4ed8" opacity="0.65" />
          <circle cx="14" cy="8" r="1" fill="#1d4ed8" opacity="0.65" />
          <circle cx="19" cy="8" r="1" fill="#1d4ed8" opacity="0.65" />
        </svg>
      );
    }
    if (glyph === "object_storage") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <rect x="10" y="4" width="8" height="8" fill="#c2410c" stroke="#9a3412" strokeWidth="1" opacity="0.85" />
        </svg>
      );
    }
    if (glyph === "object_structure") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <rect x="3" y="4" width="22" height="8" rx="1" fill="#64748b" stroke="#475569" strokeWidth="1" opacity="0.85" />
        </svg>
      );
    }
    if (glyph === "object_safety") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <circle cx="14" cy="8" r="4.1" fill="#0891b2" stroke="#0e7490" strokeWidth="1" opacity="0.9" />
        </svg>
      );
    }
    if (glyph === "object_other") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <rect x="4.5" y="4.5" width="19" height="7" rx="1" fill="#4b5563" stroke="#334155" strokeWidth="1" opacity="0.82" />
        </svg>
      );
    }
    if (glyph === "wall") {
      return (
        <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
          <rect x="2.5" y="5.2" width="23" height="5.6" rx="0.8" fill="#4b5563" stroke="#374151" strokeWidth="1" />
        </svg>
      );
    }
    return (
      <svg viewBox="0 0 28 16" className="h-4 w-7 shrink-0">
        <rect x="3" y="3" width="22" height="10" rx="1" fill="#7c3f11" stroke="#d6a677" strokeWidth="1" strokeDasharray="1.6 1" />
        <rect x="4.4" y="4.2" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="9.5" y="4.2" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="14.6" y="4.2" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="19.7" y="4.2" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="4.4" y="9.0" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="9.5" y="9.0" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="14.6" y="9.0" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <rect x="19.7" y="9.0" width="4.2" height="2.5" fill="#b7793e" fillOpacity="0.68" />
        <line x1="14" y1="3.3" x2="14" y2="12.7" stroke="#5f2f0d" strokeWidth="0.7" opacity="0.85" />
        <line x1="3.3" y1="8" x2="24.7" y2="8" stroke="#5f2f0d" strokeWidth="0.7" opacity="0.85" />
      </svg>
    );
  })();

  return (
    <div
      className={`inline-flex items-center rounded-full border border-slate-700/80 bg-slate-950/60 text-slate-200 ${
        compact ? "min-h-7 gap-1 px-2 py-1 text-[10px] [&_svg]:h-3 [&_svg]:w-5" : "min-h-9 gap-2 px-3 py-1.5 text-xs"
      }`}
    >
      {icon}
      {label}
    </div>
  );
}

export default App;
