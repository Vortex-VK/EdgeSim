import { useEffect, useMemo, useState, type ReactNode } from "react";
import {
  ArrowRight,
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
import heroImage from "../assets/a9237e4d5d4f6425eeda2fc91d3834f1a381c233.png";
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
type ExplorerTab = "overview" | "map" | "artifacts";
type LidarPlaybackFrame = Scenario["test"]["lidar"]["playback"]["frames"][number];

const demoData = demoDataRaw as DemoPayload;

const artifactLabels: Record<string, string> = {
  "scenario.yaml": "Scenario YAML",
  "world.json": "World Geometry JSON",
  "run_one.csv": "Single-Run Time Series CSV",
  "lidar.npz": "LiDAR Scan Log (NPZ)",
  "actors.csv": "Actor Trace CSV",
  "validation.json": "Validation Summary JSON",
  "dataset_manifest.json": "Dataset Manifest JSON",
  "report.html": "HTML Report",
  "report.md": "Markdown Report",
  "summary.json": "Batch Summary JSON",
  "coverage.json": "Coverage JSON",
  "perf.json": "Performance JSON",
  "per_run_index.csv": "Per-Run Batch Index CSV",
};

function isCollisionCategory(outcome: string): boolean {
  return outcome === "collision_human" || outcome === "collision_static" || outcome === "collision_dominant";
}

function outcomeLabel(outcome: string): string {
  if (outcome === "mission_success") return "success";
  if (outcome === "collision_human") return "collision (human)";
  if (outcome === "collision_static") return "collision (static)";
  if (outcome === "arrival_dominant" || outcome === "success_dominant" || outcome === "low_collision") return "low collision mix";
  if (outcome === "contact_dominant" || outcome === "collision_dominant") return "collision-dominant mix";
  return outcome;
}

function App() {
  const [activeScenarioId, setActiveScenarioId] = useState(demoData.scenarios[0]?.id ?? "");
  const [activeTab, setActiveTab] = useState<ExplorerTab>("overview");

  const activeScenario = useMemo(
    () => demoData.scenarios.find((scenario) => scenario.id === activeScenarioId) ?? demoData.scenarios[0],
    [activeScenarioId],
  );

  if (!activeScenario) {
    return <div className="min-h-screen bg-slate-950 text-slate-100 p-10">No demo scenarios found.</div>;
  }

  const scrollTo = (id: string) => {
    document.getElementById(id)?.scrollIntoView({ behavior: "smooth", block: "start" });
  };

  return (
    <div
      className="min-h-screen bg-slate-950 text-slate-100"
      style={{ fontFamily: '"Sora", "Avenir Next", "Segoe UI", sans-serif' }}
    >
      <header className="sticky top-0 z-40 border-b border-slate-800/80 bg-slate-950/85 backdrop-blur">
        <div className="mx-auto flex max-w-7xl items-center justify-between px-6 py-4">
          <button
            className="text-left"
            onClick={() => scrollTo("hero")}
            aria-label="Jump to top of page"
          >
            <div className="text-xl tracking-tight">EdgeSim Demo</div>
            <div className="text-xs text-slate-400">Premade runs, no live simulation required</div>
          </button>
          <div className="hidden gap-3 sm:flex">
            <Button variant="outline" className="border-slate-700 bg-transparent text-slate-100" onClick={() => scrollTo("workflow")}>
              Workflow
            </Button>
            <Button variant="outline" className="border-slate-700 bg-transparent text-slate-100" onClick={() => scrollTo("explorer")}>
              Scenario Explorer
            </Button>
            <Button className="bg-cyan-600 text-white hover:bg-cyan-500" onClick={() => scrollTo("comparison")}>
              Compare Prompts
            </Button>
          </div>
        </div>
      </header>

      <section id="hero" className="relative overflow-hidden border-b border-slate-800">
        <div className="absolute inset-0">
          <img src={heroImage} alt="Warehouse aisle background" className="h-full w-full object-cover opacity-30" />
          <div className="absolute inset-0 bg-gradient-to-b from-slate-900/40 via-slate-950/90 to-slate-950" />
        </div>

        <div className="relative mx-auto max-w-7xl px-6 pb-20 pt-16 sm:pt-24">
          <div className="max-w-3xl">
            <p className="mb-3 text-sm uppercase tracking-[0.2em] text-cyan-300">EdgeSim Public Demo</p>
            <h1 className="mb-4 text-4xl leading-tight sm:text-6xl">
              Understand the simulator through
              <span className="text-cyan-300"> real run results</span>
            </h1>
            <p className="max-w-2xl text-lg text-slate-300">
              This site is driven by 4 prompt scenarios, each with 1 test run and 1 batch run (100 seeds). It explains what EdgeSim does, shows
              what changed between scenarios, and links to raw artifacts so viewers can verify every claim.
            </p>

            <div className="mt-8 flex flex-wrap gap-3">
              <Button className="bg-cyan-600 text-white hover:bg-cyan-500" onClick={() => scrollTo("explorer")}>
                Open Scenario Explorer
                <ArrowRight className="ml-2 h-4 w-4" />
              </Button>
              <Button variant="outline" className="border-slate-600 bg-slate-900/30 text-slate-100" onClick={() => scrollTo("workflow")}>
                How the pipeline works
              </Button>
            </div>
          </div>

          <div className="mt-12 grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
            <StatCard label="Prompt scenarios" value={String(demoData.global.prompt_count)} icon={<FileText className="h-5 w-5" />} />
            <StatCard label="Total recorded runs" value={String(demoData.global.total_runs)} icon={<Database className="h-5 w-5" />} />
            <StatCard
              label="Batch collision rate"
              value={`${Math.max(0, 100 - demoData.global.batch_success_rate_pct).toFixed(2)}%`}
              icon={<ShieldCheck className="h-5 w-5" />}
            />
            <StatCard label="Average batch runtime" value={`${demoData.global.avg_batch_time_s.toFixed(2)}s`} icon={<Timer className="h-5 w-5" />} />
          </div>
        </div>
      </section>

      <section id="workflow" className="mx-auto max-w-7xl px-6 py-16">
        <div className="mb-8 max-w-2xl">
          <h2 className="text-3xl sm:text-4xl">How a viewer should read this demo</h2>
          <p className="mt-3 text-slate-300">
            The same prompt is executed twice: first as a single-run sanity test, then as a seeded 100-run batch. This lets you compare one
            trajectory against distribution-level behavior.
          </p>
        </div>
        <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
          <WorkflowCard
            title="1. Prompt To Scenario"
            detail="Natural language is parsed into geometry, actors, traction zones, and sensor settings."
            code="edgesim simulate <prompt>"
          />
          <WorkflowCard
            title="2. Test Run"
            detail="One rollout validates geometry and interaction logic, with frame-level telemetry in run_one.csv."
            code="edgesim run-one <prompt>"
          />
          <WorkflowCard
            title="3. Batch Run"
            detail="100 seeded runs produce outcome distributions, coverage stats, and performance metrics."
            code="edgesim run-batch <prompt> --runs 100"
          />
          <WorkflowCard
            title="4. Review Evidence"
            detail="Use map overlays, event counts, and downloadable artifacts to verify behavior."
            code="summary.json + coverage.json + report.html"
          />
        </div>
      </section>

      <section id="explorer" className="border-y border-slate-800 bg-slate-900/30">
        <div className="mx-auto max-w-7xl px-6 py-16">
          <div className="mb-8 max-w-3xl">
            <h2 className="text-3xl sm:text-4xl">Scenario Explorer</h2>
            <p className="mt-3 text-slate-300">
              Pick any of the 4 prompt pairs. The panel updates with verified test/batch metrics, trajectory map, and direct artifact links.
            </p>
          </div>

          <div className="grid gap-6 lg:grid-cols-[330px_1fr]">
            <Card className="border-slate-800 bg-slate-950/70 p-3">
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
                      <div className="mb-2 flex items-center justify-between gap-3">
                        <p className="text-sm uppercase tracking-wide text-slate-400">{scenario.timestamps.test}</p>
                        <span className="rounded-full bg-orange-500/20 px-2 py-0.5 text-xs text-orange-200">
                          {collisionRate.toFixed(1)}% collision
                        </span>
                      </div>
                      <h3 className="mb-1 text-lg leading-tight text-white">{scenario.title}</h3>
                      <p className="text-sm text-slate-300">{scenario.summary}</p>
                    </button>
                  );
                })}
              </div>
            </Card>

            <div className="space-y-4">
              <Card className="border-slate-800 bg-slate-950/80 p-6">
                <div className="mb-4 flex flex-wrap items-center justify-between gap-4">
                  <div>
                    <h3 className="text-2xl text-white">{activeScenario.title}</h3>
                    <p className="mt-1 text-sm text-slate-400">Test run: {activeScenario.timestamps.test} | Batch run: {activeScenario.timestamps.batch}</p>
                  </div>
                  <div className="flex items-center gap-2 rounded-full border border-slate-700 bg-slate-900/70 px-3 py-1 text-sm text-slate-300">
                    <Gauge className="h-4 w-4 text-cyan-300" />
                    Profile: <span className="text-cyan-200">{activeScenario.batch.perf.profile}</span>
                  </div>
                </div>

                <div className="rounded-lg border border-slate-800 bg-slate-900/60 p-4">
                  <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Prompt</p>
                  <p className="font-mono text-sm text-slate-100">{activeScenario.prompt}</p>
                </div>

                <div className="mt-3 rounded-lg border border-slate-800 bg-slate-900/45 p-4 text-sm text-slate-300">
                  <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Interpretation guide</p>
                  <p>Single test run: one concrete trajectory to inspect.</p>
                  <p>100-run batch: outcome distribution under different seeds.</p>
                  <p>Blue marks success cells, orange/red marks collision cells.</p>
                  <p>Artifacts tab: raw files behind every number shown here.</p>
                </div>

                <div className="mt-4 grid gap-4 md:grid-cols-2">
                  <OutcomeCard
                    title="Single Test Run"
                    outcome={activeScenario.test.outcome}
                    timeValue={`${activeScenario.test.duration_s.toFixed(2)}s`}
                    detailA={`Min TTC ${activeScenario.test.min_ttc_s.toFixed(2)}s`}
                    detailB={`${activeScenario.test.events.collisions ?? 0} total contact events`}
                  />
                  <OutcomeCard
                    title="100-Run Batch"
                    outcome={Math.max(0, 100 - activeScenario.batch.coverage_pct.success) >= 50 ? "collision_dominant" : "low_collision"}
                    timeValue={`${activeScenario.batch.avg_time_s.toFixed(2)}s avg`}
                    detailA={`${Math.max(0, 100 - activeScenario.batch.coverage_pct.success).toFixed(1)}% collision / non-success`}
                    detailB={`${activeScenario.batch.successes}/${activeScenario.batch.runs} success`}
                  />
                </div>

                <div className="mt-5">
                  <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">
                    Batch Run Collision View (left to right = run 1..100)
                  </p>
                  <div className="mb-2 flex flex-wrap gap-2 text-xs">
                    <span className="rounded-full bg-blue-500/20 px-2 py-0.5 text-blue-200">Success</span>
                    <span className="rounded-full bg-orange-500/20 px-2 py-0.5 text-orange-200">Collision / non-success</span>
                  </div>
                  <div className="grid grid-cols-20 gap-1">
                    {activeScenario.batch.success_curve.map((point) => (
                      <span
                        key={point.idx}
                        title={`Run ${point.idx}: ${point.success ? "non-collision" : "collision / non-success"} (${point.time_s.toFixed(2)}s)`}
                        className={`h-2 rounded-sm ${point.success ? "bg-blue-400/85" : "bg-orange-500/95"}`}
                      />
                    ))}
                  </div>
                </div>
              </Card>

              <div className="flex flex-wrap gap-2">
                <TabButton label="Overview" active={activeTab === "overview"} onClick={() => setActiveTab("overview")} />
                <TabButton label="Map + Timeline" active={activeTab === "map"} onClick={() => setActiveTab("map")} />
                <TabButton label="Artifacts" active={activeTab === "artifacts"} onClick={() => setActiveTab("artifacts")} />
              </div>

              {activeTab === "overview" && <OverviewTab scenario={activeScenario} />}
              {activeTab === "map" && <MapTab scenario={activeScenario} />}
              {activeTab === "artifacts" && <ArtifactsTab scenario={activeScenario} />}
            </div>
          </div>
        </div>
      </section>

      <section id="comparison" className="mx-auto max-w-7xl px-6 py-16">
        <div className="mb-6 max-w-3xl">
          <h2 className="text-3xl sm:text-4xl">Prompt-to-Prompt Comparison</h2>
          <p className="mt-3 text-slate-300">
            This view helps first-time viewers see what actually changes when prompt geometry and actor traffic change.
          </p>
        </div>

        <Card className="overflow-hidden border-slate-800 bg-slate-900/50">
          <div className="overflow-x-auto">
            <table className="min-w-full text-left text-sm">
              <thead className="bg-slate-900/80 text-slate-300">
                <tr>
                  <th className="px-4 py-3">Scenario</th>
                  <th className="px-4 py-3">Test Outcome</th>
                  <th className="px-4 py-3">Batch Collision</th>
                  <th className="px-4 py-3">Human Contact</th>
                  <th className="px-4 py-3">Wet Encounter</th>
                  <th className="px-4 py-3">Avg Time</th>
                </tr>
              </thead>
              <tbody>
                {demoData.scenarios.map((scenario) => (
                  <tr key={scenario.id} className="border-t border-slate-800 text-slate-200">
                    <td className="px-4 py-3">
                      <p className="text-white">{scenario.title}</p>
                      <p className="text-xs text-slate-400">{scenario.summary}</p>
                    </td>
                    <td className="px-4 py-3">{outcomeLabel(scenario.test.outcome)}</td>
                    <td className="px-4 py-3">{Math.max(0, 100 - scenario.batch.coverage_pct.success).toFixed(1)}%</td>
                    <td className="px-4 py-3">{scenario.batch.coverage_pct.collision_human.toFixed(1)}%</td>
                    <td className="px-4 py-3">{scenario.batch.coverage_pct.wet.toFixed(1)}%</td>
                    <td className="px-4 py-3">{scenario.batch.avg_time_s.toFixed(2)}s</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Card>

        <div className="mt-6 grid gap-4 md:grid-cols-3">
          <Card className="border-slate-800 bg-slate-900/40 p-5">
            <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Hardest Prompt</p>
            <p className="text-lg text-slate-200">{demoData.global.hardest_prompt ?? "n/a"}</p>
          </Card>
          <Card className="border-slate-800 bg-slate-900/40 p-5">
            <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Easiest Prompt</p>
            <p className="text-lg text-cyan-300">{demoData.global.easiest_prompt ?? "n/a"}</p>
          </Card>
          <Card className="border-slate-800 bg-slate-900/40 p-5">
            <p className="mb-1 text-xs uppercase tracking-wide text-slate-400">Mean Throughput</p>
            <p className="text-lg text-cyan-300">{demoData.global.avg_sims_per_min.toFixed(2)} sims/min</p>
          </Card>
        </div>
      </section>

      <footer className="border-t border-slate-800 bg-slate-950/80">
        <div className="mx-auto max-w-7xl px-6 py-10">
          <h3 className="text-xl text-white">What this demo proves</h3>
          <p className="mt-2 max-w-3xl text-slate-300">
            EdgeSim is an environment and stress-test data generator, not a motion planner. These four prompt pairs show how scenario structure
            directly changes safety outcomes, timing distributions, and coverage statistics.
          </p>
          <p className="mt-4 text-sm text-slate-500">Data generated at {demoData.generated_at_utc} UTC from local run artifacts.</p>
        </div>
      </footer>
    </div>
  );
}

function StatCard({ label, value, icon }: { label: string; value: string; icon: ReactNode }) {
  return (
    <Card className="border-slate-800 bg-slate-900/70 p-4">
      <div className="mb-3 flex h-9 w-9 items-center justify-center rounded-lg bg-cyan-500/15 text-cyan-300">{icon}</div>
      <p className="text-2xl text-white">{value}</p>
      <p className="text-sm text-slate-400">{label}</p>
    </Card>
  );
}

function WorkflowCard({ title, detail, code }: { title: string; detail: string; code: string }) {
  return (
    <Card className="border-slate-800 bg-slate-900/45 p-5">
      <h3 className="text-lg text-white">{title}</h3>
      <p className="mt-2 text-sm text-slate-300">{detail}</p>
      <p className="mt-4 rounded bg-slate-950/80 px-3 py-2 font-mono text-xs text-cyan-200">{code}</p>
    </Card>
  );
}

function OutcomeCard({
  title,
  outcome,
  timeValue,
  detailA,
  detailB,
}: {
  title: string;
  outcome: string;
  timeValue: string;
  detailA: string;
  detailB: string;
}) {
  const isCollision = isCollisionCategory(outcome);
  return (
    <Card className={`border p-4 ${isCollision ? "border-orange-500/40 bg-orange-900/10" : "border-blue-500/40 bg-blue-900/10"}`}>
      <div className="mb-3 flex items-center justify-between">
        <p className="text-sm uppercase tracking-wide text-slate-300">{title}</p>
        <span
          className={`rounded-full px-2 py-0.5 text-xs ${
            isCollision ? "bg-orange-500/20 text-orange-200" : "bg-blue-500/20 text-blue-200"
          }`}
        >
          {isCollision ? "collision" : "success"}
        </span>
      </div>
      <p className="text-lg text-white">{outcomeLabel(outcome)}</p>
      <p className="text-sm text-slate-300">{timeValue}</p>
      <p className="mt-2 text-xs text-slate-400">{detailA}</p>
      <p className="text-xs text-slate-400">{detailB}</p>
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

function OverviewTab({ scenario }: { scenario: Scenario }) {
  return (
    <Card className="border-slate-800 bg-slate-950/70 p-6">
      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        <MetricCard label="LiDAR Rate" value={`${scenario.config.lidar_hz.toFixed(1)} Hz`} icon={<Gauge className="h-4 w-4" />} />
        <MetricCard label="Sim Step" value={`${scenario.config.dt.toFixed(2)} s`} icon={<Timer className="h-4 w-4" />} />
        <MetricCard label="Humans / Vehicles" value={`${scenario.config.humans} / ${scenario.config.vehicles}`} icon={<CircleAlert className="h-4 w-4" />} />
        <MetricCard label="Racks / Walls" value={`${scenario.config.racks} / ${scenario.config.walls}`} icon={<Map className="h-4 w-4" />} />
      </div>

      <div className="mt-6 grid gap-4 lg:grid-cols-2">
        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Batch time distribution (seconds)</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Min: {scenario.batch.time_distribution_s.min.toFixed(2)}</li>
            <li>P10: {scenario.batch.time_distribution_s.p10.toFixed(2)}</li>
            <li>Median: {scenario.batch.time_distribution_s.p50.toFixed(2)}</li>
            <li>P90: {scenario.batch.time_distribution_s.p90.toFixed(2)}</li>
            <li>Max: {scenario.batch.time_distribution_s.max.toFixed(2)}</li>
          </ul>
        </Card>

        <Card className="border-slate-800 bg-slate-900/60 p-4">
          <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Batch event totals</p>
          <ul className="space-y-1 text-sm text-slate-200">
            <li>Human contacts: {scenario.batch.event_counts.collision_human ?? 0}</li>
            <li>Static contacts: {scenario.batch.event_counts.collision_static ?? 0}</li>
            <li>Slips: {scenario.batch.event_counts.slip ?? 0}</li>
            <li>Near misses: {scenario.batch.event_counts.near_miss ?? 0}</li>
            <li>Occluded hazards: {scenario.batch.event_counts.occluded_hazard ?? 0}</li>
          </ul>
        </Card>
      </div>

      {!scenario.validation.ok && (
        <div className="mt-4 rounded-lg border border-rose-600/40 bg-rose-900/20 p-3 text-sm text-rose-200">
          <p className="font-medium">Validation errors detected</p>
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

function MapTab({ scenario }: { scenario: Scenario }) {
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
  const activeTime = activeFrame ? activeFrame.t : scenario.test.path[Math.min(frameIdx, Math.max(0, scenario.test.path.length - 1))]?.t ?? 0;

  return (
    <Card className="border-slate-800 bg-slate-950/70 p-6">
      <div className="mb-4 rounded-lg border border-slate-800 bg-slate-900/50 p-3">
        <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">Frame playback controls (map + lidar)</p>
        {!hasFrames && <p className="text-sm text-slate-400">No LiDAR playback frames available for this scenario.</p>}
        {hasFrames && (
          <div className="flex flex-wrap items-center gap-2">
            <Button
              variant="outline"
              className="border-slate-700 bg-slate-950 text-slate-100"
              onClick={() => setFrameIdx((prev) => (prev - 1 + lidarFrames.length) % lidarFrames.length)}
            >
              Prev
            </Button>
            <Button className="bg-blue-600 text-white hover:bg-blue-500" onClick={() => setIsPlaying((prev) => !prev)}>
              {isPlaying ? "Pause" : "Play"}
            </Button>
            <Button
              variant="outline"
              className="border-slate-700 bg-slate-950 text-slate-100"
              onClick={() => setFrameIdx((prev) => (prev + 1) % lidarFrames.length)}
            >
              Next
            </Button>
            <input
              type="range"
              min={0}
              max={Math.max(0, lidarFrames.length - 1)}
              value={clampedFrameIdx}
              onChange={(event) => setFrameIdx(Number(event.target.value))}
              className="h-2 min-w-[220px] flex-1 accent-blue-500"
            />
            <span className="text-sm text-slate-300">
              Frame {clampedFrameIdx + 1}/{lidarFrames.length} @ t={activeTime.toFixed(2)}s
            </span>
          </div>
        )}
      </div>

      <div className="grid gap-5 lg:grid-cols-2">
        <MapPlaybackPanel scenario={scenario} activeFrame={activeFrame} activeTime={activeTime} />
        <LidarScanPanel scenario={scenario} activeFrame={activeFrame} />
      </div>
    </Card>
  );
}

function MapPlaybackPanel({
  scenario,
  activeFrame,
  activeTime,
}: {
  scenario: Scenario;
  activeFrame: LidarPlaybackFrame | null;
  activeTime: number;
}) {
  const [mapW, mapH] = scenario.world.map_size_m;
  const toSvgY = (y: number) => mapH - y;
  const gridX = Array.from({ length: Math.floor(mapW) + 1 }, (_, idx) => idx);
  const gridY = Array.from({ length: Math.floor(mapH) + 1 }, (_, idx) => idx);
  const fullPath = scenario.test.path.map((point) => `${point.x},${toSvgY(point.y)}`).join(" ");
  const traversedPath = scenario.test.path
    .filter((point) => point.t <= activeTime)
    .map((point) => `${point.x},${toSvgY(point.y)}`)
    .join(" ");

  const robotPose = robotPoseAtTime(scenario, activeFrame, activeTime);
  const headingLength = 0.65;
  const headingEnd = {
    x: robotPose.x + headingLength * Math.cos(robotPose.yaw),
    y: robotPose.y + headingLength * Math.sin(robotPose.yaw),
  };

  const actorSnapshots = scenario.test.actors
    .map((track) => ({ track, pose: sampleAtTime(track.samples, activeTime) }))
    .filter((item) => item.pose !== null);

  return (
    <div>
      <p className="mb-3 text-xs uppercase tracking-wide text-slate-400">Top-down map frame-by-frame (with moving actors)</p>
      <Card className="border-slate-800 bg-slate-900/60 p-3">
        <svg viewBox={`-1 -1 ${mapW + 2} ${mapH + 2}`} className="aspect-square w-full rounded bg-slate-100">
          <defs>
            <pattern id={`wetDots-${scenario.id}`} width="0.6" height="0.6" patternUnits="userSpaceOnUse">
              <circle cx="0.15" cy="0.15" r="0.06" fill="#1d4ed8" opacity="0.45" />
              <circle cx="0.45" cy="0.45" r="0.06" fill="#1d4ed8" opacity="0.45" />
            </pattern>
          </defs>

          <rect x={0} y={0} width={mapW} height={mapH} fill="#f8fafc" stroke="#cbd5e1" strokeWidth={0.04} />

          {gridX.map((x) => (
            <line key={`grid-x-${x}`} x1={x} y1={0} x2={x} y2={mapH} stroke="#e2e8f0" strokeWidth={0.02} />
          ))}
          {gridY.map((yVal) => (
            <line key={`grid-y-${yVal}`} x1={0} y1={toSvgY(yVal)} x2={mapW} y2={toSvgY(yVal)} stroke="#e2e8f0" strokeWidth={0.02} />
          ))}

          {scenario.world.aisles.map((rect, idx) => (
            <rect
              key={`aisle-${idx}`}
              x={rect[0]}
              y={toSvgY(rect[3])}
              width={rect[2] - rect[0]}
              height={rect[3] - rect[1]}
              fill="#bfdbfe"
              opacity={0.25}
            />
          ))}
          {scenario.world.traction.map((rect, idx) => (
            <rect
              key={`traction-${idx}`}
              x={rect[0]}
              y={toSvgY(rect[3])}
              width={rect[2] - rect[0]}
              height={rect[3] - rect[1]}
              fill={`url(#wetDots-${scenario.id})`}
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
              fill="#92400e"
              fillOpacity={0.72}
              stroke="#78350f"
              strokeWidth={0.03}
            />
          ))}

          <line
            x1={scenario.world.start[0]}
            y1={toSvgY(scenario.world.start[1])}
            x2={scenario.world.goal[0]}
            y2={toSvgY(scenario.world.goal[1])}
            stroke="#ef4444"
            strokeWidth={0.05}
            strokeDasharray="0.25 0.2"
            opacity={0.45}
          />

          {fullPath && <polyline points={fullPath} fill="none" stroke="#7dd3fc" strokeWidth={0.07} opacity={0.45} />}
          {traversedPath && <polyline points={traversedPath} fill="none" stroke="#0284c7" strokeWidth={0.13} opacity={0.95} />}

          {actorSnapshots.map(({ track, pose }) => {
            if (!pose) return null;
            const kind = actorKind(track.type);
            if (kind === "amr") return null;

            const trail = track.samples.filter((sample) => sample.t <= activeTime).slice(-15);
            const trailPoints = trail.map((sample) => `${sample.x},${toSvgY(sample.y)}`).join(" ");
            const color = kind === "human" ? "#22c55e" : "#f59e0b";
            const stroke = kind === "human" ? "#15803d" : "#b45309";
            const headingLen = kind === "human" ? 0.38 : 0.54;
            const hx = pose.x + headingLen * Math.cos(pose.yaw);
            const hy = pose.y + headingLen * Math.sin(pose.yaw);

            return (
              <g key={`actor-${track.id}`}>
                {trailPoints && (
                  <polyline points={trailPoints} fill="none" stroke={color} strokeWidth={0.05} opacity={0.35} strokeDasharray="0.18 0.12" />
                )}
                <line x1={pose.x} y1={toSvgY(pose.y)} x2={hx} y2={toSvgY(hy)} stroke={stroke} strokeWidth={0.04} opacity={0.95} />
                <circle cx={pose.x} cy={toSvgY(pose.y)} r={kind === "human" ? 0.22 : 0.28} fill={color} stroke={stroke} strokeWidth={0.04} />
              </g>
            );
          })}

          {scenario.test.timeline.map((moment, idx) => (
            <circle key={`event-${idx}`} cx={moment.x} cy={toSvgY(moment.y)} r={0.15} fill="#e11d48" />
          ))}

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

      <div className="mt-3 grid gap-2 sm:grid-cols-2 xl:grid-cols-6">
        <LegendItem color="bg-red-500" label="Robot position" />
        <LegendItem color="bg-sky-500" label="Robot path (to frame)" />
        <LegendItem color="bg-green-500" label="Human dynamic track" />
        <LegendItem color="bg-amber-500" label="Vehicle dynamic track" />
        <LegendItem color="bg-blue-500" label="Wet/traction patches" />
        <LegendItem color="bg-rose-600" label="Flagged events" />
      </div>
    </div>
  );
}

function LidarScanPanel({
  scenario,
  activeFrame,
}: {
  scenario: Scenario;
  activeFrame: LidarPlaybackFrame | null;
}) {
  const lidarArtifact = scenario.artifacts.test["lidar.npz"];
  if (!activeFrame) {
    return (
      <Card className="border-slate-800 bg-slate-900/60 p-4">
        <p className="text-sm text-slate-400">LiDAR frame-by-frame scan data is unavailable for this run.</p>
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

  return (
    <div>
      <div className="mb-3 flex items-center justify-between gap-2">
        <p className="text-xs uppercase tracking-wide text-slate-400">LiDAR frame-by-frame sensor view (rays only)</p>
        {lidarArtifact && (
          <a href={lidarArtifact} target="_blank" rel="noreferrer" className="text-xs text-cyan-300 underline-offset-2 hover:underline">
            Open lidar.npz
          </a>
        )}
      </div>
      <Card className="border-slate-800 bg-slate-900/60 p-3">
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

            <line x1={0} y1={0} x2={viewRange * 0.35} y2={0} stroke="#60a5fa" strokeWidth={0.06} opacity={0.95} />
            <circle cx={0} cy={0} r={0.15} fill="#60a5fa" />
          </g>
        </svg>
      </Card>
      <p className="mt-2 text-xs text-slate-500">
        Sensor-frame scan: center is LiDAR origin, +X is robot forward. No map geometry is rendered in this panel.
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
          <p className="mb-3 text-xs uppercase tracking-wide text-slate-400">Single Test Run Files</p>
          <div className="space-y-2">
            {testEntries.map(([key, href]) => (
              <ArtifactLink key={key} label={artifactLabels[key] ?? key} href={href} />
            ))}
          </div>
        </div>

        <div>
          <p className="mb-3 text-xs uppercase tracking-wide text-slate-400">Batch Run Files</p>
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
        Open
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

function LegendItem({ color, label }: { color: string; label: string }) {
  return (
    <div className="flex items-center gap-2 text-xs text-slate-300">
      <span className={`inline-block h-2.5 w-2.5 rounded-full ${color}`} />
      {label}
    </div>
  );
}

export default App;
