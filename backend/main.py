from __future__ import annotations

import os
import subprocess
import uuid
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


# Simple job registry (in-memory)
JOBS: Dict[str, subprocess.Popen[str]] = {}
JOB_LOGS: Dict[str, Path] = {}
JOB_FILES: Dict[str, Any] = {}

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "runs" / "_job_logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)


class SimulationConfig(BaseModel):
    mode: str  # "test" or "batch"
    command: str  # Full CLI command (e.g., edgesim run-one "prompt" --seed 42 ...)
    config: Dict[str, Any]


app = FastAPI(title="EdgeSim API", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> Dict[str, str]:
    return {"status": "healthy"}


@app.post("/api/simulate")
def simulate(req: SimulationConfig) -> Dict[str, Any]:
	"""Start an EdgeSim run using the provided CLI command."""
	if req.mode not in {"test", "batch"}:
		raise HTTPException(status_code=400, detail="mode must be 'test' or 'batch'")

	job_id = str(uuid.uuid4())

	try:
		log_path = LOG_DIR / f"{job_id}.log"
		log_file = open(log_path, "w", encoding="utf-8")
		# Launch the process; rely on the current env (where edgesim is installed)
		proc = subprocess.Popen(
			req.command,
			shell=True,
			cwd=ROOT,
			stdout=log_file,
			stderr=subprocess.STDOUT,
			text=True,
		)
	except Exception as e:
		raise HTTPException(status_code=500, detail=f"Failed to start process: {e}")

	JOBS[job_id] = proc
	JOB_LOGS[job_id] = log_path
	JOB_FILES[job_id] = log_file
	return {"success": True, "job_id": job_id, "message": "started", "log_path": str(log_path)}


@app.get("/api/simulate/{job_id}")
def simulate_status(job_id: str) -> Dict[str, Any]:
	"""Check the status of a launched job."""
	proc = JOBS.get(job_id)
	if proc is None:
		raise HTTPException(status_code=404, detail="job not found")

	log_path = JOB_LOGS.get(job_id)
	if proc.poll() is None:
		return {"status": "running", "job_id": job_id, "log_path": str(log_path) if log_path else None}

	# Process finished; avoid blocking on large output by not piping in-memory.
	proc.communicate()
	log_file = JOB_FILES.pop(job_id, None)
	if log_file:
		try:
			log_file.close()
		except Exception:
			pass
	return {
		"status": "completed" if proc.returncode == 0 else "failed",
		"job_id": job_id,
		"return_code": proc.returncode,
		"log_path": str(log_path) if log_path else None,
	}
