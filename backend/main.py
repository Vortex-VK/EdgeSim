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

ROOT = Path(__file__).resolve().parents[1]


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
        # Launch the process; rely on the current env (where edgesim is installed)
        proc = subprocess.Popen(
            req.command,
            shell=True,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start process: {e}")

    JOBS[job_id] = proc
    return {"success": True, "job_id": job_id, "message": "started"}


@app.get("/api/simulate/{job_id}")
def simulate_status(job_id: str) -> Dict[str, Any]:
    """Check the status of a launched job."""
    proc = JOBS.get(job_id)
    if proc is None:
        raise HTTPException(status_code=404, detail="job not found")

    if proc.poll() is None:
        return {"status": "running", "job_id": job_id}

    stdout, stderr = proc.communicate()
    return {
        "status": "completed" if proc.returncode == 0 else "failed",
        "job_id": job_id,
        "return_code": proc.returncode,
        "output": stdout,
        "error": stderr,
    }
